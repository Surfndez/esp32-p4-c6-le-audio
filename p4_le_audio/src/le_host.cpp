// LE Audio host for ESP32-P4: HCI over esp_hosted SDIO, L2CAP, ATT/GATT client and local GATT server, SMP LE Secure
// Connections, CSIP set member join. Taken from the BT HAL of the firmware this project was extracted from; the protocol
// paths are unchanged, only the HAL / settings / UI glue was replaced (le_port.h) and diagnostics-only code was removed.
//
// Bluetooth LE / LE Audio over HCI H4 on SDIO (ESP32-P4 <-> ESP32-C6).
//
// The ESP32-C6 coprocessor running esp_hosted 1.4.1 firmware exposes a BLE 5 controller
// (Extended Advertising, Central role, LE Data Length) on the SDIO ESP_HCI_IF channel.
// The C6 controller has NO isochronous channels (CIS/BIS): LE Audio streaming
// does not work, and ISO commands are sent only when the controller reports the CIS Central
// bit (HCI_LE_Read_Local_Supported_Features). This module implements a lightweight, direct
// HCI Central stack + L2CAP + ATT/GATT Client + SMP LE Secure Connections (Just Works
// P-256 ECDH + AES-CMAC f5/f6) + inspection of LE Audio profiles over GATT (PACS, ASCS,
// CAS, VCS, Battery) without the memory cost of a full Bluedroid in SRAM.
// Everything that comes over the air (advertisements, HCI events, ACL/L2CAP, ATT, SMP) is
// untrusted data: every length and offset is checked before reading and copying.

#include "le_bt_types.h"
#include "le_host_internal.h"
#include "le_port.h"
#include <bt/bt_bond_model.h>
#include <bt/csip_model.h>
#include <bt/le_audio_model.h>

#include <algorithm>
#include <cstdio>
#include <cstring>
#include <mutex>
#include <new>
#include <string>
#include <vector>

#include "esp_attr.h"
#include "esp_hosted.h"
#include "esp_hosted_cyberdeck.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_random.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "mbedtls/aes.h"
#include "mbedtls/ecdh.h"
#include "mbedtls/ecp.h"

namespace {

constexpr const char* kTag = "hal_bt";

using BtState       = lehost::BtState;
using BtError       = lehost::BtError;
using BtDevice_t    = lehost::BtDevice_t;
using BtStatus_t    = lehost::BtStatus_t;
using BtGattInfo_t  = lehost::BtGattInfo_t;
using BtGattService = lehost::BtGattService_t;

struct AdvRecord {
    uint8_t addr[6]      = {};
    uint8_t addrType     = 0;
    int8_t rssi          = -127;
    bool extAdv          = false;
    bool leAudio         = false;
    bool audioCandidate  = false;
    uint16_t mfgId       = 0xFFFF;
    uint8_t mfgData[14]  = {};
    uint8_t mfgLen       = 0;
    uint16_t appearance  = 0;
    uint16_t uuids16[12] = {};
    uint8_t nUuids16     = 0;
    char name[40]        = {};
    bool hasRsi          = false;  // RSI (AD 0x2E): CSIP set member, recognized by its SIRK
    uint8_t rsi[6]       = {};
    int16_t adFlags      = -1;     // AD Flags (0x01); bit 0 = LE Limited Discoverable, -1 = none
};

struct BtEngine {
    std::mutex stateLock;
    // Recursive: btReconnect checks for a free link and connects (btConnect) under one lock
    std::recursive_mutex opLock;
    std::mutex txLock;  // serializes SDIO HCI writes from the onHciRx and btConnect threads

    uint8_t lastPlacementL = 0;
    uint8_t lastPlacementR = 0;
    uint8_t lastRawSamsung[24] = {};
    uint8_t lastRawSamsungLen  = 0;

    bool initialized       = false;
    bool radioOn           = true;
    bool cisCentral        = false;  // controller reports LE feature bit 28 (CIS Central); the C6 does not
    bool isoSkipLogged     = false;
    BtStatus_t status      = {};
    uint8_t ownRandAddr[6] = {0x55, 0x44, 0x33, 0x22, 0x11, 0xC6};
    // Identity used when connecting and pairing (setting bt.ident, at radio start): address and its HCI type (0 public,
    // 1 random) and whether we hand out our IRK. The old variant (fixed random + IRK without private addresses) was not
    // remembered by Galaxy Buds - every reconnect ended with 0x06 (Tab5 2026-11-01, docs/c6-link-layer.md).
    uint8_t ownAddr[6]  = {0x55, 0x44, 0x33, 0x22, 0x11, 0xC6};
    uint8_t ownAddrType = 1;
    bool ownIdKey       = true;
    // Background connection (btReconnect, under opLock): saved key only, without erasing it and without pairing
    bool bondOnly     = false;
    bool bondRejected = false;  // in this connection the earbuds rejected the saved key (0x06)

    // Waiting for a synchronous HCI response (0x0E Command Complete / 0x0F Command Status)
    uint16_t waitOpcode = 0;
    bool cmdDone        = false;
    uint8_t cmdStatus   = 0xFF;
    uint8_t cmdRet[252] = {};  // max Return_Parameters without status; C6 diagnostics 0xFC30 returns up to ~200 B
    uint8_t cmdRetLen   = 0;

    // Waiting for the LE connection to be established (0x3E sub 0x01 / 0x0A)
    bool connDone       = false;
    uint8_t connStatus  = 0xFF;
    uint16_t connHandle = 0xFFFF;

    // L2CAP reassembly and ATT (CID 0x0004) / SMP (CID 0x0006) response
    // Per connection: with two ACLs (main + second earbud, 27 B packets before DLE) fragments of one
    // must not end up in the PDU of the other
    struct L2capRx {
        uint16_t handle  = 0xFFFF;
        uint16_t expLen  = 0;  // 0 = no PDU in progress
        uint16_t cid     = 0;
        uint16_t curLen  = 0;
        uint8_t buf[260] = {};
    } l2rx[2];

    bool attWait      = false;
    bool attDone      = false;
    uint8_t attReqOp  = 0;  // opcode of the pending ATT request: we accept only its response or an error
    uint16_t attConn  = 0xFFFF;  // connection of the pending ATT request (two ACLs at once: only its response counts)
    uint8_t attRx[256] = {};
    uint16_t attRxLen = 0;

    // Queue of incoming SMP frames (CID 0x0006), because e.g. Public Key (0x0C) and Confirm (0x03) arrive back to back
    uint8_t smpQ[6][72] = {};
    uint16_t smpQLen[6] = {};
    int smpQHead        = 0;
    int smpQTail        = 0;
    uint16_t smpConn    = 0xFFFF;  // connection being paired - only its SMP PDUs go to the queue

    // Connection encryption state (HCI_Encryption_Change 0x08 / Key_Refresh 0x30)
    bool encDone       = false;
    uint8_t encStatus  = 0xFF;
    uint8_t encEnabled = 0;
    uint16_t encConn   = 0xFFFF;  // connection on which startLinkEncryption waits for Encryption Change

    // Bonds of paired devices (LTK, IRK, identity address) in RAM / NVS - each earbud has its own
    app::btbond::BondList bonds;

    // Handles of notification characteristics in the earbuds (VCS / MICS / Battery / Samsung Vendor)
    uint16_t vcsStateHandle  = 0;
    uint16_t vcsCpHandle     = 0;
    uint8_t vcsChangeCounter = 0;
    uint16_t micsMuteHandle  = 0;
    bool muted               = false;
    uint8_t localMediaState  = 0x02;  // GMCS Media State: 0 Inactive, 1 Playing, 2 Paused (btSetMediaState)
    std::string localTrackTitle;     // GMCS Track Title (btSetMediaState)
    int32_t localTrackDurCs   = -1;  // GMCS Track Duration in 0.01 s, -1 = unknown
    uint8_t mediaStateNtfMask = 0;   // CCCD Media State enabled: bit 0 main connection, bit 1 second earbud
    lehost::BtMediaCmd mediaCmds[8] = {};  // MCP commands for the app (btTakeMediaCommand), FIFO queue
    uint8_t mediaCmdHead  = 0;
    uint8_t mediaCmdCount = 0;
    uint32_t cccdSubscribed  = 0;

    // ASCS (Audio Stream Control Service 0x184E) for LE Audio
    uint16_t ascsCpHandle         = 0;
    uint16_t ascsSinkAseHandle[2] = {0, 0};
    uint8_t  ascsSinkAseId[2]     = {0, 0};
    uint8_t  ascsSinkCount        = 0;
    uint8_t  ascsState            = 0;  // 0=Idle, 1=Codec, 2=QoS, 3=Enabling, 4=Streaming
    uint16_t ascsSrcAseHandle[2]  = {0, 0};  // Source ASE (0x2BC5) - earbud microphone (hal_le_audio.cpp)
    uint8_t  ascsSrcCount         = 0;
    uint16_t pacsStart = 0, pacsEnd = 0;
    // State of each ACL connection by handle (main + second set member, docs/c6-link-layer.md): HCI events
    // carry the handle, so the second connection does not overwrite the main one's data (linkMetaLocked)
    struct LinkMeta {
        uint16_t handle = 0xFFFF;
        uint16_t dleTx  = 0;      // LE Data Length Change (0x3E/0x07); 0 = no event
        uint16_t itvl   = 0;      // ACL interval (1.25 ms) from Connection Complete / Update Complete; 0 = unknown
        bool enc        = false;  // Encryption Change: enabled
    } linkMeta[4];
    uint16_t secondHandle = 0xFFFF;  // second set member (stereo); main = status.connHandle
    uint8_t secondPeer[6] = {};      // address the second member connected with (btForget)
    // CSIP set of the main connection (readCsis): SIRK decrypted with the link key - to recognize the second earbud
    // by the RSI in its advertisement. RAM only, read on every connection (Android also reads it after encryption).
    struct SetInfo {
        bool valid      = false;
        uint8_t sirk[16] = {};
        uint8_t size    = 0;
        uint8_t rank    = 0;
        uint16_t conn   = 0xFFFF;
        bool fresh      = false;  // main paired in this connection (not with a saved key) - set pairing session
    } set;
    bool setJoining = false;  // second earbud is being joined (joinSetMember) - LE Audio start waits for it

    // History of commands / gestures received from the earbuds
    uint32_t eventCount = 0;
    std::string lastEvent;
    std::vector<std::string> eventHistory;

    // Scan buffers
    AdvRecord scanDevs[64] = {};
    int nScanDevs          = 0;
    uint32_t advReports    = 0;
};

// ~8.5 kB of state (64 scan records, ATT/SMP/L2CAP buffers) in PSRAM instead of internal .bss (internal RAM has only
// a few tens of kB spare, Wi-Fi needs it for DMA buffers). noinit memory + construction during global constructors -
// PSRAM is already mapped by then (CONFIG_SPIRAM_BOOT_INIT). Tasks only, no interrupts.
EXT_RAM_NOINIT_ATTR alignas(BtEngine) uint8_t g_btMem[sizeof(BtEngine)];
BtEngine& g_bt = *new (g_btMem) BtEngine();

// State entry of an ACL connection by handle (under stateLock). create: free entry for a connection without one;
// nullptr = no entry or table full.
BtEngine::LinkMeta* linkMetaLocked(uint16_t h, bool create)
{
    if (h > 0x0EFF) {
        return nullptr;  // 0xFFFF = no connection, not a free entry
    }
    BtEngine::LinkMeta* empty = nullptr;
    for (auto& m : g_bt.linkMeta) {
        if (m.handle == h) {
            return &m;
        }
        if (empty == nullptr && m.handle == 0xFFFF) {
            empty = &m;
        }
    }
    if (!create || empty == nullptr) {
        return nullptr;
    }
    *empty        = BtEngine::LinkMeta{};
    empty->handle = h;
    return empty;
}

// ACL connection is alive: main or with a state entry (Connection Complete without Disconnection Complete).
bool aclUpLocked(uint16_t h)
{
    return h != 0xFFFF && (h == g_bt.status.connHandle || linkMetaLocked(h, false) != nullptr);
}

void dropLinkMetaLocked(uint16_t h)
{
    if (BtEngine::LinkMeta* m = linkMetaLocked(h, false)) {
        *m = BtEngine::LinkMeta{};
    }
    if (h == g_bt.secondHandle) {
        g_bt.secondHandle = 0xFFFF;
        g_bt.mediaStateNtfMask &= (uint8_t)~2u;
    }
    if (h == g_bt.status.connHandle) {
        g_bt.mediaStateNtfMask &= (uint8_t)~1u;
    }
    if (h == g_bt.set.conn) {
        g_bt.set = BtEngine::SetInfo{};
    }
}

void recordBtEventLocked(const std::string& msg)
{
    g_bt.eventCount++;
    g_bt.lastEvent = msg;
    g_bt.eventHistory.insert(g_bt.eventHistory.begin(), msg);
    if (g_bt.eventHistory.size() > 12) {
        g_bt.eventHistory.resize(12);
    }
    ESP_LOGI(kTag, "BT EVENT [#%lu]: %s", (unsigned long)g_bt.eventCount, msg.c_str());
}

// Bonds (address + LTK [+ IRK]) as hex in the secrets store - the old single-bond format is read as a list. Via
// GetHAL()->kv*, because the BT API is called from threads with PSRAM stacks (app::task::spawn), and raw nvs_* from
// such a stack = esp_task_stack_is_sane_cache_disabled assertion and restart. Address stored with the key: the LTK is
// used only for that device.
constexpr const char* kBondKey = "bt.bond";

void wipeBonds(app::btbond::BondList& l)
{
    for (auto& b : l.items) {
        std::memset(b.ltk, 0, sizeof(b.ltk));
        std::memset(b.irk, 0, sizeof(b.irk));
    }
}

void saveBondNvs(const app::btbond::BondList& l)
{
    std::string hex = app::btbond::formatBonds(l);
    if (!leport::kvWritable()) {
        ESP_LOGW(kTag, "Bond store is read-only - the bond list stays in RAM until reset (nothing written)");
    } else if (!leport::kvSet(kBondKey, hex)) {
        ESP_LOGW(kTag, "Could not save the BT pairing key (kv %s)", kBondKey);
    }
    std::fill(hex.begin(), hex.end(), '\0');
}

// Our IRK (Identity Resolving Key) handed out during pairing: generated once and kept in the secrets kv store, because
// the earbuds remember it together with the bond - a new IRK after a restart would make us look like another device.
constexpr const char* kIrkKey = "bt.irk";

// IRK generated in this session when the store could not keep it (read-only store): every pairing until reset
// hands out the same one, so both earbuds of a set get one identity.
bool g_irkRam = false;
uint8_t g_irkRamVal[16] = {};

void ownIrk(uint8_t out[16])
{
    if (g_irkRam) {
        std::memcpy(out, g_irkRamVal, 16);
        return;
    }
    std::string s;
    bool ok = leport::kvGet(kIrkKey, s) && s.size() == 32;
    for (size_t i = 0; ok && i < 16; i++) {
        unsigned v = 0;
        ok         = std::sscanf(s.c_str() + 2 * i, "%2x", &v) == 1;
        out[i]     = (uint8_t)v;
    }
    std::fill(s.begin(), s.end(), '\0');
    if (ok) {
        return;
    }
    esp_fill_random(out, 16);
    char hex[33] = {};
    for (size_t i = 0; i < 16; i++) {
        std::snprintf(&hex[2 * i], 3, "%02x", out[i]);
    }
    if (!leport::kvSet(kIrkKey, hex)) {
        ESP_LOGW(kTag, "Could not save IRK (kv %s) - after a restart the earbuds may not recognize us", kIrkKey);
        std::memcpy(g_irkRamVal, out, 16);
        g_irkRam = true;
    }
    std::memset(hex, 0, sizeof(hex));
}

void loadBondNvs()
{
    std::string s;
    app::btbond::BondList b;
    if (!leport::kvGet(kBondKey, s)) {
        return;
    }
    const bool ok = app::btbond::parseBonds(s, b);
    std::fill(s.begin(), s.end(), '\0');
    if (!ok) {
        ESP_LOGW(kTag, "Corrupt BT pairing key record (kv %s) - the earbuds will pair again", kBondKey);
        return;
    }
    {
        std::lock_guard<std::mutex> l(g_bt.stateLock);
        g_bt.bonds = b;
    }
    ESP_LOGI(kTag, "Bond: saved devices %u", (unsigned)app::btbond::bondCount(b));
    // Diagnostics without key material: which bonds can resolve a private address (IRK) and the identity address
    // tail (LSB first in the record -> printed MSB first like a MAC)
    for (size_t i = 0; i < app::btbond::kMaxBonds; i++) {
        const app::btbond::Bond& e = b.items[i];
        if (e.valid) {
            ESP_LOGI(kTag, "Bond %u: identity ..:..:..:..:%02X:%02X, IRK %s", (unsigned)i, e.addr[1], e.addr[0],
                     e.hasIrk ? "yes (resolves private addresses)" : "NO (matches only this exact address)");
        }
    }
    wipeBonds(b);
}

/* ----------------------- helper names of manufacturers and services ----------------------- */

const char* mfgNameFromId(uint16_t id)
{
    switch (id) {
        case 0x0075: return "Samsung";
        case 0x004C: return "Apple";
        case 0x00E0: return "Google";
        case 0x012D: return "Sony";
        case 0x009E: return "Bose";
        case 0x0057: return "Harman/JBL";
        case 0x038F: return "Xiaomi";
        case 0x0006: return "Microsoft";
        case 0x0059: return "Nordic";
        case 0x0087: return "Garmin";
        case 0x02E5: return "Espressif";
        case 0x000A: return "Qualcomm";
        case 0x0046: return "MediaTek";
        case 0x005D: return "Realtek";
        case 0x0171: return "Amazon";
        case 0x0310: return "Nothing";
        case 0x0157: return "Huawei";
        default: return "";
    }
}

const char* gattServiceName(uint16_t uuid16)
{
    switch (uuid16) {
        case 0x1800: return "Generic Access (GAP)";
        case 0x1801: return "Generic Attribute (GATT)";
        case 0x180A: return "Device Information";
        case 0x180F: return "Battery Service";
        case 0x1844: return "VCS · Volume Control (LE Audio)";
        case 0x1846: return "CSIS · Coordinated Set (LE Audio)";
        case 0x1848: return "MCS · Media Control (LE Audio)";
        case 0x1849: return "GMCS · Generic Media Control";
        case 0x184B: return "TBS · Telephone Bearer";
        case 0x184C: return "GTBS · Generic Telephone Bearer";
        case 0x184D: return "MICS · Microphone Control (LE Audio)";
        case 0x184E: return "ASCS · Audio Stream Control (LE Audio)";
        case 0x184F: return "BASS · Broadcast Audio Scan (LE Audio)";
        case 0x1850: return "PACS · Published Audio Capabilities (LE Audio)";
        case 0x1853: return "CAS · Common Audio Service (LE Audio)";
        case 0x1854: return "HAP · Hearing Access (LE Audio)";
        case 0x1855: return "TMAP · Telephony & Media Audio (LE Audio)";
        case 0x1856: return "PBP · Public Broadcast Profile";
        case 0xFD69: return "Samsung Galaxy Buds Service";
        case 0xFE2C: return "Google Fast Pair";
        default: return "GATT service";
    }
}

bool isLeAudioUuid(uint16_t u)
{
    return u == 0x184E || u == 0x1850 || u == 0x1853 || u == 0x1854 || u == 0x1855 || u == 0x184F || u == 0x1844 ||
           u == 0x1846 || u == 0x184D;
}

bool isAudioCandidateUuid(uint16_t u)
{
    return isLeAudioUuid(u) || u == 0xFD69 || u == 0xFE2C || u == 0x110B;
}

void formatMac(const uint8_t addr[6], char out[20])
{
    std::snprintf(out, 20, "%02X:%02X:%02X:%02X:%02X:%02X", addr[5], addr[4], addr[3], addr[2], addr[1], addr[0]);
}

bool parseMac(const std::string& str, uint8_t addr[6])
{
    unsigned int b[6] = {};
    if (std::sscanf(str.c_str(), "%02x:%02x:%02x:%02x:%02x:%02x", &b[5], &b[4], &b[3], &b[2], &b[1], &b[0]) != 6) {
        return false;
    }
    for (int i = 0; i < 6; i++) {
        addr[i] = (uint8_t)b[i];
    }
    return true;
}

std::string hexBytes(const uint8_t* data, size_t len, size_t maxBytes = 24)
{
    std::string out;
    const size_t n = std::min(len, maxBytes);
    char buf[8];
    for (size_t i = 0; i < n; i++) {
        std::snprintf(buf, sizeof(buf), "%s%02X", i ? " " : "", data[i]);
        out += buf;
    }
    if (len > maxBytes) {
        out += " …";
    }
    return out;
}

/* --------------------- decoding BLE advertisements (Extended and Legacy) --------------------- */

void addUuid16(AdvRecord& d, uint16_t u)
{
    if (isLeAudioUuid(u)) {
        d.leAudio        = true;
        d.audioCandidate = true;
    } else if (isAudioCandidateUuid(u)) {
        d.audioCandidate = true;
    }
    for (uint8_t i = 0; i < d.nUuids16; i++) {
        if (d.uuids16[i] == u) {
            return;
        }
    }
    if (d.nUuids16 < 12) {
        d.uuids16[d.nUuids16++] = u;
    }
}

void parseAdData(AdvRecord& d, const uint8_t* data, uint8_t dlen)
{
    uint8_t pos = 0;
    while (pos < dlen) {
        const uint8_t adLen = data[pos];
        if (adLen == 0 || (uint32_t)pos + 1u + adLen > dlen) {
            break;
        }
        const uint8_t adType = data[pos + 1];
        const uint8_t* adVal = &data[pos + 2];
        const uint8_t valLen = adLen - 1;

        if (adType == 0x01 && valLen >= 1) {
            d.adFlags = adVal[0];
        }
        if ((adType == 0x08 || adType == 0x09) && valLen > 0) {
            const size_t copyLen = std::min<size_t>(valLen, sizeof(d.name) - 1);
            if (adType == 0x09 || d.name[0] == '\0' || copyLen > std::strlen(d.name)) {
                std::memcpy(d.name, adVal, copyLen);
                d.name[copyLen] = '\0';
                for (size_t i = 0; i < copyLen; i++) {
                    if ((uint8_t)d.name[i] < 0x20 || (uint8_t)d.name[i] > 0x7E) {
                        d.name[i] = '?';
                    }
                }
            }
        } else if (adType == 0x02 || adType == 0x03) {
            for (uint8_t k = 0; k + 1 < valLen; k += 2) {
                const uint16_t u = (uint16_t)adVal[k] | ((uint16_t)adVal[k + 1] << 8);
                addUuid16(d, u);
            }
        } else if (adType == 0x16 && valLen >= 2) {
            const uint16_t u = (uint16_t)adVal[0] | ((uint16_t)adVal[1] << 8);
            addUuid16(d, u);
        } else if (adType == 0x19 && valLen >= 2) {
            d.appearance       = (uint16_t)adVal[0] | ((uint16_t)adVal[1] << 8);
            const uint16_t cat = d.appearance >> 6;
            if (cat == 0x0021 || cat == 0x0025) {
                d.audioCandidate = true;
            }
        } else if (adType == 0xFF && valLen >= 2) {
            d.mfgId  = (uint16_t)adVal[0] | ((uint16_t)adVal[1] << 8);
            d.mfgLen = std::min<uint8_t>(valLen - 2, sizeof(d.mfgData));
            if (d.mfgLen > 0) {
                std::memcpy(d.mfgData, &adVal[2], d.mfgLen);
            }
            if (d.mfgId == 0x0075 && d.mfgLen >= 2 && d.mfgData[0] == 0x42) {
                d.audioCandidate = true;
            }
        }
        pos = (uint8_t)(pos + 1u + adLen);
    }
    if (d.name[0] != '\0') {
        if (std::strstr(d.name, "Buds") != nullptr || std::strstr(d.name, "Audio") != nullptr ||
            std::strstr(d.name, "Ear") != nullptr || std::strstr(d.name, "Head") != nullptr ||
            std::strstr(d.name, "WF-") != nullptr || std::strstr(d.name, "WH-") != nullptr ||
            std::strstr(d.name, "JBL") != nullptr || std::strstr(d.name, "AirPods") != nullptr) {
            d.audioCandidate = true;
        }
    }
}

void recordAdvLocked(uint8_t addrType, const uint8_t addr[6], int8_t rssi, bool extAdv, const uint8_t* data,
                     uint8_t dlen)
{
    g_bt.advReports++;
    int idx = -1;
    for (int i = 0; i < g_bt.nScanDevs; i++) {
        if (std::memcmp(g_bt.scanDevs[i].addr, addr, 6) == 0) {
            idx = i;
            break;
        }
    }
    if (idx < 0) {
        if (g_bt.nScanDevs >= 64) {
            // Table full (Tab5 2026-11-02 in a crowd: 64 devices in 8 s, mostly phones): an earbud advertisement
            // (LE Audio services or set RSI) replaces the weakest non-audio device - otherwise the second earbud
            // never made it into the table and joining it did not see it
            AdvRecord t;
            parseAdData(t, data, dlen);
            if (!t.leAudio && !app::csip::findRsi(data, dlen, t.rsi)) {
                return;
            }
            for (int i = 0; i < g_bt.nScanDevs; i++) {
                const AdvRecord& r = g_bt.scanDevs[i];
                if (!r.audioCandidate && !r.hasRsi && (idx < 0 || r.rssi < g_bt.scanDevs[idx].rssi)) {
                    idx = i;
                }
            }
            if (idx < 0) {
                return;
            }
        } else {
            idx = g_bt.nScanDevs++;
        }
        g_bt.scanDevs[idx] = AdvRecord{};
        std::memcpy(g_bt.scanDevs[idx].addr, addr, 6);
        g_bt.scanDevs[idx].addrType = addrType & 0x01;
        g_bt.scanDevs[idx].rssi     = rssi;
    } else {
        if (rssi > g_bt.scanDevs[idx].rssi && rssi != 127) {
            g_bt.scanDevs[idx].rssi = rssi;
        }
    }
    if (extAdv) {
        g_bt.scanDevs[idx].extAdv = true;
    }
    parseAdData(g_bt.scanDevs[idx], data, dlen);
    if (!g_bt.scanDevs[idx].hasRsi && app::csip::findRsi(data, dlen, g_bt.scanDevs[idx].rsi)) {
        g_bt.scanDevs[idx].hasRsi = true;
    }
}

/* --------------------- sending ACL L2CAP and receiving HCI H4 --------------------- */

bool sendAclL2cap(uint16_t connHandle, uint16_t cid, const uint8_t* payload, uint16_t plen)
{
    if (connHandle == 0xFFFF || plen > 248) {
        return false;
    }
    uint8_t pkt[260]      = {};
    const uint16_t aclLen = (uint16_t)(4 + plen);
    pkt[0]                = 0x02;  // HCI_H4_ACL
    pkt[1]                = (uint8_t)(connHandle & 0xFF);
    pkt[2]                = (uint8_t)((connHandle >> 8) & 0x0F);  // PB = 00 (First non-auto-flushable), BC = 00
    pkt[3]                = (uint8_t)(aclLen & 0xFF);
    pkt[4]                = (uint8_t)((aclLen >> 8) & 0xFF);
    pkt[5]                = (uint8_t)(plen & 0xFF);
    pkt[6]                = (uint8_t)((plen >> 8) & 0xFF);
    pkt[7]                = (uint8_t)(cid & 0xFF);
    pkt[8]                = (uint8_t)((cid >> 8) & 0xFF);
    if (plen > 0 && payload != nullptr) {
        std::memcpy(&pkt[9], payload, plen);
    }
    {
        std::lock_guard<std::mutex> l(g_bt.stateLock);
        g_bt.status.txPkts++;
    }
    std::lock_guard<std::mutex> tl(g_bt.txLock);
    return esp_hosted_cyberdeck_hci_tx(pkt, (uint16_t)(5 + aclLen)) == 0;
}

struct PendingL2capReply {
    struct Pkt {
        uint16_t handle   = 0xFFFF;
        uint16_t cid      = 0;
        uint16_t len      = 0;
        uint8_t data[128] = {};
    };
    Pkt pkts[4];
    int count = 0;
};

void queueReply(PendingL2capReply& r, uint16_t handle, uint16_t cid, const uint8_t* data, uint16_t len)
{
    if (r.count >= 4 || len > sizeof(r.pkts[0].data)) {
        return;
    }
    auto& p  = r.pkts[r.count++];
    p.handle = handle;
    p.cid    = cid;
    p.len    = len;
    std::memcpy(p.data, data, len);
}

/* --------- Local GATT server on Tab5 (GMCS 0x1849 + GTBS 0x184C for earbud gestures) --------- */

struct LocalGattAttr {
    uint16_t handle;
    uint16_t uuid16;
    uint8_t prop;        // used for 0x2803 (Characteristic Declaration)
    uint16_t valHandle;  // used for 0x2803
    uint16_t valUuid16;  // used for 0x2803 or 0x2800 (endHandle in valHandle)
};

// Local GATT handles - the table, reads, writes and notifications use these names (previously numbers in several
// places). GMCS has the full set of mandatory MCS characteristics (Media Player Name, Track Changed / Title / Duration
// / Position, Media State, CCID) and the optional ones that reference servers always register (Playback / Seeking
// Speed, Playing Order(s)): the MCP client in the earbuds may reject an incomplete service, and then gestures do not
// arrive (Tab5 2026-11-01: clicks did not arrive, volume did - that is VCS in the earbuds, not our GMCS).
namespace gatt_h {
constexpr uint16_t kSvcChangedCccd = 0x0007;
constexpr uint16_t kGmcs           = 0x0010;
constexpr uint16_t kPlayerName     = 0x0012;
constexpr uint16_t kTrackChanged   = 0x0015;
constexpr uint16_t kTrackTitle     = 0x0018;
constexpr uint16_t kTrackDuration  = 0x001B;
constexpr uint16_t kTrackPosition  = 0x001E;
constexpr uint16_t kPlaybackSpeed  = 0x0021;
constexpr uint16_t kSeekingSpeed   = 0x0024;
constexpr uint16_t kPlayingOrder   = 0x0027;
constexpr uint16_t kPlayingOrders  = 0x002A;
constexpr uint16_t kMediaState     = 0x002C;
constexpr uint16_t kMediaStateCccd = 0x002D;
constexpr uint16_t kMediaCp        = 0x002F;
constexpr uint16_t kMediaCpOps     = 0x0032;
constexpr uint16_t kGmcsCcidH      = 0x0035;
constexpr uint16_t kGtbs           = 0x0040;
constexpr uint16_t kCallState      = 0x0042;
constexpr uint16_t kCallCp         = 0x0045;
constexpr uint16_t kCallCpOpt      = 0x0048;
constexpr uint16_t kGtbsCcidH      = 0x004A;
}  // namespace gatt_h
using namespace gatt_h;

// Characteristic properties (Core Vol 3 Part G 3.3.1.1)
constexpr uint8_t kPropR = 0x02, kPropWnr = 0x04, kPropW = 0x08, kPropN = 0x10;
constexpr uint8_t kPropRwN = kPropR | kPropW | kPropWnr | kPropN;

constexpr LocalGattAttr kLocalGatt[] = {
    // 0x1800 Generic Access (0x0001..0x0003)
    {0x0001, 0x2800, 0, 0x0003, 0x1800},
    {0x0002, 0x2803, kPropR, 0x0003, 0x2A00},
    {0x0003, 0x2A00, 0, 0, 0},
    // 0x1801 Generic Attribute (0x0004..0x0007)
    {0x0004, 0x2800, 0, kSvcChangedCccd, 0x1801},
    {0x0005, 0x2803, 0x20, 0x0006, 0x2A05},
    {0x0006, 0x2A05, 0, 0, 0},
    {kSvcChangedCccd, 0x2902, 0, 0, 0},
    // 0x1849 GMCS - Generic Media Control Service (MCS 1.0, section 3)
    {kGmcs, 0x2800, 0, kGmcsCcidH, 0x1849},
    {kPlayerName - 1, 0x2803, kPropR | kPropN, kPlayerName, 0x2B93},  // Media Player Name
    {kPlayerName, 0x2B93, 0, 0, 0},
    {kPlayerName + 1, 0x2902, 0, 0, 0},
    {kTrackChanged - 1, 0x2803, kPropN, kTrackChanged, 0x2B96},  // Track Changed (notification only)
    {kTrackChanged, 0x2B96, 0, 0, 0},
    {kTrackChanged + 1, 0x2902, 0, 0, 0},
    {kTrackTitle - 1, 0x2803, kPropR | kPropN, kTrackTitle, 0x2B97},  // Track Title
    {kTrackTitle, 0x2B97, 0, 0, 0},
    {kTrackTitle + 1, 0x2902, 0, 0, 0},
    {kTrackDuration - 1, 0x2803, kPropR | kPropN, kTrackDuration, 0x2B98},  // Track Duration (0.01 s)
    {kTrackDuration, 0x2B98, 0, 0, 0},
    {kTrackDuration + 1, 0x2902, 0, 0, 0},
    {kTrackPosition - 1, 0x2803, kPropRwN, kTrackPosition, 0x2B99},  // Track Position
    {kTrackPosition, 0x2B99, 0, 0, 0},
    {kTrackPosition + 1, 0x2902, 0, 0, 0},
    {kPlaybackSpeed - 1, 0x2803, kPropRwN, kPlaybackSpeed, 0x2B9A},  // Playback Speed
    {kPlaybackSpeed, 0x2B9A, 0, 0, 0},
    {kPlaybackSpeed + 1, 0x2902, 0, 0, 0},
    {kSeekingSpeed - 1, 0x2803, kPropR | kPropN, kSeekingSpeed, 0x2B9B},  // Seeking Speed
    {kSeekingSpeed, 0x2B9B, 0, 0, 0},
    {kSeekingSpeed + 1, 0x2902, 0, 0, 0},
    {kPlayingOrder - 1, 0x2803, kPropRwN, kPlayingOrder, 0x2BA1},  // Playing Order
    {kPlayingOrder, 0x2BA1, 0, 0, 0},
    {kPlayingOrder + 1, 0x2902, 0, 0, 0},
    {kPlayingOrders - 1, 0x2803, kPropR, kPlayingOrders, 0x2BA2},  // Playing Orders Supported
    {kPlayingOrders, 0x2BA2, 0, 0, 0},
    {kMediaState - 1, 0x2803, kPropR | kPropN, kMediaState, 0x2BA3},  // Media State
    {kMediaState, 0x2BA3, 0, 0, 0},
    {kMediaStateCccd, 0x2902, 0, 0, 0},
    {kMediaCp - 1, 0x2803, kPropW | kPropWnr | kPropN, kMediaCp, 0x2BA4},  // Media Control Point
    {kMediaCp, 0x2BA4, 0, 0, 0},
    {kMediaCp + 1, 0x2902, 0, 0, 0},
    {kMediaCpOps - 1, 0x2803, kPropR | kPropN, kMediaCpOps, 0x2BA5},  // Media Control Point Opcodes Supported
    {kMediaCpOps, 0x2BA5, 0, 0, 0},
    {kMediaCpOps + 1, 0x2902, 0, 0, 0},
    {kGmcsCcidH - 1, 0x2803, kPropR, kGmcsCcidH, 0x2BBA},  // Content Control ID (CCID)
    {kGmcsCcidH, 0x2BBA, 0, 0, 0},
    // 0x184C GTBS - Generic Telephone Bearer Service
    {kGtbs, 0x2800, 0, kGtbsCcidH, 0x184C},
    {kCallState - 1, 0x2803, kPropR | kPropN, kCallState, 0x2BBD},  // Call State
    {kCallState, 0x2BBD, 0, 0, 0},
    {kCallState + 1, 0x2902, 0, 0, 0},
    {kCallCp - 1, 0x2803, kPropW | kPropWnr | kPropN, kCallCp, 0x2BBE},  // Call Control Point
    {kCallCp, 0x2BBE, 0, 0, 0},
    {kCallCp + 1, 0x2902, 0, 0, 0},
    {kCallCpOpt - 1, 0x2803, kPropR, kCallCpOpt, 0x2BBF},  // Call Control Point Optional Opcodes
    {kCallCpOpt, 0x2BBF, 0, 0, 0},
    {kGtbsCcidH - 1, 0x2803, kPropR, kGtbsCcidH, 0x2BBA},  // Content Control ID (CCID)
    {kGtbsCcidH, 0x2BBA, 0, 0, 0},
};

bool isLocalCccd(uint16_t h)
{
    for (const auto& a : kLocalGatt) {
        if (a.handle == h) {
            return a.uuid16 == 0x2902;
        }
    }
    return false;
}

void putLe32(uint8_t* o, uint32_t v)
{
    o[0] = (uint8_t)v;
    o[1] = (uint8_t)(v >> 8);
    o[2] = (uint8_t)(v >> 16);
    o[3] = (uint8_t)(v >> 24);
}

// Opcodes Supported (MCS 3.20): only what the player really does - Play, Pause, Stop, Previous / Next Track
constexpr uint32_t kMcpOpsSupported = (1u << 0) | (1u << 1) | (1u << 4) | (1u << 11) | (1u << 12);

uint16_t readLocalAttrValue(uint16_t handle, uint8_t* out, uint16_t maxLen)
{
    auto putStr = [&](const char* str) {
        const uint16_t l = (uint16_t)std::min<size_t>(std::strlen(str), maxLen);
        std::memcpy(out, str, l);
        return l;
    };
    switch (handle) {
        case 0x0003:  // GAP Device Name
            return putStr(leport::config().deviceName.c_str());
        case 0x0006: {  // Service Changed
            const uint8_t v[4] = {0x01, 0x00, 0xFF, 0xFF};
            std::memcpy(out, v, 4);
            return 4;
        }
        case kPlayerName:
            return putStr(leport::config().deviceName.c_str());
        case kTrackChanged:  // notification only
            return 0;
        case kTrackTitle:
            return putStr(g_bt.localTrackTitle.c_str());
        case kTrackDuration:  // int32 in 0.01 s, -1 = unknown
            putLe32(out, g_bt.localTrackDurCs < 0 ? 0xFFFFFFFFu : (uint32_t)g_bt.localTrackDurCs);
            return 4;
        case kTrackPosition:  // we do not track position - "unavailable" (MCS 3.7)
            putLe32(out, 0xFFFFFFFFu);
            return 4;
        case kPlaybackSpeed:  // int8, 0 = 1x
        case kSeekingSpeed:   // int8, 0 = no seeking
            out[0] = 0;
            return 1;
        case kPlayingOrder:  // 0x03 = in order, once (the player queue ends at the last track)
            out[0] = 0x03;
            return 1;
        case kPlayingOrders:  // bit 2 = in order, once
            out[0] = 0x04;
            out[1] = 0x00;
            return 2;
        case kMediaState:  // 0 Inactive, 1 Playing, 2 Paused (app: btSetMediaState)
            out[0] = g_bt.localMediaState;
            return 1;
        case kMediaCpOps:
            putLe32(out, kMcpOpsSupported);
            return 4;
        case kGmcsCcidH:
            out[0] = kGmcsCcid;
            return 1;
        case kCallState:  // no active call
            return 0;
        case kCallCpOpt:
            out[0] = 0x03;
            out[1] = 0x00;
            return 2;
        case kGtbsCcidH:
            out[0] = kGtbsCcid;
            return 1;
        default: {  // CCCD descriptors 0x2902
            out[0] = 0x01;
            out[1] = 0x00;
            return 2;
        }
    }
}

// MCP command from the earbuds: to the app queue (btTakeMediaCommand - the player runs in the UI thread), result
// immediately (MCS 3.20.2). The player state (Media State) is reported later by the app via btSetMediaState - after a
// real change.
void handleMcpWriteLocked(uint16_t connHandle, const uint8_t* val, uint16_t vlen, PendingL2capReply& reply)
{
    using MC          = lehost::BtMediaCmd;
    const uint8_t cmd = vlen >= 1 ? val[0] : 0;
    bool known        = true;
    MC mc             = MC::Play;
    const char* what  = "";
    switch (cmd) {
        case 0x01: mc = MC::Play, what = "Click: Play (MCP Play)"; break;
        case 0x02: mc = MC::Pause, what = "Click: Pause (MCP Pause)"; break;
        case 0x05: mc = MC::Stop, what = "Click: Stop (MCP Stop)"; break;
        case 0x30: mc = MC::Previous, what = "3× Click: Previous track (MCP Previous Track)"; break;
        case 0x31: mc = MC::Next, what = "2× Click: Next track (MCP Next Track)"; break;
        default: known = false; break;
    }
    uint8_t result = 0x02;  // Opcode Not Supported
    if (known) {
        constexpr uint8_t kCap = sizeof(g_bt.mediaCmds) / sizeof(g_bt.mediaCmds[0]);
        if (g_bt.mediaCmdCount < kCap) {
            g_bt.mediaCmds[(g_bt.mediaCmdHead + g_bt.mediaCmdCount) % kCap] = mc;
            g_bt.mediaCmdCount++;
        }
        result = 0x01;  // Success
        recordBtEventLocked(what);
    } else {
        char buf[64];
        std::snprintf(buf, sizeof(buf), "Media command from the earbuds (MCP 0x%02X - unsupported)", cmd);
        recordBtEventLocked(buf);
    }
    const uint8_t cpNtf[5] = {0x1B, (uint8_t)(kMediaCp & 0xFF), (uint8_t)(kMediaCp >> 8), cmd, result};
    queueReply(reply, connHandle, 0x0004, cpNtf, sizeof(cpNtf));
}

void handleCcpWriteLocked(uint16_t connHandle, const uint8_t* val, uint16_t vlen, PendingL2capReply& reply)
{
    const uint8_t cmd = vlen >= 1 ? val[0] : 0;
    switch (cmd) {
        case 0x00: recordBtEventLocked("Click: Answer call (Accept · CCP 0x00)"); break;
        case 0x01: recordBtEventLocked("Click: End call (Terminate · CCP 0x01)"); break;
        case 0x02: recordBtEventLocked("Click: Hold call (Hold · CCP 0x02)"); break;
        default: {
            char buf[64];
            std::snprintf(buf, sizeof(buf), "Call command from the earbuds (CCP 0x%02X)", cmd);
            recordBtEventLocked(buf);
            break;
        }
    }
    const uint8_t cpNtf[6] = {0x1B, (uint8_t)(kCallCp & 0xFF), (uint8_t)(kCallCp >> 8), cmd, 0x01, 0x00};
    queueReply(reply, connHandle, 0x0004, cpNtf, sizeof(cpNtf));
}

void handleLocalGattServerLocked(uint16_t connHandle, const uint8_t* pdu, uint16_t plen, PendingL2capReply& reply)
{
    const uint8_t op  = pdu[0];
    const uint8_t hLo = plen >= 2 ? pdu[1] : 0x01;
    const uint8_t hHi = plen >= 3 ? pdu[2] : 0x00;

    // Discovery of our services by the earbuds (several requests per connection): without it we cannot tell whether the
    // MCP client looks for GMCS at all (gestures, docs/c6-link-layer.md - test plan)
    if (op == 0x10 && plen >= 7) {
        ESP_LOGI(kTag, "Local GATT: earbuds 0x%03X look for services from 0x%02X%02X", connHandle, pdu[2], pdu[1]);
    } else if (op == 0x06 && plen >= 9) {
        ESP_LOGI(kTag, "Local GATT: earbuds 0x%03X look for service 0x%02X%02X", connHandle, pdu[8], pdu[7]);
    }

    if (op == 0x10 && plen >= 7) {
        // ATT_READ_BY_GROUP_TYPE_REQ
        const uint16_t sH   = (uint16_t)pdu[1] | ((uint16_t)pdu[2] << 8);
        const uint16_t eH   = (uint16_t)pdu[3] | ((uint16_t)pdu[4] << 8);
        const uint16_t uuid = (uint16_t)pdu[5] | ((uint16_t)pdu[6] << 8);
        if (uuid == 0x2800) {
            uint8_t rsp[64] = {0x11, 6};
            uint16_t rlen   = 2;
            for (const auto& a : kLocalGatt) {
                if (a.uuid16 == 0x2800 && a.handle >= sH && a.handle <= eH && rlen + 6 <= sizeof(rsp)) {
                    rsp[rlen + 0] = (uint8_t)(a.handle & 0xFF);
                    rsp[rlen + 1] = (uint8_t)(a.handle >> 8);
                    rsp[rlen + 2] = (uint8_t)(a.valHandle & 0xFF);
                    rsp[rlen + 3] = (uint8_t)(a.valHandle >> 8);
                    rsp[rlen + 4] = (uint8_t)(a.valUuid16 & 0xFF);
                    rsp[rlen + 5] = (uint8_t)(a.valUuid16 >> 8);
                    rlen          = (uint16_t)(rlen + 6);
                }
            }
            if (rlen > 2) {
                queueReply(reply, connHandle, 0x0004, rsp, rlen);
                return;
            }
        }
    } else if (op == 0x06 && plen >= 9) {
        // ATT_FIND_BY_TYPE_VALUE_REQ (searching for service 0x2800 by UUID)
        const uint16_t sH      = (uint16_t)pdu[1] | ((uint16_t)pdu[2] << 8);
        const uint16_t eH      = (uint16_t)pdu[3] | ((uint16_t)pdu[4] << 8);
        const uint16_t type    = (uint16_t)pdu[5] | ((uint16_t)pdu[6] << 8);
        const uint16_t valUuid = (uint16_t)pdu[7] | ((uint16_t)pdu[8] << 8);
        if (type == 0x2800) {
            for (const auto& a : kLocalGatt) {
                if (a.uuid16 == 0x2800 && a.handle >= sH && a.handle <= eH && a.valUuid16 == valUuid) {
                    const uint8_t rsp[5] = {0x07, (uint8_t)(a.handle & 0xFF), (uint8_t)(a.handle >> 8),
                                            (uint8_t)(a.valHandle & 0xFF), (uint8_t)(a.valHandle >> 8)};
                    queueReply(reply, connHandle, 0x0004, rsp, sizeof(rsp));
                    return;
                }
            }
        }
    } else if (op == 0x08 && plen >= 7) {
        // ATT_READ_BY_TYPE_REQ
        const uint16_t sH   = (uint16_t)pdu[1] | ((uint16_t)pdu[2] << 8);
        const uint16_t eH   = (uint16_t)pdu[3] | ((uint16_t)pdu[4] << 8);
        const uint16_t uuid = (uint16_t)pdu[5] | ((uint16_t)pdu[6] << 8);
        if (uuid == 0x2803) {
            uint8_t rsp[96] = {0x09, 7};
            uint16_t rlen   = 2;
            for (const auto& a : kLocalGatt) {
                if (a.uuid16 == 0x2803 && a.handle >= sH && a.handle <= eH && rlen + 7 <= sizeof(rsp)) {
                    rsp[rlen + 0] = (uint8_t)(a.handle & 0xFF);
                    rsp[rlen + 1] = (uint8_t)(a.handle >> 8);
                    rsp[rlen + 2] = a.prop;
                    rsp[rlen + 3] = (uint8_t)(a.valHandle & 0xFF);
                    rsp[rlen + 4] = (uint8_t)(a.valHandle >> 8);
                    rsp[rlen + 5] = (uint8_t)(a.valUuid16 & 0xFF);
                    rsp[rlen + 6] = (uint8_t)(a.valUuid16 >> 8);
                    rlen          = (uint16_t)(rlen + 7);
                }
            }
            if (rlen > 2) {
                queueReply(reply, connHandle, 0x0004, rsp, rlen);
                return;
            }
        } else {
            for (const auto& a : kLocalGatt) {
                if (a.uuid16 == uuid && a.handle >= sH && a.handle <= eH) {
                    uint8_t rsp[64]     = {0x09, 0, (uint8_t)(a.handle & 0xFF), (uint8_t)(a.handle >> 8)};
                    const uint16_t vlen = readLocalAttrValue(a.handle, &rsp[4], 56);
                    rsp[1]              = (uint8_t)(2 + vlen);
                    queueReply(reply, connHandle, 0x0004, rsp, (uint16_t)(4 + vlen));
                    return;
                }
            }
        }
    } else if (op == 0x04 && plen >= 5) {
        // ATT_FIND_INFORMATION_REQ
        const uint16_t sH = (uint16_t)pdu[1] | ((uint16_t)pdu[2] << 8);
        const uint16_t eH = (uint16_t)pdu[3] | ((uint16_t)pdu[4] << 8);
        uint8_t rsp[96]   = {0x05, 0x01};
        uint16_t rlen     = 2;
        for (const auto& a : kLocalGatt) {
            if (a.handle >= sH && a.handle <= eH && rlen + 4 <= sizeof(rsp)) {
                rsp[rlen + 0] = (uint8_t)(a.handle & 0xFF);
                rsp[rlen + 1] = (uint8_t)(a.handle >> 8);
                rsp[rlen + 2] = (uint8_t)(a.uuid16 & 0xFF);
                rsp[rlen + 3] = (uint8_t)(a.uuid16 >> 8);
                rlen          = (uint16_t)(rlen + 4);
            }
        }
        if (rlen > 2) {
            queueReply(reply, connHandle, 0x0004, rsp, rlen);
            return;
        }
    } else if (op == 0x0A && plen >= 3) {
        // ATT_READ_REQ
        const uint16_t h = (uint16_t)pdu[1] | ((uint16_t)pdu[2] << 8);
        for (const auto& a : kLocalGatt) {
            if (a.handle == h) {
                uint8_t rsp[64]     = {0x0B};
                const uint16_t vlen = readLocalAttrValue(h, &rsp[1], 60);
                queueReply(reply, connHandle, 0x0004, rsp, (uint16_t)(1 + vlen));
                return;
            }
        }
    } else if ((op == 0x12 || op == 0x52) && plen >= 3) {
        // ATT_WRITE_REQ (0x12) or ATT_WRITE_CMD (0x52) from the earbuds
        const uint16_t h       = (uint16_t)pdu[1] | ((uint16_t)pdu[2] << 8);
        const uint8_t* val     = &pdu[3];
        const uint16_t valLen  = (uint16_t)(plen - 3);
    if (op == 0x12) {
        const uint8_t wrRsp[1] = {0x13};
        queueReply(reply, connHandle, 0x0004, wrRsp, sizeof(wrRsp));
    }
    ESP_LOGI(kTag, "Local GATT write h=0x%04X op=0x%02X len=%u", h, op, (unsigned)valLen);
    if (h == kMediaCp) {
        handleMcpWriteLocked(connHandle, val, valLen, reply);
    } else if (h == kCallCp) {
        handleCcpWriteLocked(connHandle, val, valLen, reply);
    } else if (h == kMediaStateCccd) {
        // The earbuds want notifications about the player state - btSetMediaState sends them only then
        const uint8_t bit = connHandle == g_bt.status.connHandle ? 1 : 2;
        if (valLen >= 1 && (val[0] & 0x01) != 0) {
            g_bt.mediaStateNtfMask |= bit;
        } else {
            g_bt.mediaStateNtfMask &= (uint8_t)~bit;
        }
    } else if (h == kTrackPosition || h == kPlaybackSpeed || h == kPlayingOrder) {
        // Write accepted and ignored: we do not change position, speed or order (MCS allows rejecting silently)
    } else if (!isLocalCccd(h)) {
        char buf[80];
        std::snprintf(buf, sizeof(buf), "GATT write from the earbuds [0x%04X]: %s", h,
                      hexBytes(val, valLen, 8).c_str());
        recordBtEventLocked(buf);
    }
    return;
}

const uint8_t errRsp[5] = {0x01, op, hLo, hHi, 0x0A};  // Attribute Not Found
queueReply(reply, connHandle, 0x0004, errRsp, sizeof(errRsp));
}

void handlePeerNotificationLocked(uint16_t attrHandle, const uint8_t* val, uint16_t vlen)
{
ESP_LOGI(kTag, "NTF h=0x%04X len=%u: %s", attrHandle, (unsigned)vlen, hexBytes(val, vlen, 32).c_str());
if ((attrHandle == g_bt.vcsStateHandle || (attrHandle >= 101 && attrHandle <= 109 && vlen == 3)) && vlen >= 2) {
    const int newVol  = val[0];
    const bool newMut = (val[1] != 0);
    const int prevVol = g_bt.status.gatt.volume;
    const bool prevM  = g_bt.muted;
    const int pct     = (newVol * 100 + 127) / 255;

    g_bt.status.gatt.volume = newVol;
    g_bt.muted              = newMut;
    if (vlen >= 3) {
        g_bt.vcsChangeCounter = val[2];
    }

    char buf[96];
    if (prevVol >= 0 && newVol > prevVol) {
        std::snprintf(buf, sizeof(buf), "Louder (Swipe ↑) · %d%% (%d/255)", pct, newVol);
    } else if (prevVol >= 0 && newVol < prevVol) {
        std::snprintf(buf, sizeof(buf), "Quieter (Swipe ↓) · %d%% (%d/255)", pct, newVol);
    } else if (newMut != prevM) {
        std::snprintf(buf, sizeof(buf), "VCS mute: %s (%d%%)", newMut ? "ON" : "OFF", pct);
    } else {
        std::snprintf(buf, sizeof(buf), "VCS volume · %d%% (%d/255)", pct, newVol);
    }
    recordBtEventLocked(buf);
    return;
}

    if ((attrHandle == g_bt.micsMuteHandle || (attrHandle >= 97 && attrHandle <= 100)) && vlen >= 1) {
        g_bt.muted = (val[0] != 0);
        recordBtEventLocked(val[0] ? "Microphone MICS: Muted" : "Microphone MICS: Active");
        return;
    }

    if (g_bt.ascsCpHandle != 0 && attrHandle == g_bt.ascsCpHandle && vlen >= 4) {
        const uint8_t op   = val[0];
        const uint8_t id   = val[2];
        const uint8_t res  = val[3];
        const char* opName = (op == 0x01) ? "Config Codec" :
                             (op == 0x02) ? "Config QoS" :
                             (op == 0x03) ? "Enable" : "ASCS CP";
        char buf[80];
        std::snprintf(buf, sizeof(buf), "ASCS Rsp: %s ASE#%u -> %s (0x%02X)", opName, (unsigned)id,
                      (res == 0x00) ? "Success" : "Error", res);
        recordBtEventLocked(buf);
        return;
    }

    if ((attrHandle == g_bt.ascsSinkAseHandle[0] || attrHandle == g_bt.ascsSinkAseHandle[1]) &&
        attrHandle != 0 && vlen >= 2) {
        const uint8_t id    = val[0];
        const uint8_t state = val[1];
        const char* stName  = (state == 0) ? "Idle" :
                              (state == 1) ? "Codec Configured" :
                              (state == 2) ? "QoS Configured" :
                              (state == 3) ? "Enabling" :
                              (state == 4) ? "Streaming" : "Disabling";
        char buf[80];
        std::snprintf(buf, sizeof(buf), "ASCS State ASE#%u: %s", (unsigned)id, stName);
        recordBtEventLocked(buf);
        return;
    }

if (attrHandle >= 41 && attrHandle <= 50 && vlen >= 1) {
    const int pct               = std::clamp<int>(val[0], 0, 100);
    const int prev              = g_bt.status.gatt.batteryPct;
    g_bt.status.gatt.batteryPct = pct;
    if (pct != prev && prev >= 0) {
        char buf[64];
        std::snprintf(buf, sizeof(buf), "Earbud battery level: %d%%", pct);
        recordBtEventLocked(buf);
    }
    return;
}

if (attrHandle >= 51 && attrHandle <= 58) {
    // Samsung Galaxy Buds frame (SPP/RFCOMM protocol tunneled over GATT)
    // val[0] = 0xFD (Preamble), val[1] = len, val[2] = seq, val[3] = MsgId (e.g. 0x75 or 0xF2)
    // val[4] = 'L' (0x4C) or 'R' (0x52)
    const char side      = (vlen >= 5 && val[4] == 0x4C) ? 'L' : ((vlen >= 5 && val[4] == 0x52) ? 'R' : '?');
    const char* sideName = (side == 'L') ? "Left" : ((side == 'R') ? "Right" : "Earbud");

    if (vlen >= 7 && val[0] == 0xFD) {
        const uint8_t msgId  = val[3];
        uint8_t& lastP = (side == 'L') ? g_bt.lastPlacementL : g_bt.lastPlacementR;

        // 1. Check for a change of the in-ear state (In-Ear / Out-of-Ear / In-Case)
        uint8_t currentPlacement = 0;
        if (msgId == 0x75 && vlen >= 8) {
            if (val[7] >= 1 && val[7] <= 4) {
                currentPlacement = val[7];
            }
        } else if (msgId == 0xF2) {
            if (vlen >= 7 && val[6] >= 1 && val[6] <= 4) {
                currentPlacement = val[6];
            } else if (vlen >= 9 && val[8] >= 1 && val[8] <= 4) {
                currentPlacement = val[8];
            }
        }

        if (currentPlacement != 0 && currentPlacement != lastP) {
            const char* stName = (currentPlacement == 1) ? "put in ear (In-Ear)" :
                                 (currentPlacement == 2) ? "taken out of ear (Out-of-Ear)" :
                                 (currentPlacement == 3) ? "put in case (In-Case)" : "closed in case";
            char buf[96];
            std::snprintf(buf, sizeof(buf), "Gesture (%s): %s", sideName, stName);
            recordBtEventLocked(buf);
            lastP = currentPlacement;
            return;
        }
        if (currentPlacement != 0) {
            lastP = currentPlacement;
        }

        // 2. Handling clicks and stem squeezes (Pinch / Touch)
        // MsgId 0xF2 is a touch / stem squeeze (Pinch) notification from the earbud
        if (msgId == 0xF2) {
            static uint32_t s_lastPinchMs = 0;
            static int s_pinchCount = 0;
            const uint32_t nowMs = (uint32_t)(esp_timer_get_time() / 1000);

            if (nowMs - s_lastPinchMs <= 500) {
                s_pinchCount++;
            } else {
                s_pinchCount = 1;
            }
            s_lastPinchMs = nowMs;

            char buf[96];
            if (s_pinchCount == 1) {
                std::snprintf(buf, sizeof(buf), "1× Pinch (Click) · %s", sideName);
            } else if (s_pinchCount == 2) {
                std::snprintf(buf, sizeof(buf), "2× Pinch (Next) · %s", sideName);
            } else if (s_pinchCount >= 3) {
                std::snprintf(buf, sizeof(buf), "3× Pinch (Previous) · %s", sideName);
            }
            recordBtEventLocked(buf);
            return;
        }

        // 3. Other Samsung notifications (e.g. 0x61, 0x75, other statuses)
        char buf[96];
        std::snprintf(buf, sizeof(buf), "Samsung event (%s) [0x%02X]: %s", sideName, msgId,
                      hexBytes(val, vlen, 8).c_str());
        recordBtEventLocked(buf);
        return;
    }

    // Any other notification on Samsung Vendor handles 51..58
    char buf[96];
    std::snprintf(buf, sizeof(buf), "Samsung event [0x%04X]: %s", attrHandle,
                  hexBytes(val, vlen, 8).c_str());
    recordBtEventLocked(buf);
    return;
}
}

// LE Audio events and ATT values are collected under the state lock in onHciRx and handed to hal_le_audio.cpp after it
// is released (like `reply`): the LE Audio module calls hal_bt commands, so its locks must not be taken under ours.
// Buffer in PSRAM (first ensureRadioReady) - internal memory is at a premium; SDIO receive thread only.
struct LeAudioForward {
    static constexpr int kEv = 4, kEvLen = 64, kAtt = 3, kAttLen = 96;
    uint8_t ev[kEv][kEvLen];
    uint16_t evLen[kEv];
    int nEv;
    struct Att {
        uint16_t conn, attr, len;
        uint8_t data[kAttLen];
    } att[kAtt];
    int nAtt;
};
LeAudioForward* g_leFwd = nullptr;

void forwardLeEventLocked(const uint8_t* ev, uint32_t len)
{
    LeAudioForward* f = g_leFwd;
    if (f == nullptr || f->nEv >= LeAudioForward::kEv || len > LeAudioForward::kEvLen) {
        ESP_LOGW(kTag, "LE Audio: HCI event 0x%02X (%lu B) dropped - queue full or too long", ev[0],
                 (unsigned long)len);
        return;
    }
    std::memcpy(f->ev[f->nEv], ev, len);
    f->evLen[f->nEv++] = (uint16_t)len;
}

void forwardLeAttLocked(uint16_t conn, uint16_t attr, const uint8_t* val, uint16_t len)
{
    LeAudioForward* f = g_leFwd;
    if (f == nullptr || f->nAtt >= LeAudioForward::kAtt || len > LeAudioForward::kAttLen) {
        ESP_LOGW(kTag, "LE Audio: ATT value 0x%04X (%u B) dropped - queue full or too long", attr,
                 (unsigned)len);
        return;
    }
    auto& a = f->att[f->nAtt++];
    a.conn  = conn;
    a.attr  = attr;
    a.len   = len;
    std::memcpy(a.data, val, len);
}

void handleCompleteL2capLocked(uint16_t connHandle, uint16_t cid, const uint8_t* pdu, uint16_t plen,
                               PendingL2capReply& reply)
{
    if (plen == 0) {
        return;
    }
    if (cid == 0x0004) {
        // ATT channel
        const uint8_t op = pdu[0];
        if (op == 0x02) {
            // The earbuds sent ATT_EXCHANGE_MTU_REQ -> we reply with ATT_EXCHANGE_MTU_RSP (247)
            const uint8_t rsp[3] = {0x03, 0xF7, 0x00};
            queueReply(reply, connHandle, 0x0004, rsp, sizeof(rsp));
            return;
        }
        if (op == 0x1D) {
            // ATT_HANDLE_VALUE_IND -> we confirm with ATT_HANDLE_VALUE_CFM and decode the event
            const uint8_t cfm[1] = {0x1E};
            queueReply(reply, connHandle, 0x0004, cfm, sizeof(cfm));
            if (plen >= 3) {
                const uint16_t attrH = (uint16_t)pdu[1] | ((uint16_t)pdu[2] << 8);
                if (halLeAudioOwnsAttr(attrH)) {
                    forwardLeAttLocked(connHandle, attrH, &pdu[3], (uint16_t)(plen - 3));
                } else {
                    handlePeerNotificationLocked(attrH, &pdu[3], (uint16_t)(plen - 3));
                }
            }
            return;
        }
        if (op == 0x1B) {
            // ATT_HANDLE_VALUE_NTF — notification from the earbuds (e.g. VCS volume change, battery, gesture).
            // ASE / ASE CP handles (known after GATT inspection) go to LE Audio, the rest as before (fixed ranges
            // 41-50 battery, 51-58 Samsung).
            if (plen >= 3) {
                const uint16_t attrH = (uint16_t)pdu[1] | ((uint16_t)pdu[2] << 8);
                if (halLeAudioOwnsAttr(attrH)) {
                    forwardLeAttLocked(connHandle, attrH, &pdu[3], (uint16_t)(plen - 3));
                } else {
                    handlePeerNotificationLocked(attrH, &pdu[3], (uint16_t)(plen - 3));
                }
            }
            return;
        }
        // If the earbuds query our GATT server (GMCS 0x1849 / GTBS 0x184C / GAP / GATT) or send an MCP/CCP command:
        if (op == 0x04 || op == 0x06 || op == 0x08 || op == 0x0A || op == 0x0C || op == 0x10 || op == 0x12 ||
            op == 0x16 || op == 0x18 || op == 0x52) {
            handleLocalGattServerLocked(connHandle, pdu, plen, reply);
            return;
        }
        // Response to our ATT client request (0x01, 0x03, 0x05, 0x07, 0x09, 0x0B, 0x11, 0x13): only an
        // opcode matching the request sent (a late response after a timeout will not reach another request)
        const bool rspMatches = connHandle == g_bt.attConn &&
                                ((op == (uint8_t)(g_bt.attReqOp + 1)) ||
                                 (op == 0x01 && plen >= 2 && pdu[1] == g_bt.attReqOp));
        if (g_bt.attWait && rspMatches) {
            g_bt.attRxLen = std::min<uint16_t>(plen, sizeof(g_bt.attRx));
            std::memcpy(g_bt.attRx, pdu, g_bt.attRxLen);
            g_bt.attDone = true;
            g_bt.attWait = false;
        }
    } else if (cid == 0x0005 && plen >= 2) {
        // LE L2CAP Signaling Channel: e.g. Connection Parameter Update Request (0x12)
        if (pdu[0] == 0x12) {
            // Rejected (0x0001): "accepted" commits the central to an LE Connection Update, which we do not do - the
            // interval stays a multiple of 10 ms (kAclIntervalUnits, CIS emulation in the C6).
            const uint8_t rsp[6] = {0x13, pdu[1], 0x02, 0x00, 0x01, 0x00};
            queueReply(reply, connHandle, 0x0005, rsp, sizeof(rsp));
        }
    } else if (cid == 0x0006) {
        // SMP channel (Security Manager Protocol) — to the FIFO queue only from the connection being paired: with two
        // earbuds the PDU of the main one must not reach the pairing of the second (nor its Pairing Failed abort it)
        const int nextTail = (g_bt.smpQTail + 1) % 6;
        if (connHandle == g_bt.smpConn && nextTail != g_bt.smpQHead) {
            const uint16_t copyLen      = std::min<uint16_t>(plen, sizeof(g_bt.smpQ[0]));
            g_bt.smpQLen[g_bt.smpQTail] = copyLen;
            std::memcpy(g_bt.smpQ[g_bt.smpQTail], pdu, copyLen);
            g_bt.smpQTail = nextTail;
        }
        if (pdu[0] == 0x0B) {
            // SMP_Security_Request from the earbuds
            g_bt.status.gatt.authRequired = true;
        }
    }
}

// Static, not on the stack: the callback runs only on the sdio_process_rx thread (4 kB stack, measured peak ~2.1 kB
// without BT). 540 B in PSRAM like g_bt (noinit + construction during global constructors), not in internal .data.
EXT_RAM_NOINIT_ATTR alignas(PendingL2capReply) uint8_t g_rxReplyMem[sizeof(PendingL2capReply)];
PendingL2capReply& g_rxReply = *new (g_rxReplyMem) PendingL2capReply();

void onHciRx(const uint8_t* data, uint32_t len)
{
    if (data == nullptr || len < 2) {
        return;
    }
    PendingL2capReply& reply = g_rxReply;
    reply.count = 0;
    if (data[0] == 0x05) {
        // HCI ISO packet (CIS stream from the earbuds): no state lock, straight to LE Audio
        halLeAudioOnIsoRx(data, len);
        return;
    }
    if (g_leFwd != nullptr) {
        g_leFwd->nEv  = 0;
        g_leFwd->nAtt = 0;
    }
    {
        std::lock_guard<std::mutex> l(g_bt.stateLock);
        g_bt.status.rxPkts++;

        // ACL packet (0x02)
        if (data[0] == 0x02 && len >= 5) {
            const uint16_t hdr    = (uint16_t)data[1] | ((uint16_t)data[2] << 8);
            const uint16_t handle = hdr & 0x0FFF;
            const uint8_t pb      = (hdr >> 12) & 0x03;
            const uint16_t aclLen = (uint16_t)data[3] | ((uint16_t)data[4] << 8);
            const uint32_t avail  = std::min<uint32_t>(len - 5, aclLen);

            // Reassembly entry of this connection; a new PDU takes a free one (no PDU in progress) or the first
            BtEngine::L2capRx* rx = nullptr;
            for (auto& r : g_bt.l2rx) {
                if (r.handle == handle) {
                    rx = &r;
                }
            }
            if (pb == 0x02 || pb == 0x00) {
                if (avail < 4) {
                    return;
                }
                for (auto& r : g_bt.l2rx) {
                    if (rx == nullptr && r.expLen == 0) {
                        rx = &r;
                    }
                }
                rx                   = rx != nullptr ? rx : &g_bt.l2rx[0];
                rx->handle           = handle;
                rx->expLen           = (uint16_t)data[5] | ((uint16_t)data[6] << 8);
                rx->cid              = (uint16_t)data[7] | ((uint16_t)data[8] << 8);
                const uint16_t chunk = (uint16_t)std::min<uint32_t>(avail - 4, sizeof(rx->buf));
                std::memcpy(rx->buf, &data[9], chunk);
                rx->curLen = chunk;
            } else if (pb == 0x01 && rx != nullptr && rx->expLen > 0) {
                const uint16_t space = (uint16_t)(sizeof(rx->buf) - rx->curLen);
                const uint16_t chunk = (uint16_t)std::min<uint32_t>(avail, space);
                if (chunk > 0) {
                    std::memcpy(&rx->buf[rx->curLen], &data[5], chunk);
                    rx->curLen = (uint16_t)(rx->curLen + chunk);
                }
            }
            if (rx != nullptr && rx->expLen > 0 && rx->curLen >= rx->expLen) {
                const uint16_t plen = std::min<uint16_t>(rx->expLen, sizeof(rx->buf));
                const uint16_t cid  = rx->cid;
                rx->expLen          = 0;
                handleCompleteL2capLocked(handle, cid, rx->buf, plen, reply);
            }
        } else if (data[0] == 0x04 && len >= 3) {
            // HCI event (H4 0x04). ISO (0x05) is handled above; a truncated ACL is dropped - previously an ISO packet
            // with byte 0x05 was read as Disconnection_Complete.
            const uint8_t* ev     = data + 1;
            const uint32_t evLen  = len - 1;
            const uint8_t evtCode = ev[0];
            const uint8_t plen    = ev[1];
            const uint8_t* params = &ev[2];
            const uint32_t avail  = std::min<uint32_t>(evLen - 2, plen);

            if (evtCode == 0x0E && avail >= 4) {
                // HCI_Command_Complete
                const uint16_t op = (uint16_t)params[1] | ((uint16_t)params[2] << 8);
                if (g_bt.waitOpcode != 0 && op == g_bt.waitOpcode) {
                    g_bt.cmdStatus = params[3];
                    g_bt.cmdRetLen = (uint8_t)std::min<uint32_t>(avail - 4, sizeof(g_bt.cmdRet));
                    if (g_bt.cmdRetLen > 0) {
                        std::memcpy(g_bt.cmdRet, &params[4], g_bt.cmdRetLen);
                    }
                    g_bt.cmdDone = true;
                }
            } else if (evtCode == 0x0F && avail >= 4) {
                // HCI_Command_Status
                const uint16_t op = (uint16_t)params[2] | ((uint16_t)params[3] << 8);
                if (g_bt.waitOpcode != 0 && op == g_bt.waitOpcode) {
                    g_bt.cmdStatus = params[0];
                    g_bt.cmdRetLen = 0;
                    g_bt.cmdDone   = true;
                }
            } else if (evtCode == 0x08 && avail >= 4) {
                // HCI_Encryption_Change
                const uint16_t h = ((uint16_t)params[1] | ((uint16_t)params[2] << 8)) & 0x0FFF;
                if (BtEngine::LinkMeta* m = linkMetaLocked(h, true)) {
                    m->enc = params[0] == 0 && params[3] != 0;
                }
                if (h == g_bt.encConn) {
                    g_bt.encStatus  = params[0];
                    g_bt.encEnabled = params[3];
                    g_bt.encDone    = true;
                }
            } else if (evtCode == 0x30 && avail >= 3) {
                // HCI_Encryption_Key_Refresh_Complete
                const uint16_t h = ((uint16_t)params[1] | ((uint16_t)params[2] << 8)) & 0x0FFF;
                if (BtEngine::LinkMeta* m = linkMetaLocked(h, true)) {
                    m->enc = params[0] == 0;
                }
                if (h == g_bt.encConn) {
                    g_bt.encStatus  = params[0];
                    g_bt.encEnabled = (params[0] == 0) ? 1 : 0;
                    g_bt.encDone    = true;
                }
            } else if (evtCode == 0x05 && avail >= 4) {
                // HCI_Disconnection_Complete
                const uint16_t h = ((uint16_t)params[1] | ((uint16_t)params[2] << 8)) & 0x0FFF;
                ESP_LOGW(kTag, "HCI Disconnection_Complete handle=0x%04X status=0x%02X reason=0x%02X", h, params[0],
                         avail >= 4 ? params[3] : 0xFF);
                forwardLeEventLocked(ev, 2 + avail);  // CIS or earbud ACL - LE Audio ends the stream
                if (params[0] == 0x00 && h == g_bt.secondHandle) {
                    ESP_LOGI(kTag, "Second set member disconnected (handle 0x%03X)", h);
                }
                if (params[0] == 0x00) {
                    dropLinkMetaLocked(h);
                }
                if (params[0] == 0x00 && h == g_bt.status.connHandle) {
                    g_bt.encEnabled  = 0;
                    g_bt.status.connHandle = 0xFFFF;
                    if (g_bt.status.state == BtState::Connected) {
                        g_bt.status.state = g_bt.radioOn ? BtState::Idle : BtState::Off;
                    }
                }
            } else if (evtCode == 0x13 && avail >= 1) {
                // Number_Of_Completed_Packets: ISO buffer credits (ACL is sent without flow control)
                forwardLeEventLocked(ev, 2 + avail);
            } else if (evtCode == 0x3E && avail >= 2) {
                const uint8_t sub = params[0];
                if (sub == 0x19) {
                    forwardLeEventLocked(ev, 2 + avail);  // LE_CIS_Established
                } else if (sub == 0x07 && avail >= 11) {
                    // LE_Data_Length_Change: CIS needs at least 36 B on ACL (LL_CIS_REQ without fragmentation)
                    const uint16_t h = ((uint16_t)params[1] | ((uint16_t)params[2] << 8)) & 0x0FFF;
                    if (BtEngine::LinkMeta* m = linkMetaLocked(h, true)) {
                        m->dleTx = (uint16_t)params[3] | ((uint16_t)params[4] << 8);
                    }
                }
                if ((sub == 0x01 || sub == 0x0A || sub == 0x29) && avail >= 11) {
                    // LE_Connection_Complete (0x01) or LE_Enhanced_Connection_Complete (0x0A / 0x29)
                    g_bt.connStatus = params[1];
                    g_bt.connHandle = ((uint16_t)params[2] | ((uint16_t)params[3] << 8)) & 0x0FFF;
                    g_bt.connDone   = true;
                    const uint32_t itv = sub == 0x01 ? 12u : 24u;  // Conn_Interval after addresses (0x0A/0x29: + 2 RPA)
                    if (g_bt.connStatus == 0) {
                        // New connection: entry from scratch (the handle may have belonged to a connection whose end
                        // was lost)
                        dropLinkMetaLocked(g_bt.connHandle);
                        if (BtEngine::LinkMeta* m = linkMetaLocked(g_bt.connHandle, true)) {
                            m->itvl = avail >= itv + 2 ? (uint16_t)(params[itv] | (params[itv + 1] << 8)) : 0;
                        }
                    }
                } else if (sub == 0x03 && avail >= 6 && params[1] == 0) {
                    // LE_Connection_Update_Complete
                    const uint16_t h = ((uint16_t)params[2] | ((uint16_t)params[3] << 8)) & 0x0FFF;
                    if (BtEngine::LinkMeta* m = linkMetaLocked(h, true)) {
                        m->itvl = (uint16_t)(params[4] | (params[5] << 8));
                    }
                } else if (sub == 0x06 && avail >= 11) {
                    // LE_Remote_Connection_Parameter_Request: the controller waits for the host's reply (without it the
                    // earbuds' LL procedure ends after 40 s by dropping the ACL). The command is sent from a thread -
                    // we hold stateLock here.
                    const app::leaudio::HciCmd r = app::leaudio::connParamRequestReply(params, avail);
                    if (r.opcode != 0) {
                        ESP_LOGI(kTag, "earbuds' request for connection parameters %u..%u -> 0x%04X",
                                 (unsigned)(params[3] | (params[4] << 8)), (unsigned)(params[5] | (params[6] << 8)),
                                 r.opcode);
                        leport::spawn("bt-connparam", 4 * 1024, [r] {
                            uint8_t st = 0xFF;
                            halBtHciCmd(r.opcode, r.p, r.len, 500, &st, nullptr, 0, nullptr);
                        });
                    }
                } else if (sub == 0x02) {
                    // LE_Advertising_Report (Legacy)
                    const uint8_t num = params[1];
                    uint32_t off      = 2;
                    for (uint8_t i = 0; i < num && off + 9 <= avail; i++) {
                        const uint8_t addrType = params[off + 1];
                        const uint8_t* addr    = &params[off + 2];
                        const uint8_t dlen     = params[off + 8];
                        if (off + 9u + dlen + 1u > avail) {
                            break;
                        }
                        const uint8_t* adData = &params[off + 9];
                        const int8_t rssi     = (int8_t)params[off + 9 + dlen];
                        recordAdvLocked(addrType, addr, rssi, false, adData, dlen);
                        off += 10u + dlen;
                    }
                } else if (sub == 0x0D) {
                    // LE_Extended_Advertising_Report (BT 5.0+ / LE Audio)
                    const uint8_t num = params[1];
                    uint32_t off      = 2;
                    for (uint8_t i = 0; i < num && off + 24 <= avail; i++) {
                        const uint8_t addrType = params[off + 2];
                        const uint8_t* addr    = &params[off + 3];
                        const int8_t rssi      = (int8_t)params[off + 13];
                        const uint8_t dlen     = params[off + 23];
                        if (off + 24u + dlen > avail) {
                            break;
                        }
                        const uint8_t* adData = &params[off + 24];
                        recordAdvLocked(addrType, addr, rssi, true, adData, dlen);
                        off += 24u + dlen;
                    }
                }
            }
        }
    }
    for (int i = 0; i < reply.count; i++) {
        sendAclL2cap(reply.pkts[i].handle, reply.pkts[i].cid, reply.pkts[i].data, reply.pkts[i].len);
    }
    if (g_leFwd != nullptr) {
        for (int i = 0; i < g_leFwd->nEv; i++) {
            halLeAudioOnHciEvent(g_leFwd->ev[i], g_leFwd->evLen[i]);
        }
        for (int i = 0; i < g_leFwd->nAtt; i++) {
            const auto& a = g_leFwd->att[i];
            halLeAudioOnAttValue(a.conn, a.attr, a.data, a.len);
        }
    }
}

bool sendHciCmd(uint16_t opcode, const uint8_t* params, uint8_t plen, int timeoutMs = 450)
{
    if (plen > 64) {
        return false;
    }
    uint8_t pkt[68] = {};
    pkt[0]          = 0x01;  // HCI_H4_CMD
    pkt[1]          = (uint8_t)(opcode & 0xFF);
    pkt[2]          = (uint8_t)((opcode >> 8) & 0xFF);
    pkt[3]          = plen;
    if (plen > 0 && params != nullptr) {
        std::memcpy(&pkt[4], params, plen);
    }
    {
        std::lock_guard<std::mutex> l(g_bt.stateLock);
        g_bt.waitOpcode = opcode;
        g_bt.cmdDone    = false;
        g_bt.cmdStatus  = 0xFF;
        g_bt.cmdRetLen  = 0;
        g_bt.status.txPkts++;
    }
    // waitOpcode is cleared on every exit: after returning, the caller reads cmdStatus/cmdRet without the lock, and
    // a late response (after a timeout) can no longer overwrite them or be attributed to the next command.
    bool txOk = false;
    {
        std::lock_guard<std::mutex> tl(g_bt.txLock);
        txOk = esp_hosted_cyberdeck_hci_tx(pkt, (uint16_t)(4 + plen)) == 0;
    }
    const int64_t deadline = esp_timer_get_time() + (int64_t)timeoutMs * 1000;
    while (txOk && esp_timer_get_time() < deadline) {
        {
            std::lock_guard<std::mutex> l(g_bt.stateLock);
            if (g_bt.cmdDone) {
                g_bt.waitOpcode = 0;
                return true;
            }
        }
        vTaskDelay(pdMS_TO_TICKS(5));
    }
    std::lock_guard<std::mutex> l(g_bt.stateLock);
    g_bt.waitOpcode = 0;
    return false;
}

// LE Audio on CIS emulation in the C6 (payload v5) is an experiment: without bt.leaudio enabled the P4 sends no ISO
// commands, so the CIS adapter in the C6 only passes through (v4 behavior) - a test-time switch, on Tab5 only.
bool leAudioEnabled()
{
    return true;  // the original firmware had an experimental bt.leaudio switch; this component exists for LE Audio
}

// ISO commands (CIS/CIG, Set Host Feature) only when the controller has CIS Central and LE Audio is enabled;
// otherwise one debug log and skip.
bool isoAllowed(const char* what)
{
    const bool enabled = leAudioEnabled();
    std::lock_guard<std::mutex> l(g_bt.stateLock);
    if (g_bt.cisCentral && enabled) {
        return true;
    }
    if (!g_bt.isoSkipLogged) {
        g_bt.isoSkipLogged = true;
        ESP_LOGD(kTag, "%s: skipping ISO commands (%s)",
                 enabled ? "Controller lacks CIS Central (LE feature bit 28)" : "LE Audio disabled (bt.leaudio)", what);
    }
    return false;
}

bool ensureRadioReady()
{
    {
        std::lock_guard<std::mutex> l(g_bt.stateLock);
        if (g_bt.initialized) {
            return true;
        }
    }
    if (g_leFwd == nullptr) {
        g_leFwd = static_cast<LeAudioForward*>(heap_caps_calloc(1, sizeof(LeAudioForward), MALLOC_CAP_SPIRAM));
    }
    esp_hosted_cyberdeck_set_hci_rx_cb(onHciRx);

    esp_hosted_coprocessor_fwver_t fwver = {};
    char fwBuf[32]                       = "1.4.1";
    if (esp_hosted_get_coprocessor_fwversion(&fwver) == ESP_OK) {
        std::snprintf(fwBuf, sizeof(fwBuf), "%lu.%lu.%lu", (unsigned long)fwver.major1, (unsigned long)fwver.minor1,
                      (unsigned long)fwver.patch1);
    }

    if (!sendHciCmd(0x0C03, nullptr, 0, 700) || g_bt.cmdStatus != 0) {
        std::lock_guard<std::mutex> l(g_bt.stateLock);
        g_bt.status.error     = BtError::RadioDown;
        g_bt.status.lastError = "C6 coprocessor did not answer HCI_Reset";
        return false;
    }

    char mac[20] = {};
    if (sendHciCmd(0x1009, nullptr, 0) && g_bt.cmdStatus == 0 && g_bt.cmdRetLen >= 6) {
        formatMac(g_bt.cmdRet, mac);
        std::memcpy(g_bt.ownRandAddr, g_bt.cmdRet, 6);
        g_bt.ownRandAddr[5] |= 0xC0;  // Static Random Address (upper 2 bits = 11)
        const int ident  = leport::config().identity;  // setting bt.ident
        g_bt.ownAddrType = ident == 0 ? 1 : 0;
        g_bt.ownIdKey    = ident != 1;
        std::memcpy(g_bt.ownAddr, ident == 0 ? g_bt.ownRandAddr : g_bt.cmdRet, 6);
        ESP_LOGI(kTag, "Identity: address %s (bt.ident %d), IRK %s", ident == 0 ? "fixed random" : "public", ident,
                 g_bt.ownIdKey ? "handed out" : "no");
    }

    // HCI_LE_Read_Local_Supported_Features: 8 B mask, bit 28 = Connected Isochronous Stream (Central)
    bool cisCentral = false;
    if (sendHciCmd(0x2003, nullptr, 0) && g_bt.cmdStatus == 0 && g_bt.cmdRetLen >= 8) {
        cisCentral = (g_bt.cmdRet[3] & 0x10) != 0;
    }
    {
        std::lock_guard<std::mutex> l(g_bt.stateLock);
        g_bt.cisCentral = cisCentral;
    }

    const uint8_t evtMask[8]      = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x3F};
    const uint8_t leEvtMask[8]    = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
    const uint8_t cisHostParam[2] = {32, 1};  // bit 32 = Connected Isochronous Stream (Host Support)
    const uint8_t defDle[4]       = {251, 0x00, 0x48, 0x08};
    sendHciCmd(0x0C01, evtMask, 8);
    sendHciCmd(0x2001, leEvtMask, 8);
    sendHciCmd(0x2005, g_bt.ownRandAddr, 6);
    sendHciCmd(0x2024, defDle, sizeof(defDle));
    if (isoAllowed("LE Set Host Feature")) {
        sendHciCmd(0x2074, cisHostParam, sizeof(cisHostParam));
    }
    loadBondNvs();

    {
        std::lock_guard<std::mutex> l(g_bt.stateLock);
        g_bt.initialized          = true;
        g_bt.status.coprocessorFw = fwBuf;
        if (mac[0] != '\0') {
            g_bt.status.bdAddr = mac;
        }
        if (g_bt.status.state == BtState::Off && g_bt.radioOn) {
            g_bt.status.state = BtState::Idle;
        }
        g_bt.status.error = BtError::None;
    }
    return true;
}

/* --------------------- decoding PACS records (LC3) --------------------- */

std::string decodePacRecords(const uint8_t* data, size_t len)
{
    if (data == nullptr || len < 7) {
        return "";
    }
    const uint8_t numRecs = data[0];
    size_t pos            = 1;
    std::string summary;

    for (uint8_t r = 0; r < numRecs && pos + 6 <= len; r++) {
        const uint8_t codingFmt  = data[pos];
        const uint16_t companyId = (uint16_t)data[pos + 1] | ((uint16_t)data[pos + 2] << 8);
        const uint16_t vendorId  = (uint16_t)data[pos + 3] | ((uint16_t)data[pos + 4] << 8);
        const uint8_t capLen     = data[pos + 5];
        pos += 6;
        if (pos + capLen > len) {
            break;
        }

        uint16_t freqMask  = 0;
        uint8_t durMask    = 0;
        uint8_t chanMask   = 0;
        uint16_t minOctets = 0;
        uint16_t maxOctets = 0;

        size_t cpos       = pos;
        const size_t cend = pos + capLen;
        while (cpos < cend) {
            const uint8_t ltvLen = data[cpos];
            if (ltvLen == 0 || cpos + 1 + ltvLen > cend) {
                break;
            }
            const uint8_t ltvType = data[cpos + 1];
            const uint8_t* val    = &data[cpos + 2];
            const uint8_t vlen    = ltvLen - 1;
            if (ltvType == 0x01 && vlen >= 2) {
                freqMask = (uint16_t)val[0] | ((uint16_t)val[1] << 8);
            } else if (ltvType == 0x02 && vlen >= 1) {
                durMask = val[0];
            } else if (ltvType == 0x03 && vlen >= 1) {
                chanMask = val[0];
            } else if (ltvType == 0x04 && vlen >= 4) {
                minOctets = (uint16_t)val[0] | ((uint16_t)val[1] << 8);
                maxOctets = (uint16_t)val[2] | ((uint16_t)val[3] << 8);
            }
            cpos += 1 + ltvLen;
        }
        pos += capLen;
        if (pos < len) {
            const uint8_t metaLen = data[pos];
            pos += 1 + metaLen;
        }

        std::string rec;
        if (codingFmt == 0x06) {
            rec = "LC3";
        } else {
            char vbuf[32];
            std::snprintf(vbuf, sizeof(vbuf), "Codec 0x%02X (%04X:%04X)", codingFmt, companyId, vendorId);
            rec = vbuf;
        }

        if (freqMask != 0) {
            struct FreqBit {
                uint16_t bit;
                const char* label;
            };
            static constexpr FreqBit kFreqs[] = {
                {1 << 0, "8"},  {1 << 2, "16"},   {1 << 4, "24"},   {1 << 5, "32"},
                {1 << 6, "44.1"}, {1 << 7, "48"}, {1 << 8, "88.2"}, {1 << 9, "96"},
            };
            std::string fs;
            for (const auto& f : kFreqs) {
                if (freqMask & f.bit) {
                    if (!fs.empty()) {
                        fs += "/";
                    }
                    fs += f.label;
                }
            }
            if (!fs.empty()) {
                rec += " " + fs + " kHz";
            }
        }
        if (durMask != 0) {
            std::string ds;
            if (durMask & 0x01) {
                ds += "7.5";
            }
            if (durMask & 0x02) {
                if (!ds.empty()) {
                    ds += "/";
                }
                ds += "10";
            }
            if (!ds.empty()) {
                rec += " · " + ds + " ms";
            }
        }
        if (maxOctets > 0) {
            char obuf[32];
            std::snprintf(obuf, sizeof(obuf), " · %u-%u B", (unsigned)minOctets, (unsigned)maxOctets);
            rec += obuf;
        }
        if (chanMask != 0) {
            if (chanMask & 0x02) {
                rec += " · 2ch";
            } else if (chanMask & 0x01) {
                rec += " · 1ch";
            }
        }
        if (!summary.empty()) {
            summary += "; ";
        }
        summary += rec;
    }
    return summary;
}

/* --------------------- ATT / GATT client requests --------------------- */

// rspCap = capacity of the caller's buffer: a response from the air is up to 256 B, and callers' buffers can be 16 B.
bool attRequestBuf(uint16_t connHandle, const uint8_t* req, uint16_t reqLen, uint8_t* rsp, uint16_t rspCap,
                   uint16_t& rspLen, int timeoutMs)
{
    rspLen = 0;
    if (reqLen == 0) {
        return false;
    }
    {
        std::lock_guard<std::mutex> l(g_bt.stateLock);
        g_bt.attDone  = false;
        g_bt.attRxLen = 0;
        g_bt.attReqOp = req[0];
        g_bt.attConn  = connHandle;
        g_bt.attWait  = true;
    }
    if (!sendAclL2cap(connHandle, 0x0004, req, reqLen)) {
        std::lock_guard<std::mutex> l(g_bt.stateLock);
        g_bt.attWait = false;
        return false;
    }
    const int64_t deadline = esp_timer_get_time() + (int64_t)timeoutMs * 1000;
    while (esp_timer_get_time() < deadline) {
        {
            std::lock_guard<std::mutex> l(g_bt.stateLock);
            if (g_bt.attDone) {
                rspLen = std::min<uint16_t>(g_bt.attRxLen, rspCap);
                std::memcpy(rsp, g_bt.attRx, rspLen);
                return true;
            }
            if (!aclUpLocked(connHandle)) {
                g_bt.attWait = false;
                return false;
            }
        }
        vTaskDelay(pdMS_TO_TICKS(5));
    }
    std::lock_guard<std::mutex> l(g_bt.stateLock);
    g_bt.attWait = false;
    return false;
}

template <size_t N>
bool attRequest(uint16_t connHandle, const uint8_t* req, uint16_t reqLen, uint8_t (&rsp)[N], uint16_t& rspLen,
                int timeoutMs = 800)
{
    static_assert(N <= 0xFFFF, "ATT buffer too large");
    return attRequestBuf(connHandle, req, reqLen, rsp, (uint16_t)N, rspLen, timeoutMs);
}

template <size_t N>
bool attReadByType(uint16_t connHandle, uint16_t startH, uint16_t endH, uint16_t uuid16, uint8_t (&outVal)[N],
                   uint16_t& outLen, uint16_t* outValueHandle = nullptr, bool* outAuthErr = nullptr)
{
    outLen = 0;
    const uint8_t req[7] = {
        0x08,  // ATT_READ_BY_TYPE_REQ
        (uint8_t)(startH & 0xFF),
        (uint8_t)(startH >> 8),
        (uint8_t)(endH & 0xFF),
        (uint8_t)(endH >> 8),
        (uint8_t)(uuid16 & 0xFF),
        (uint8_t)(uuid16 >> 8),
    };
    uint8_t rsp[256] = {};
    uint16_t rspLen  = 0;
    if (!attRequest(connHandle, req, sizeof(req), rsp, rspLen)) {
        return false;
    }
    if (rspLen >= 5 && rsp[0] == 0x01) {
        const uint8_t err = rsp[4];
        if ((err == 0x05 || err == 0x08 || err == 0x0F) && outAuthErr != nullptr) {
            *outAuthErr = true;
        }
        return false;
    }
    if (rspLen < 4 || rsp[0] != 0x09) {
        return false;
    }
    const uint8_t itemLen = rsp[1];
    if (itemLen < 2 || 2 + itemLen > rspLen) {
        return false;
    }
    const uint16_t valHandle = (uint16_t)rsp[2] | ((uint16_t)rsp[3] << 8);
    if (outValueHandle != nullptr) {
        *outValueHandle = valHandle;
    }
    // The value length is dictated by the device (up to 253 B) - clipped to the caller's buffer (e.g. 16 B for battery)
    outLen = (uint16_t)std::min<size_t>(itemLen - 2, N);
    if (outLen > 0) {
        std::memcpy(outVal, &rsp[4], outLen);
    }
    if (itemLen >= 20 && valHandle != 0) {
        const uint8_t rreq[3] = {0x0A, (uint8_t)(valHandle & 0xFF), (uint8_t)(valHandle >> 8)};
        uint8_t rrsp[256]     = {};
        uint16_t rrspLen      = 0;
        if (attRequest(connHandle, rreq, sizeof(rreq), rrsp, rrspLen) && rrspLen > 1 && rrsp[0] == 0x0B) {
            outLen = (uint16_t)std::min<size_t>(rrspLen - 1, N);
            std::memcpy(outVal, &rrsp[1], outLen);
        }
    }
    return true;
}

/* --------------------- BLE SMP LE Secure Connections cryptography (RFC 4493 + P-256) --------------------- */

void swapBuf(uint8_t* dst, const uint8_t* src, size_t len)
{
    for (size_t i = 0; i < len; i++) {
        dst[len - 1 - i] = src[i];
    }
}

void swapInPlace(uint8_t* buf, size_t len)
{
    for (size_t i = 0; i < len / 2; i++) {
        std::swap(buf[i], buf[len - 1 - i]);
    }
}

void shiftLeft128(const uint8_t in[16], uint8_t out[16])
{
    uint8_t ovf = 0;
    for (int i = 15; i >= 0; i--) {
        out[i] = (uint8_t)((in[i] << 1) | ovf);
        ovf    = (in[i] >> 7) & 1;
    }
}

bool aes128Be(const uint8_t keyBe[16], const uint8_t inBe[16], uint8_t outBe[16])
{
    mbedtls_aes_context ctx;
    mbedtls_aes_init(&ctx);
    const bool ok = (mbedtls_aes_setkey_enc(&ctx, keyBe, 128) == 0) &&
                    (mbedtls_aes_crypt_ecb(&ctx, MBEDTLS_AES_ENCRYPT, inBe, outBe) == 0);
    mbedtls_aes_free(&ctx);
    return ok;
}

bool aesCmacBe(const uint8_t keyBe[16], const uint8_t* msgBe, size_t len, uint8_t outBe[16])
{
    uint8_t zero[16] = {};
    uint8_t L[16]    = {};
    if (!aes128Be(keyBe, zero, L)) {
        return false;
    }
    uint8_t K1[16] = {}, K2[16] = {};
    shiftLeft128(L, K1);
    if (L[0] & 0x80) {
        K1[15] ^= 0x87;
    }
    shiftLeft128(K1, K2);
    if (K1[0] & 0x80) {
        K2[15] ^= 0x87;
    }

    const size_t nBlocks = (len == 0) ? 1 : (len + 15) / 16;
    const bool complete  = (len > 0) && ((len % 16) == 0);

    uint8_t mLast[16] = {};
    if (complete) {
        const uint8_t* lastPtr = msgBe + (nBlocks - 1) * 16;
        for (int i = 0; i < 16; i++) {
            mLast[i] = lastPtr[i] ^ K1[i];
        }
    } else {
        const size_t rem       = len % 16;
        const uint8_t* lastPtr = msgBe + (nBlocks - 1) * 16;
        uint8_t padded[16]     = {};
        if (rem > 0) {
            std::memcpy(padded, lastPtr, rem);
        }
        padded[rem] = 0x80;
        for (int i = 0; i < 16; i++) {
            mLast[i] = padded[i] ^ K2[i];
        }
    }

    uint8_t X[16] = {};
    uint8_t Y[16] = {};
    for (size_t b = 0; b + 1 < nBlocks; b++) {
        const uint8_t* blk = msgBe + b * 16;
        for (int i = 0; i < 16; i++) {
            Y[i] = X[i] ^ blk[i];
        }
        if (!aes128Be(keyBe, Y, X)) {
            return false;
        }
    }
    for (int i = 0; i < 16; i++) {
        Y[i] = X[i] ^ mLast[i];
    }
    return aes128Be(keyBe, Y, outBe);
}

bool smF5(const uint8_t dhKeyLe[32], const uint8_t n1Le[16], const uint8_t n2Le[16], uint8_t a1t,
          const uint8_t a1Le[6], uint8_t a2t, const uint8_t a2Le[6], uint8_t macKeyLe[16], uint8_t ltkLe[16])
{
    static constexpr uint8_t kSaltBe[16] = {0x6c, 0x88, 0x83, 0x91, 0xaa, 0xf5, 0xa5, 0x38,
                                            0x60, 0x37, 0x0b, 0xdb, 0x5a, 0x60, 0x83, 0xbe};
    uint8_t wsBe[32] = {};
    uint8_t tBe[16]  = {};
    swapBuf(wsBe, dhKeyLe, 32);
    if (!aesCmacBe(kSaltBe, wsBe, 32, tBe)) {
        return false;
    }
    uint8_t m[53] = {
        0x00,                    // counter
        0x62, 0x74, 0x6c, 0x65,  // "btle"
    };
    swapBuf(m + 5, n1Le, 16);
    swapBuf(m + 21, n2Le, 16);
    m[37] = a1t;
    swapBuf(m + 38, a1Le, 6);
    m[44] = a2t;
    swapBuf(m + 45, a2Le, 6);
    m[51] = 0x01;
    m[52] = 0x00;

    if (!aesCmacBe(tBe, m, sizeof(m), macKeyLe)) {
        return false;
    }
    swapInPlace(macKeyLe, 16);

    m[0] = 0x01;
    if (!aesCmacBe(tBe, m, sizeof(m), ltkLe)) {
        return false;
    }
    swapInPlace(ltkLe, 16);
    return true;
}

bool smF6(const uint8_t macKeyLe[16], const uint8_t n1Le[16], const uint8_t n2Le[16], const uint8_t rLe[16],
          const uint8_t iocapLe[3], uint8_t a1t, const uint8_t a1Le[6], uint8_t a2t, const uint8_t a2Le[6],
          uint8_t checkLe[16])
{
    uint8_t wsBe[16] = {};
    uint8_t m[65]    = {};
    swapBuf(wsBe, macKeyLe, 16);
    swapBuf(m, n1Le, 16);
    swapBuf(m + 16, n2Le, 16);
    swapBuf(m + 32, rLe, 16);
    swapBuf(m + 48, iocapLe, 3);
    m[51] = a1t;
    swapBuf(m + 52, a1Le, 6);
    m[58] = a2t;
    swapBuf(m + 59, a2Le, 6);
    if (!aesCmacBe(wsBe, m, sizeof(m), checkLe)) {
        return false;
    }
    swapInPlace(checkLe, 16);
    return true;
}

int espRngCb(void*, unsigned char* buf, size_t len)
{
    esp_fill_random(buf, len);
    return 0;
}

bool smGenKeypairP256(uint8_t pubXLe[32], uint8_t pubYLe[32], uint8_t privLe[32])
{
    mbedtls_ecp_keypair kp;
    mbedtls_ecp_keypair_init(&kp);
    bool ok            = false;
    uint8_t privBe[32] = {};
    uint8_t pubBe[65]  = {};
    size_t olen        = 0;

    if (mbedtls_ecp_gen_key(MBEDTLS_ECP_DP_SECP256R1, &kp, espRngCb, nullptr) == 0 &&
        mbedtls_mpi_write_binary(&kp.MBEDTLS_PRIVATE(d), privBe, 32) == 0 &&
        mbedtls_ecp_point_write_binary(&kp.MBEDTLS_PRIVATE(grp), &kp.MBEDTLS_PRIVATE(Q), MBEDTLS_ECP_PF_UNCOMPRESSED,
                                       &olen, pubBe, sizeof(pubBe)) == 0 &&
        olen == 65) {
        swapBuf(privLe, privBe, 32);
        swapBuf(pubXLe, &pubBe[1], 32);
        swapBuf(pubYLe, &pubBe[33], 32);
        ok = true;
    }
    mbedtls_ecp_keypair_free(&kp);
    return ok;
}

bool smGenDhkeyP256(const uint8_t peerXLe[32], const uint8_t peerYLe[32], const uint8_t privLe[32],
                    uint8_t dhKeyLe[32])
{
    mbedtls_ecp_group grp;
    mbedtls_ecp_point Q;
    mbedtls_mpi d, z;
    mbedtls_ecp_group_init(&grp);
    mbedtls_ecp_point_init(&Q);
    mbedtls_mpi_init(&d);
    mbedtls_mpi_init(&z);

    uint8_t pubBe[65]  = {0x04};
    uint8_t privBe[32] = {};
    uint8_t dhBe[32]   = {};
    swapBuf(&pubBe[1], peerXLe, 32);
    swapBuf(&pubBe[33], peerYLe, 32);
    swapBuf(privBe, privLe, 32);

    bool ok = false;
    if (mbedtls_ecp_group_load(&grp, MBEDTLS_ECP_DP_SECP256R1) == 0 &&
        mbedtls_ecp_point_read_binary(&grp, &Q, pubBe, 65) == 0 && mbedtls_ecp_check_pubkey(&grp, &Q) == 0 &&
        mbedtls_mpi_read_binary(&d, privBe, 32) == 0 &&
        mbedtls_ecdh_compute_shared(&grp, &z, &Q, &d, espRngCb, nullptr) == 0 &&
        mbedtls_mpi_write_binary(&z, dhBe, 32) == 0) {
        swapBuf(dhKeyLe, dhBe, 32);
        ok = true;
    }

    mbedtls_ecp_point_free(&Q);
    mbedtls_mpi_free(&d);
    mbedtls_mpi_free(&z);
    mbedtls_ecp_group_free(&grp);
    return ok;
}

const char* g_smpStage = "start";  // SMP pairing stage, for diagnosing failed pairings
uint8_t g_smpFailReason = 0;       // code from SMP_Pairing_Failed sent by the other side (0 = none arrived)

// Time to wait for the other side's response in a pairing step: the Core SMP limit (Vol 3 Part H 3.4, 30 s from the
// last PDU; Android SMP_WAIT_FOR_RSP_TOUT is also 30 s). With 1.5 s the second earbud, which was computing the DHKey
// with two connections, missed DHKey Check and pairing failed ("peer_dhkey_check", without Pairing Failed). A dropped
// link ends the wait.
constexpr int kSmpRspTimeoutMs = 30000;

bool waitSmpOpcode(uint8_t expectedOp, uint8_t* outPdu, uint16_t& outLen, int timeoutMs = kSmpRspTimeoutMs)
{
    const int64_t deadline = esp_timer_get_time() + (int64_t)timeoutMs * 1000;
    while (esp_timer_get_time() < deadline) {
        {
            std::lock_guard<std::mutex> l(g_bt.stateLock);
            if (!aclUpLocked(g_bt.smpConn)) {
                return false;
            }
            while (g_bt.smpQHead != g_bt.smpQTail) {
                const int idx       = g_bt.smpQHead;
                const uint16_t plen = g_bt.smpQLen[idx];
                const uint8_t op    = g_bt.smpQ[idx][0];
                g_bt.smpQHead       = (g_bt.smpQHead + 1) % 6;
                if (op == 0x05) {
                    g_smpFailReason = plen >= 2 ? g_bt.smpQ[idx][1] : 0xFF;  // SMP_Pairing_Failed (Core Vol 3 H 3.5.5)
                    return false;
                }
                if (op == expectedOp) {
                    outLen = plen;
                    std::memcpy(outPdu, g_bt.smpQ[idx], plen);
                    return true;
                }
            }
        }
        vTaskDelay(pdMS_TO_TICKS(5));
    }
    return false;
}

// Result of starting encryption: with a saved key we need to know whether the controller refused (e.g. 0x0C while
// another LL procedure is running), the earbuds did, or simply nothing arrived (2026-10-31: "failed" with no cause).
struct EncResult {
    bool ok       = false;
    uint8_t cmd   = 0xFF;  // Command Status 0x2019 status (0xFF = no controller response)
    uint8_t evt   = 0xFF;  // Encryption Change status (0xFF = did not arrive)
    bool dropped  = false; // connection dropped meanwhile
    int ms        = 0;
};

EncResult startLinkEncryption(uint16_t connHandle, const uint8_t ltkLe[16], int timeoutMs = 1800)
{
    EncResult r;
    uint8_t encCmd[28] = {};
    encCmd[0]          = (uint8_t)(connHandle & 0xFF);
    encCmd[1]          = (uint8_t)((connHandle >> 8) & 0x0F);
    std::memcpy(&encCmd[12], ltkLe, 16);

    {
        std::lock_guard<std::mutex> l(g_bt.stateLock);
        g_bt.encDone    = false;
        g_bt.encStatus  = 0xFF;
        g_bt.encEnabled = 0;
        g_bt.encConn    = connHandle;
    }
    const int64_t t0 = esp_timer_get_time();
    const bool sent  = sendHciCmd(0x2019, encCmd, sizeof(encCmd), 500);
    std::memset(encCmd, 0, sizeof(encCmd));
    if (sent) {
        std::lock_guard<std::mutex> l(g_bt.stateLock);
        r.cmd = g_bt.cmdStatus;
    }
    if (r.cmd != 0) {
        // 0x02 Unknown Connection Identifier: the link is gone (Tab5 2026-11-02: 0x3E while connecting the second
        // earbud), not a key refusal
        r.dropped = r.cmd == 0x02;
        return r;
    }
    const int64_t deadline = t0 + (int64_t)timeoutMs * 1000;
    while (esp_timer_get_time() < deadline) {
        {
            std::lock_guard<std::mutex> l(g_bt.stateLock);
            if (g_bt.encDone) {
                r.evt = g_bt.encStatus;
                r.ok  = g_bt.encStatus == 0 && g_bt.encEnabled != 0;
                break;
            }
            if (!aclUpLocked(connHandle)) {
                r.dropped = true;
                break;
            }
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    r.ms = (int)((esp_timer_get_time() - t0) / 1000);
    return r;
}

// Encrypting with the saved key right after connecting, before GATT - like a phone (HCI log 2026-11-01: LE Enable
// Encryption ~75 ms after connecting); faster reconnect. It did not help the left Buds3 Pro: its key from pairing
// without the right one connected works neither early nor later (docs/c6-link-layer.md).
enum class BondEnc { Unknown, Ok, Failed, Erased, Rejected };

// mayErase = false (background connection, second earbud with the key only): a rejected key stays (Rejected) - only a
// connection that is about to re-pair with the user present erases it
BondEnc encryptWithBond(uint16_t connHandle, const uint8_t peerAddr[6], uint8_t peerAddrType, bool mayErase = true)
{
    int idx              = -1;
    size_t saved         = 0;
    uint8_t savedLtk[16] = {};
    {
        std::lock_guard<std::mutex> l(g_bt.stateLock);
        // Key only for the device it was established with (another device will reject encryption and often drops the
        // link before we can pair) - also under a new private address when we have its IRK
        idx   = app::btbond::findBond(g_bt.bonds, peerAddrType, peerAddr, aes128Be);
        saved = app::btbond::bondCount(g_bt.bonds);
        if (idx >= 0) {
            std::memcpy(savedLtk, g_bt.bonds.items[idx].ltk, 16);
        }
    }
    if (idx < 0) {
        ESP_LOGI(kTag, "Bond: unknown device (saved %u) - pairing", (unsigned)saved);
        return BondEnc::Unknown;
    }
    // 3 s: right after connecting the earbuds ask for connection parameters, and the controller runs one LL procedure
    // at a time - 1 s was sometimes too short and pairing ran on a link with encryption start still in progress
    // (2026-10-31)
    EncResult r = startLinkEncryption(connHandle, savedLtk, 3000);
    if (r.cmd == 0x0C) {  // Command Disallowed: another LL procedure in progress - one retry
        vTaskDelay(pdMS_TO_TICKS(500));
        r = startLinkEncryption(connHandle, savedLtk, 3000);
    }
    std::memset(savedLtk, 0, sizeof(savedLtk));
    if (r.ok) {
        ESP_LOGI(kTag, "Bond: recognized (entry %d of %u), encrypted with saved key after %d ms", idx, (unsigned)saved,
                 r.ms);
        return BondEnc::Ok;
    }
    ESP_LOGW(kTag, "Bond: recognized (entry %d), encryption with the saved key failed: command 0x%02X, "
             "Encryption Change 0x%02X, %s after %d ms", idx, r.cmd, r.evt, r.dropped ? "dropped" : "waiting", r.ms);
    // We delete only a key that the earbuds explicitly do not have (Encryption Change 0x06 = PIN or Key Missing). A
    // missing response is sometimes transient: the left Buds3 Pro did not respond 2 min after pairing, and its key was
    // good (2026-11-01) - deleting it forced re-pairing.
    if (r.dropped || r.evt != 0x06) {
        return BondEnc::Failed;
    }
    if (!mayErase) {
        ESP_LOGW(kTag, "Bond: entry %d rejected (0x06) - kept, no background pairing", idx);
        return BondEnc::Rejected;
    }
    app::btbond::BondList all;
    {
        std::lock_guard<std::mutex> l(g_bt.stateLock);
        app::btbond::eraseBond(g_bt.bonds, idx);
        all = g_bt.bonds;
    }
    saveBondNvs(all);
    ESP_LOGW(kTag, "Bond: entry %d removed, %u left", idx, (unsigned)app::btbond::bondCount(all));
    wipeBonds(all);
    return BondEnc::Erased;  // re-pair on this link
}

bool performSmpScJustWorks(uint16_t connHandle, const uint8_t peerAddr[6], uint8_t peerAddrType, BtGattInfo_t& gatt)
{
    {
        std::lock_guard<std::mutex> l(g_bt.stateLock);
        g_bt.smpQHead = 0;
        g_bt.smpQTail = 0;
        g_bt.smpConn  = connHandle;
    }
    g_smpFailReason = 0;

    // Responder Key Distribution = IdKey: the earbuds hand over IRK and identity address, so we recognize them after
    // their private address changes (without it every reconnect pairs from scratch, which earbuds with a bond reject).
    // Pairing Request like Android (AOSP bta_dm_co.cc bte_appl_cfg, btm_sec_api_types.h): IO KeyboardDisplay, AuthReq
    // Bond|MITM|SC|CT2, keys Enc|Id|Sign|LinkKey both ways. In LE SC EncKey is ignored, LinkKey is only the derivation
    // of the BR/EDR key (no PDU), Id and Sign go as PDUs (Core Vol 3 Part H 3.6.1). Previously (Id only) the
    // Buds3 Pro earbuds lost the second one's key after pairing the first (docs/c6-link-layer.md).
    // Without our IRK (bt.ident 1, like Linux without privacy) the IdKey bit in Initiator Key Distribution is cleared.
    const uint8_t preq[7] = {0x01, 0x04, 0x00, 0x2D, 0x10, (uint8_t)(g_bt.ownIdKey ? 0x0F : 0x0D), 0x0F};
    if (!sendAclL2cap(connHandle, 0x0006, preq, sizeof(preq))) {
        return false;
    }

    uint8_t pres[72] = {};
    uint16_t presLen = 0;
    g_smpStage = "pairing_response";
    if (!waitSmpOpcode(0x02, pres, presLen) || presLen < 7) {
        return false;
    }
    gatt.smpAuthReq = pres[3];
    ESP_LOGI(kTag, "SMP: IO response 0x%02X, AuthReq 0x%02X, earbud keys 0x%02X / ours 0x%02X", pres[1], pres[3],
             pres[6], pres[5]);
    if ((pres[3] & 0x08) == 0) {
        return false;
    }
    // Method (Core Vol 3 Part H 2.3.5.1, table 2.8 for SC): with MITM and our KeyboardDisplay a DisplayOnly or
    // KeyboardOnly responder requires a passkey (Passkey Entry) - we do not do that. NoInputNoOutput = Just Works,
    // DisplayYesNo and KeyboardDisplay = Numeric Comparison (same computations as Just Works, the user confirms on
    // their side).
    if (((preq[3] | pres[3]) & 0x04) != 0 && (pres[1] == 0x00 || pres[1] == 0x02)) {
        const uint8_t failed[2] = {0x05, 0x01};  // Pairing Failed: Passkey Entry Failed
        sendAclL2cap(connHandle, 0x0006, failed, sizeof(failed));
        g_smpStage = "passkey_not_supported";
        return false;
    }

    uint8_t pubAx[32] = {}, pubAy[32] = {}, privA[32] = {};
    if (!smGenKeypairP256(pubAx, pubAy, privA)) {
        return false;
    }
    uint8_t pkPdu[65] = {0x0C};
    std::memcpy(&pkPdu[1], pubAx, 32);
    std::memcpy(&pkPdu[33], pubAy, 32);
    if (!sendAclL2cap(connHandle, 0x0006, pkPdu, sizeof(pkPdu))) {
        return false;
    }

    uint8_t peerPkPdu[72] = {};
    uint16_t peerPkLen    = 0;
    g_smpStage = "peer_public_key";
    if (!waitSmpOpcode(0x0C, peerPkPdu, peerPkLen) || peerPkLen < 65) {
        return false;
    }
    const uint8_t* pubBx = &peerPkPdu[1];
    const uint8_t* pubBy = &peerPkPdu[33];

    uint8_t cfmPdu[72] = {};
    uint16_t cfmLen    = 0;
    g_smpStage = "peer_confirm";
    if (!waitSmpOpcode(0x03, cfmPdu, cfmLen) || cfmLen < 17) {
        return false;
    }

    uint8_t randPdu[17] = {0x04};
    esp_fill_random(&randPdu[1], 16);
    const uint8_t* Na = &randPdu[1];
    if (!sendAclL2cap(connHandle, 0x0006, randPdu, sizeof(randPdu))) {
        return false;
    }

    uint8_t peerRandPdu[72] = {};
    uint16_t peerRandLen    = 0;
    g_smpStage = "peer_random";
    if (!waitSmpOpcode(0x04, peerRandPdu, peerRandLen) || peerRandLen < 17) {
        return false;
    }
    const uint8_t* Nb = &peerRandPdu[1];

    uint8_t dhKey[32]  = {};
    uint8_t macKey[16] = {};
    uint8_t ltk[16]    = {};
    uint8_t rZero[16]  = {};
    uint8_t dhChk[17]  = {0x0D};

    if (!smGenDhkeyP256(pubBx, pubBy, privA, dhKey)) {
        return false;
    }
    if (!smF5(dhKey, Na, Nb, g_bt.ownAddrType, g_bt.ownAddr, peerAddrType & 0x01, peerAddr, macKey, ltk)) {
        return false;
    }
    if (!smF6(macKey, Na, Nb, rZero, &preq[1], g_bt.ownAddrType, g_bt.ownAddr, peerAddrType & 0x01, peerAddr,
              &dhChk[1])) {
        return false;
    }
    if (!sendAclL2cap(connHandle, 0x0006, dhChk, sizeof(dhChk))) {
        return false;
    }

    uint8_t peerChkPdu[72] = {};
    uint16_t peerChkLen    = 0;
    g_smpStage = "peer_dhkey_check";
    if (!waitSmpOpcode(0x0D, peerChkPdu, peerChkLen) || peerChkLen < 17) {
        return false;
    }

    g_smpStage = "enable_encryption";
    if (!startLinkEncryption(connHandle, ltk, 2000).ok) {
        return false;
    }

    app::btbond::Bond bond;
    bond.valid = true;
    std::memcpy(bond.addr, peerAddr, 6);
    std::memcpy(bond.ltk, ltk, 16);
    // Key distribution after encryption is enabled (Core Vol 3 Part H 3.6.1): IdKey only if the earbuds accepted it
    // in Pairing Response. Without it the bond works as before - only under the address used for pairing.
    if ((pres[6] & 0x02) != 0) {
        uint8_t idInfo[72] = {};
        uint8_t idAddr[72] = {};
        uint16_t idInfoLen = 0, idAddrLen = 0;
        g_smpStage = "identity_information";
        if (waitSmpOpcode(0x08, idInfo, idInfoLen, 1500) && idInfoLen >= 17) {
            std::memcpy(bond.irk, &idInfo[1], 16);
            bond.hasIrk = true;
            g_smpStage = "identity_address";
            if (waitSmpOpcode(0x09, idAddr, idAddrLen, 1500) && idAddrLen >= 8) {
                std::memcpy(bond.addr, &idAddr[2], 6);
            }
        }
        std::memset(idInfo, 0, sizeof(idInfo));
        ESP_LOGI(kTag, "Bond: IRK %s, identity address %s", bond.hasIrk ? "present" : "absent",
                 idAddrLen >= 8 ? "present" : "as at pairing");
    }
    if ((pres[6] & 0x04) != 0) {
        // Signing Information (earbud CSRK) after IdKey - we receive it so the distribution ends like on Android; we do
        // not use signed writes, so we do not keep the key
        uint8_t sign[72] = {};
        uint16_t signLen = 0;
        g_smpStage = "signing_information";
        const bool got = waitSmpOpcode(0x0A, sign, signLen, 1500) && signLen >= 17;
        std::memset(sign, 0, sizeof(sign));
        ESP_LOGI(kTag, "Bond: earbud CSRK %s", got ? "received" : "did not arrive");
    }
    // Our keys after the earbuds' keys (Core Vol 3 Part H 3.6.1), only if they accepted them in Pairing Response
    if ((pres[5] & 0x02) != 0) {
        uint8_t info[17] = {0x08};
        uint8_t addr[8]  = {0x09, g_bt.ownAddrType};  // same address and type we connect from
        ownIrk(&info[1]);
        std::memcpy(&addr[2], g_bt.ownAddr, 6);
        g_smpStage    = "own_identity";
        const bool ok = sendAclL2cap(connHandle, 0x0006, info, sizeof(info)) &&
                        sendAclL2cap(connHandle, 0x0006, addr, sizeof(addr));
        std::memset(info, 0, sizeof(info));
        ESP_LOGI(kTag, "Bond: our identity (IRK + address) %s", ok ? "sent" : "NOT sent");
    } else if (g_bt.ownIdKey) {
        ESP_LOGW(kTag, "Bond: the earbuds did not accept our identity (Initiator Key Distribution 0x%02X)", pres[5]);
    }
    if ((pres[5] & 0x04) != 0) {
        // Our CSRK (Signing Information) - random per pairing: we sign nothing, but the distribution should be complete
        uint8_t sign[17] = {0x0A};
        esp_fill_random(&sign[1], 16);
        const bool ok = sendAclL2cap(connHandle, 0x0006, sign, sizeof(sign));
        std::memset(sign, 0, sizeof(sign));
        ESP_LOGI(kTag, "Bond: our CSRK %s", ok ? "sent" : "NOT sent");
    }

    app::btbond::BondList all;
    {
        std::lock_guard<std::mutex> l(g_bt.stateLock);
        app::btbond::putBond(g_bt.bonds, bond);
        all = g_bt.bonds;
    }
    saveBondNvs(all);
    ESP_LOGI(kTag, "Bond: saved, devices %u", (unsigned)app::btbond::bondCount(all));
    wipeBonds(all);
    std::memset(bond.ltk, 0, sizeof(bond.ltk));
    std::memset(bond.irk, 0, sizeof(bond.irk));
    gatt.paired = true;
    return true;
}

// CSIP (Coordinated Set Identification Profile): after pairing a set member the coordinator reads the SIRK, set size
// and rank from CSIS (Android always does). Tab5 did not, and pairing one Buds3 Pro earbud invalidated the other's key
// - the phone keeps both (docs/c6-link-layer.md). On an encrypted link; the SIRK does not go to the log. A type 0 SIRK
// is decrypted with this link's key (device bond, CSIS 4.6 sdf) and for the main connection the set is remembered
// (g_bt.set) - joinSetMember uses it to find the second earbud. Result for a connection other than the main one: 1 =
// same set (SIRK like the main one's), 0 = other set, -1 = unknown (no SIRK, not decrypted, or main without a set). For
// the main one -1.
int readCsis(uint16_t connHandle, const uint8_t peerAddr[6], uint8_t peerAddrType, bool freshPair)
{
    int sameSet = -1;
    uint8_t sirk[24] = {}, size[4] = {}, rank[4] = {};
    uint16_t sirkLen = 0, sizeLen = 0, rankLen = 0;
    bool authErr  = false;
    bool haveSirk = false;
    // Several attempts: a single SIRK read may fail (Tab5 2026-11-01: "SIRK missing" with size and rank read right
    // after), and without the SIRK the second earbud does not join. A security refusal will not change - no retries.
    for (int attempt = 0; attempt < 3 && !haveSirk && !authErr; attempt++) {
        if (attempt > 0) {
            vTaskDelay(pdMS_TO_TICKS(300));
        }
        haveSirk = attReadByType(connHandle, 0x0001, 0xFFFF, 0x2B84, sirk, sirkLen, nullptr, &authErr) && sirkLen >= 17;
    }
    attReadByType(connHandle, 0x0001, 0xFFFF, 0x2B85, size, sizeLen);
    attReadByType(connHandle, 0x0001, 0xFFFF, 0x2B87, rank, rankLen);
    uint8_t ltk[16] = {}, plain[16] = {};
    bool decoded    = false;
    if (haveSirk) {
        bool haveLtk = false;
        {
            std::lock_guard<std::mutex> l(g_bt.stateLock);
            const int idx = app::btbond::findBond(g_bt.bonds, peerAddrType, peerAddr, aes128Be);
            if (idx >= 0) {
                std::memcpy(ltk, g_bt.bonds.items[idx].ltk, 16);
                haveLtk = true;
            }
        }
        decoded = app::csip::sirkFromValue(sirk, sirkLen, haveLtk ? ltk : nullptr, plain, aes128Be);
    }
    if (decoded) {
        std::lock_guard<std::mutex> l(g_bt.stateLock);
        if (g_bt.set.valid && connHandle != g_bt.status.connHandle) {
            sameSet = std::memcmp(plain, g_bt.set.sirk, 16) == 0 ? 1 : 0;
            ESP_LOGI(kTag, "CSIS: SIRK vs main connection: %s", sameSet == 1 ? "same" : "DIFFERENT");
        }
    }
    if (decoded && sizeLen >= 1) {
        std::lock_guard<std::mutex> l(g_bt.stateLock);
        if (connHandle == g_bt.status.connHandle) {
            g_bt.set.valid = true;
            std::memcpy(g_bt.set.sirk, plain, 16);
            g_bt.set.size = size[0];
            g_bt.set.rank = rankLen >= 1 ? rank[0] : 0;
            g_bt.set.conn = connHandle;
            g_bt.set.fresh = freshPair;
        }
    }
    ESP_LOGI(kTag, "CSIS: SIRK %s (type %u%s), set size %d, rank %d",
             haveSirk ? "read" : (authErr ? "refused (security)" : "missing"), haveSirk ? sirk[0] : 0u,
             haveSirk ? (decoded ? ", decrypted" : ", NOT decrypted") : "", sizeLen >= 1 ? size[0] : -1,
             rankLen >= 1 ? rank[0] : -1);
    std::memset(sirk, 0, sizeof(sirk));
    std::memset(ltk, 0, sizeof(ltk));
    std::memset(plain, 0, sizeof(plain));
    return sameSet;
}

void readProtectedGattValues(uint16_t connHandle, uint16_t batStart, uint16_t batEnd, uint16_t vcsStart,
                             uint16_t vcsEnd, uint16_t pacsStart, uint16_t pacsEnd, BtGattInfo_t& gatt)
{
    if (batStart != 0) {
        uint8_t val[16] = {};
        uint16_t vlen   = 0;
        if (attReadByType(connHandle, batStart, batEnd, 0x2A19, val, vlen, nullptr, &gatt.authRequired) && vlen >= 1) {
            gatt.batteryPct = std::clamp<int>(val[0], 0, 100);
        }
    }
    if (vcsStart != 0) {
        uint8_t val[16]  = {};
        uint16_t vlen    = 0;
        uint16_t vcsValH = 0;
        if (attReadByType(connHandle, vcsStart, vcsEnd, 0x2B7D, val, vlen, &vcsValH, &gatt.authRequired) && vlen >= 1) {
            gatt.volume = val[0];
            std::lock_guard<std::mutex> l(g_bt.stateLock);
            if (vcsValH != 0) {
                g_bt.vcsStateHandle = vcsValH;
            }
            if (vlen >= 2) {
                g_bt.muted = (val[1] != 0);
            }
            if (vlen >= 3) {
                g_bt.vcsChangeCounter = val[2];
            }
        }
    }
    if (pacsStart != 0) {
        uint8_t val[248] = {};
        uint16_t vlen    = 0;
        if (attReadByType(connHandle, pacsStart, pacsEnd, 0x2BC9, val, vlen, nullptr, &gatt.authRequired) && vlen > 0) {
            gatt.sinkPacHex     = hexBytes(val, vlen, 32);
            gatt.sinkPacSummary = decodePacRecords(val, vlen);
        }
        if (attReadByType(connHandle, pacsStart, pacsEnd, 0x2BCA, val, vlen, nullptr, &gatt.authRequired) &&
            vlen >= 4) {
            gatt.sinkLocations = (uint32_t)val[0] | ((uint32_t)val[1] << 8) | ((uint32_t)val[2] << 16) |
                                 ((uint32_t)val[3] << 24);
        }
        if (attReadByType(connHandle, pacsStart, pacsEnd, 0x2BCB, val, vlen, nullptr, &gatt.authRequired) && vlen > 0) {
            gatt.srcPacHex     = hexBytes(val, vlen, 32);
            gatt.srcPacSummary = decodePacRecords(val, vlen);
        }
        if (attReadByType(connHandle, pacsStart, pacsEnd, 0x2BCC, val, vlen, nullptr, &gatt.authRequired) &&
            vlen >= 4) {
            gatt.srcLocations = (uint32_t)val[0] | ((uint32_t)val[1] << 8) | ((uint32_t)val[2] << 16) |
                                ((uint32_t)val[3] << 24);
        }
    }
}

void subscribeEarbudNotifications(uint16_t connHandle, uint16_t vcsStart, uint16_t vcsEnd, BtGattInfo_t& gatt)
{
    // 1. Discovery of the VCS Volume State (0x2B7D) and Volume Control Point (0x2B7E) handles
    if (vcsStart != 0) {
        uint16_t chStart = vcsStart;
        for (int iter = 0; iter < 6 && chStart <= vcsEnd; iter++) {
            const uint8_t req[7] = {
                0x08,
                (uint8_t)(chStart & 0xFF),
                (uint8_t)(chStart >> 8),
                (uint8_t)(vcsEnd & 0xFF),
                (uint8_t)(vcsEnd >> 8),
                0x03,
                0x28,  // 0x2803 Characteristic Declaration
            };
            uint8_t rsp[128] = {};
            uint16_t rspLen  = 0;
            if (!attRequest(connHandle, req, sizeof(req), rsp, rspLen, 450) || rspLen < 9 || rsp[0] != 0x09 ||
                rsp[1] != 7) {
                break;
            }
            uint16_t lastH = 0;
            for (uint16_t off = 2; off + 7 <= rspLen; off = (uint16_t)(off + 7)) {
                lastH                = (uint16_t)rsp[off] | ((uint16_t)rsp[off + 1] << 8);
                const uint16_t valH  = (uint16_t)rsp[off + 3] | ((uint16_t)rsp[off + 4] << 8);
                const uint16_t cuuid = (uint16_t)rsp[off + 5] | ((uint16_t)rsp[off + 6] << 8);
                std::lock_guard<std::mutex> l(g_bt.stateLock);
                if (cuuid == 0x2B7D) {
                    g_bt.vcsStateHandle = valH;
                } else if (cuuid == 0x2B7E) {
                    g_bt.vcsCpHandle = valH;
                }
            }
            if (lastH == 0 || lastH >= vcsEnd) {
                break;
            }
            chStart = (uint16_t)(lastH + 1);
        }
    }

    // 2. Discovery and enabling of all CCCD descriptors (0x2902) in the earbuds (handles 0x0010..0x00A0)
    std::vector<uint16_t> cccds;
    uint16_t fStart = 0x0010;
    const uint16_t fEnd = 0x00A0;
    for (int iter = 0; iter < 24 && fStart <= fEnd; iter++) {
        const uint8_t req[5] = {
            0x04,  // ATT_FIND_INFORMATION_REQ
            (uint8_t)(fStart & 0xFF),
            (uint8_t)(fStart >> 8),
            (uint8_t)(fEnd & 0xFF),
            (uint8_t)(fEnd >> 8),
        };
        uint8_t rsp[256] = {};
        uint16_t rspLen  = 0;
        if (!attRequest(connHandle, req, sizeof(req), rsp, rspLen, 450) || rspLen < 4 || rsp[0] != 0x05) {
            break;
        }
        const uint8_t fmt    = rsp[1];
        const uint16_t step  = (fmt == 0x01) ? 4 : 18;
        uint16_t lastH       = 0;
        for (uint16_t off = 2; off + step <= rspLen; off = (uint16_t)(off + step)) {
            const uint16_t h = (uint16_t)rsp[off] | ((uint16_t)rsp[off + 1] << 8);
            lastH            = h;
            if (fmt == 0x01) {
                const uint16_t u16 = (uint16_t)rsp[off + 2] | ((uint16_t)rsp[off + 3] << 8);
                if (u16 == 0x2902) {
                    if (std::find(cccds.begin(), cccds.end(), h) == cccds.end()) {
                        cccds.push_back(h);
                    }
                } else if (u16 == 0x2BC3) {
                    std::lock_guard<std::mutex> l(g_bt.stateLock);
                    g_bt.micsMuteHandle = h;
                }
            }
        }
        if (lastH == 0 || lastH >= fEnd) {
            break;
        }
        fStart = (uint16_t)(lastH + 1);
    }

    // Directly make sure that the CCCD for VCS Volume State (vcsStateHandle + 1) is on the list
    if (g_bt.vcsStateHandle != 0) {
        const uint16_t vcsCccd = (uint16_t)(g_bt.vcsStateHandle + 1);
        if (std::find(cccds.begin(), cccds.end(), vcsCccd) == cccds.end()) {
            cccds.push_back(vcsCccd);
        }
    }

    uint32_t subOk = 0;
    for (uint16_t ch : cccds) {
        const uint8_t wrReq[5] = {0x12, (uint8_t)(ch & 0xFF), (uint8_t)(ch >> 8), 0x01, 0x00};
        uint8_t rsp[16]        = {};
        uint16_t rspLen        = 0;
        const bool ok = (attRequest(connHandle, wrReq, sizeof(wrReq), rsp, rspLen, 350) && rspLen >= 1 && rsp[0] == 0x13);
        if (ok) {
            subOk++;
        }
        ESP_LOGD(kTag, "CCCD 0x%04X sub: %s", ch, ok ? "OK" : "FAIL");
    }

    // 3. If the VCS volume in the earbuds is at the very ceiling (255) or floor (0), set the middle (140 = 55%),
    //    so that swiping up (Louder) and down (Quieter) immediately sends deltas!
    uint16_t vcsCp = 0;
    uint8_t chgCnt = 0;
    {
        std::lock_guard<std::mutex> l(g_bt.stateLock);
        g_bt.cccdSubscribed = subOk;
        vcsCp               = g_bt.vcsCpHandle;
        chgCnt              = g_bt.vcsChangeCounter;
    }
    if (vcsCp != 0 && (gatt.volume >= 245 || gatt.volume == 0)) {
        const uint8_t setVol[6] = {
            0x12,
            (uint8_t)(vcsCp & 0xFF),
            (uint8_t)(vcsCp >> 8),
            0x04,    // Opcode: Set Absolute Volume
            chgCnt,  // Change Counter
            140,     // 55% (headroom up and down)
        };
        uint8_t rsp[16] = {};
        uint16_t rspLen = 0;
        if (attRequest(connHandle, setVol, sizeof(setVol), rsp, rspLen, 400) && rspLen >= 1 && rsp[0] == 0x13) {
            gatt.volume = 140;
            std::lock_guard<std::mutex> l(g_bt.stateLock);
            g_bt.status.gatt.volume = 140;
        }
    }

    if (subOk > 0) {
        std::lock_guard<std::mutex> l(g_bt.stateLock);
        char buf[96];
        std::snprintf(buf, sizeof(buf), "Listening active (%lu CCCD) — click / swipe on the earbuds",
                      (unsigned long)subOk);
        recordBtEventLocked(buf);
    }
}

void inspectGattAndLeAudio(uint16_t connHandle, const uint8_t peerAddr[6], uint8_t peerAddrType, BtGattInfo_t& gatt)
{
    gatt = BtGattInfo_t{};
    {
        std::lock_guard<std::mutex> l(g_bt.stateLock);
        g_bt.lastPlacementL = 0;
        g_bt.lastPlacementR = 0;
        g_bt.lastRawSamsungLen = 0;
        // Handles are per device: without zeroing, a new connection inherited the previous one's handles
        // (ascsSinkCount grew to 2 and no longer accepted new ASEs, notifications went to the old handles)
        g_bt.vcsStateHandle       = 0;
        g_bt.vcsCpHandle          = 0;
        g_bt.micsMuteHandle       = 0;
        g_bt.cccdSubscribed       = 0;
        g_bt.ascsCpHandle         = 0;
        g_bt.ascsSinkAseHandle[0] = 0;
        g_bt.ascsSinkAseHandle[1] = 0;
        g_bt.ascsSinkCount        = 0;
        g_bt.ascsState            = 0;
        g_bt.ascsSrcAseHandle[0]  = 0;
        g_bt.ascsSrcAseHandle[1]  = 0;
        g_bt.ascsSrcCount         = 0;
        g_bt.pacsStart            = 0;
        g_bt.pacsEnd              = 0;
    }

    // 0. Known device: encryption with the saved key before anything else (encryptWithBond). In the background
    // (btReconnect) without erasing the key and without pairing from scratch.
    bool bondOnly = false;
    {
        std::lock_guard<std::mutex> l(g_bt.stateLock);
        // Without allow_pairing (le_audio_config_t) only saved keys are used: no pairing, no erased key
        bondOnly = g_bt.bondOnly || !leport::config().allowPairing;
    }
    const BondEnc bondEnc = encryptWithBond(connHandle, peerAddr, peerAddrType, !bondOnly);

    // 1. MTU negotiation = 247
    {
        const uint8_t mtuReq[3] = {0x02, 0xF7, 0x00};
        uint8_t rsp[32]         = {};
        uint16_t rspLen         = 0;
        if (attRequest(connHandle, mtuReq, sizeof(mtuReq), rsp, rspLen, 600) && rspLen >= 3 && rsp[0] == 0x03) {
            gatt.mtu = (uint16_t)rsp[1] | ((uint16_t)rsp[2] << 8);
        }
    }

    // 2. Discovery of all primary services (Primary Services, UUID 0x2800)
    uint16_t curStart  = 0x0001;
    uint16_t gapStart  = 0x0001, gapEnd = 0x000F;
    uint16_t batStart  = 0, batEnd = 0;
    uint16_t vcsStart  = 0, vcsEnd = 0;
    uint16_t pacsStart = 0, pacsEnd = 0;
    uint16_t ascsStart = 0, ascsEnd = 0;

    for (int iter = 0; iter < 24; iter++) {
        const uint8_t req[7] = {
            0x10,  // ATT_READ_BY_GROUP_TYPE_REQ
            (uint8_t)(curStart & 0xFF),
            (uint8_t)(curStart >> 8),
            0xFF,
            0xFF,
            0x00,
            0x28,  // 0x2800 Primary Service
        };
        uint8_t rsp[256] = {};
        uint16_t rspLen  = 0;
        if (!attRequest(connHandle, req, sizeof(req), rsp, rspLen, 750)) {
            break;
        }
        if (rspLen < 4 || rsp[0] != 0x11) {
            break;
        }
        const uint8_t itemLen = rsp[1];
        if (itemLen != 6 && itemLen != 20) {
            break;
        }
        uint16_t lastEnd = 0;
        for (uint16_t off = 2; off + itemLen <= rspLen; off = (uint16_t)(off + itemLen)) {
            const uint16_t sH = (uint16_t)rsp[off] | ((uint16_t)rsp[off + 1] << 8);
            const uint16_t eH = (uint16_t)rsp[off + 2] | ((uint16_t)rsp[off + 3] << 8);
            lastEnd           = eH;

            BtGattService svc;
            svc.startHandle = sH;
            svc.endHandle   = eH;
            if (itemLen == 6) {
                svc.uuid16 = (uint16_t)rsp[off + 4] | ((uint16_t)rsp[off + 5] << 8);
                char uhex[12];
                std::snprintf(uhex, sizeof(uhex), "0x%04X", svc.uuid16);
                svc.uuidHex = uhex;
                svc.name    = gattServiceName(svc.uuid16);
            } else {
                const uint8_t* u = &rsp[off + 4];
                char uhex[40];
                std::snprintf(uhex, sizeof(uhex),
                              "%02X%02X%02X%02X-%02X%02X-%02X%02X-%02X%02X-%02X%02X%02X%02X%02X%02X", u[15], u[14],
                              u[13], u[12], u[11], u[10], u[9], u[8], u[7], u[6], u[5], u[4], u[3], u[2], u[1], u[0]);
                svc.uuidHex = uhex;
                svc.name    = "128-bit Vendor service";
            }
            gatt.services.push_back(svc);

            switch (svc.uuid16) {
                case 0x1800:
                    gapStart = sH;
                    gapEnd   = eH;
                    break;
                case 0x180F:
                    if (batStart == 0) {
                        batStart = sH;
                        batEnd   = eH;
                    }
                    break;
                case 0x1844:
                    gatt.hasVcs = true;
                    vcsStart    = sH;
                    vcsEnd      = eH;
                    break;
                case 0x1846: gatt.hasCsis = true; break;
                case 0x184E:
                    gatt.hasAscs = true;
                    ascsStart    = sH;
                    ascsEnd      = eH;
                    break;
                case 0x1850:
                    gatt.hasPacs = true;
                    pacsStart    = sH;
                    pacsEnd      = eH;
                    break;
                case 0x1853: gatt.hasCas = true; break;
                case 0x1854:
                case 0x1855: gatt.hasTmAp = true; break;
                default: break;
            }
        }
        if (lastEnd == 0 || lastEnd == 0xFFFF) {
            break;
        }
        curStart = (uint16_t)(lastEnd + 1);
    }

    // 3. Read GAP Device Name (0x2A00)
    {
        uint8_t val[248] = {};
        uint16_t vlen    = 0;
        if (attReadByType(connHandle, gapStart, gapEnd, 0x2A00, val, vlen) && vlen > 0) {
            gatt.deviceName.assign(reinterpret_cast<const char*>(val), vlen);
            // The name from the air goes to the UI and JSON: without control characters and NUL
            for (char& c : gatt.deviceName) {
                if ((uint8_t)c < 0x20 || (uint8_t)c == 0x7F) {
                    c = '?';
                }
            }
        }
    }

    {
        std::lock_guard<std::mutex> l(g_bt.stateLock);
        g_bt.pacsStart = pacsStart;
        g_bt.pacsEnd   = pacsEnd;
    }
    // 4-6. Read Battery (0x2A19), VCS Volume (0x2B7D) and PACS (0x2BC9..0x2BCC)
    readProtectedGattValues(connHandle, batStart, batEnd, vcsStart, vcsEnd, pacsStart, pacsEnd, gatt);

    // 7. Read characteristic declarations (0x2803) in ASCS (0x184E) — count Sink ASE (0x2BC4) and Source ASE (0x2BC5)
    if (ascsStart != 0) {
        uint16_t chStart = ascsStart;
        for (int iter = 0; iter < 12 && chStart <= ascsEnd; iter++) {
            const uint8_t req[7] = {
                0x08,  // ATT_READ_BY_TYPE_REQ
                (uint8_t)(chStart & 0xFF),
                (uint8_t)(chStart >> 8),
                (uint8_t)(ascsEnd & 0xFF),
                (uint8_t)(ascsEnd >> 8),
                0x03,
                0x28,  // 0x2803 Characteristic Declaration
            };
            uint8_t rsp[256] = {};
            uint16_t rspLen  = 0;
            if (!attRequest(connHandle, req, sizeof(req), rsp, rspLen, 600) || rspLen < 4 || rsp[0] != 0x09) {
                break;
            }
            const uint8_t itemLen = rsp[1];
            if (itemLen != 7) {
                break;
            }
            uint16_t lastH = 0;
            for (uint16_t off = 2; off + itemLen <= rspLen; off = (uint16_t)(off + itemLen)) {
                lastH                = (uint16_t)rsp[off] | ((uint16_t)rsp[off + 1] << 8);
                const uint16_t valH  = (uint16_t)rsp[off + 3] | ((uint16_t)rsp[off + 4] << 8);
                const uint16_t cuuid = (uint16_t)rsp[off + 5] | ((uint16_t)rsp[off + 6] << 8);
                if (cuuid == 0x2BC4) {
                    gatt.sinkAseCount++;
                    std::lock_guard<std::mutex> l(g_bt.stateLock);
                    if (g_bt.ascsSinkCount < 2) {
                        g_bt.ascsSinkAseHandle[g_bt.ascsSinkCount++] = valH;
                    }
                } else if (cuuid == 0x2BC5) {
                    gatt.srcAseCount++;
                    std::lock_guard<std::mutex> l(g_bt.stateLock);
                    if (g_bt.ascsSrcCount < 2) {
                        g_bt.ascsSrcAseHandle[g_bt.ascsSrcCount++] = valH;
                    }
                } else if (cuuid == 0x2BC6) {
                    std::lock_guard<std::mutex> l(g_bt.stateLock);
                    g_bt.ascsCpHandle = valH;
                }
            }
            if (lastH == 0 || lastH >= ascsEnd) {
                break;
            }
            chStart = (uint16_t)(lastH + 1);
        }
    }

    // 8. We perform SMP LE Secure Connections pairing (Just Works P-256 ECDH + AES-CMAC f4/f5/f6)
    //    and enable HCI_LE_Enable_Encryption (0x2019) for LE Audio earbuds,
    //    then read PACS / VCS / Battery over the encrypted link!
    if (bondEnc == BondEnc::Ok) {
        gatt.paired = true;
        readCsis(connHandle, peerAddr, peerAddrType, false);
        readProtectedGattValues(connHandle, batStart, batEnd, vcsStart, vcsEnd, pacsStart, pacsEnd, gatt);
    } else if (bondEnc == BondEnc::Failed) {
        // After a failed encryption start the earbuds do not pair on this link - pairing happens on the next connection
        ESP_LOGW(kTag, "No encryption: the saved key did not work on this connection");
    } else if (bondEnc == BondEnc::Rejected) {
        std::lock_guard<std::mutex> l(g_bt.stateLock);
        g_bt.bondRejected = true;
    } else if (!bondOnly && (gatt.authRequired || gatt.hasVcs || gatt.hasPacs || gatt.hasAscs)) {
        if (performSmpScJustWorks(connHandle, peerAddr, peerAddrType, gatt)) {
            readCsis(connHandle, peerAddr, peerAddrType, true);
            readProtectedGattValues(connHandle, batStart, batEnd, vcsStart, vcsEnd, pacsStart, pacsEnd, gatt);
        } else {
            ESP_LOGW(kTag, "SMP pairing failed at stage: %s (smp_auth_req=0x%02X, Pairing Failed 0x%02X)",
                     g_smpStage, gatt.smpAuthReq, g_smpFailReason);
        }
    }

    // 9. Subscribing to CCCD notifications (0x2902) from the earbuds (VCS volume, MICS microphone, battery, Samsung
    // Vendor)
    subscribeEarbudNotifications(connHandle, vcsStart, vcsEnd, gatt);
}

}  // namespace

/* --------------------- interface for hal_le_audio.cpp --------------------- */
// Commands and ATT requests under opLock: they share one global wait state (waitOpcode, attWait), so two threads at
// once (the LE Audio procedure and e.g. scanning) would attribute each other's responses to themselves.

bool halBtHciCmd(uint16_t opcode, const uint8_t* params, uint8_t plen, int timeoutMs, uint8_t* status, uint8_t* ret,
                 uint8_t retCap, uint8_t* retLen)
{
    std::lock_guard<std::recursive_mutex> op(g_bt.opLock);
    const bool ok = sendHciCmd(opcode, params, plen, timeoutMs);
    std::lock_guard<std::mutex> l(g_bt.stateLock);
    if (status != nullptr) {
        *status = ok ? g_bt.cmdStatus : 0xFF;
    }
    const uint8_t n = ok ? std::min<uint8_t>(g_bt.cmdRetLen, retCap) : 0;
    if (ret != nullptr && n > 0) {
        std::memcpy(ret, g_bt.cmdRet, n);
    }
    if (retLen != nullptr) {
        *retLen = n;
    }
    return ok;
}

bool halBtAtt(uint16_t connHandle, const uint8_t* req, uint16_t reqLen, uint8_t* rsp, uint16_t rspCap,
              uint16_t* rspLen, int timeoutMs)
{
    std::lock_guard<std::recursive_mutex> op(g_bt.opLock);
    uint16_t n    = 0;
    const bool ok = attRequestBuf(connHandle, req, reqLen, rsp, rspCap, n, timeoutMs);
    if (rspLen != nullptr) {
        *rspLen = n;
    }
    return ok;
}

bool halBtSendRaw(const uint8_t* pkt, uint16_t len)
{
    std::lock_guard<std::mutex> tl(g_bt.txLock);
    return esp_hosted_cyberdeck_hci_tx(pkt, len) == 0;
}

HalBtLeAudioLink halBtLeAudioLink()
{
    const bool leOn = leAudioEnabled();  // before stateLock: settings have their own lock
    std::lock_guard<std::mutex> l(g_bt.stateLock);
    HalBtLeAudioLink k;
    k.acl         = g_bt.status.state == BtState::Connected ? g_bt.status.connHandle : 0xFFFF;
    const BtEngine::LinkMeta* m = k.acl != 0xFFFF ? linkMetaLocked(k.acl, false) : nullptr;
    k.encrypted   = m != nullptr && m->enc;
    k.leAudioOn   = leOn;
    k.cisCentral  = g_bt.cisCentral;
    k.dleTxOctets = m != nullptr ? m->dleTx : 0;
    k.connItvl    = m != nullptr ? m->itvl : 0;
    k.acl2        = k.acl != 0xFFFF && aclUpLocked(g_bt.secondHandle) ? g_bt.secondHandle : 0xFFFF;
    k.setJoining  = g_bt.setJoining;
    k.pacsStart   = g_bt.pacsStart;
    k.pacsEnd     = g_bt.pacsEnd;
    k.ascsCp      = g_bt.ascsCpHandle;
    for (int i = 0; i < 2; i++) {
        k.sinkAse[i] = g_bt.ascsSinkAseHandle[i];
        k.srcAse[i]  = g_bt.ascsSrcAseHandle[i];
    }
    return k;
}

HalBtAcl halBtAcl(uint16_t acl)
{
    std::lock_guard<std::mutex> l(g_bt.stateLock);
    HalBtAcl a;
    const BtEngine::LinkMeta* m = acl != 0xFFFF ? linkMetaLocked(acl, false) : nullptr;
    a.up          = aclUpLocked(acl);
    a.encrypted   = m != nullptr && m->enc;
    a.dleTxOctets = m != nullptr ? m->dleTx : 0;
    a.connItvl    = m != nullptr ? m->itvl : 0;
    return a;
}

void halBtRequestDataLength(uint16_t acl)
{
    const uint8_t p[6] = {(uint8_t)(acl & 0xFF), (uint8_t)((acl >> 8) & 0x0F), 251, 0, 0x48, 0x08};
    halBtHciCmd(0x2022, p, sizeof(p), 250, nullptr, nullptr, 0, nullptr);
}

/* =============================== host API (le_bt_types.h) =============================== */

void lehost::btSetRadio(bool on)
{
    std::lock_guard<std::recursive_mutex> op(g_bt.opLock);
    {
        std::lock_guard<std::mutex> l(g_bt.stateLock);
        g_bt.radioOn = on;
    }
    if (!on) {
        uint16_t hs[2] = {0xFFFF, 0xFFFF};
        {
            std::lock_guard<std::mutex> l(g_bt.stateLock);
            hs[0] = g_bt.status.connHandle;
            hs[1] = g_bt.secondHandle;
        }
        for (const uint16_t h : hs) {
            if (h != 0xFFFF) {
                const uint8_t discParams[3] = {(uint8_t)(h & 0xFF), (uint8_t)((h >> 8) & 0x0F), 0x13};
                sendHciCmd(0x0406, discParams, sizeof(discParams), 400);
            }
        }
        std::lock_guard<std::mutex> l(g_bt.stateLock);
        for (auto& m : g_bt.linkMeta) {
            m = BtEngine::LinkMeta{};
        }
        g_bt.secondHandle      = 0xFFFF;
        g_bt.status.state      = BtState::Off;
        g_bt.status.connHandle = 0xFFFF;
        g_bt.status.addr.clear();
        g_bt.status.name.clear();
        g_bt.status.rssi = 0;
        g_bt.status.gatt = {};
        return;
    }
    ensureRadioReady();
    std::lock_guard<std::mutex> l(g_bt.stateLock);
    if (g_bt.status.state == BtState::Off) {
        g_bt.status.state = BtState::Idle;
    }
}

lehost::BtStatus_t lehost::btStatus()
{
    std::lock_guard<std::mutex> l(g_bt.stateLock);
    return g_bt.status;
}

lehost::BtLink_t lehost::btLink()
{
    BtLink_t k;
    {
        std::lock_guard<std::mutex> l(g_bt.stateLock);
        k.state      = g_bt.status.state;
        k.connHandle = g_bt.status.connHandle;
        k.paired     = g_bt.status.gatt.paired;
        k.leAudio    = g_bt.status.gatt.hasAscs;
        k.setMissing = k.state == BtState::Connected && g_bt.set.valid && g_bt.set.size >= 2 &&
                       g_bt.set.conn == g_bt.status.connHandle && !aclUpLocked(g_bt.secondHandle) && !g_bt.setJoining;
        k.batteryPct = g_bt.status.gatt.batteryPct;
    }
    k.streamActive = halLeAudioActive();  // separate hal_le_audio lock - outside the BT state lock
    return k;
}

std::vector<lehost::BtDevice_t> lehost::btScan(int durationS)
{
    std::lock_guard<std::recursive_mutex> op(g_bt.opLock);
    durationS = std::clamp(durationS, 1, 15);

    {
        std::lock_guard<std::mutex> l(g_bt.stateLock);
        if (!g_bt.radioOn) {
            return {};
        }
    }
    if (!ensureRadioReady()) {
        return {};
    }

    BtState prevState = BtState::Idle;
    {
        std::lock_guard<std::mutex> l(g_bt.stateLock);
        prevState         = g_bt.status.state;
        g_bt.status.state = BtState::Scanning;
        g_bt.nScanDevs    = 0;
        g_bt.advReports   = 0;
    }

    const uint8_t extScanParam[8] = {0x01, 0x00, 0x01, 0x01, 0x60, 0x00, 0x30, 0x00};
    const uint8_t extScanEn[6]    = {0x01, 0x00, 0x00, 0x00, 0x00, 0x00};
    const uint8_t extScanDis[6]   = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

    const bool extOk = sendHciCmd(0x2041, extScanParam, sizeof(extScanParam)) && g_bt.cmdStatus == 0 &&
                       sendHciCmd(0x2042, extScanEn, sizeof(extScanEn)) && g_bt.cmdStatus == 0;
    if (extOk) {
        vTaskDelay(pdMS_TO_TICKS(durationS * 1000));
        sendHciCmd(0x2042, extScanDis, sizeof(extScanDis));
    } else {
        const uint8_t legScanParam[7] = {0x01, 0x60, 0x00, 0x30, 0x00, 0x01, 0x00};
        const uint8_t legScanEn[2]    = {0x01, 0x00};
        const uint8_t legScanDis[2]   = {0x00, 0x00};
        sendHciCmd(0x200B, legScanParam, sizeof(legScanParam));
        sendHciCmd(0x200C, legScanEn, sizeof(legScanEn));
        vTaskDelay(pdMS_TO_TICKS(durationS * 1000));
        sendHciCmd(0x200C, legScanDis, sizeof(legScanDis));
    }

    std::vector<BtDevice_t> out;
    {
        std::lock_guard<std::mutex> l(g_bt.stateLock);
        if (g_bt.status.state == BtState::Scanning) {
            g_bt.status.state = (g_bt.status.connHandle != 0xFFFF) ? BtState::Connected
                                : (prevState == BtState::Connected ? BtState::Idle : prevState);
            if (g_bt.status.state == BtState::Scanning) {
                g_bt.status.state = BtState::Idle;
            }
        }
        out.reserve(g_bt.nScanDevs);
        // Bonds of connected earbuds - a row with a different private address of the same set is the same earbuds
        uint8_t connAddr[6] = {};
        int mainBond = -1, secondBond = -1;
        if (g_bt.status.connHandle != 0xFFFF && parseMac(g_bt.status.addr, connAddr)) {
            mainBond = app::btbond::findBond(g_bt.bonds, g_bt.status.addrType, connAddr, aes128Be);
        }
        if (aclUpLocked(g_bt.secondHandle)) {
            secondBond = app::btbond::findBond(g_bt.bonds, 1, g_bt.secondPeer, aes128Be);
        }
        for (int i = 0; i < g_bt.nScanDevs; i++) {
            const AdvRecord& r = g_bt.scanDevs[i];
            BtDevice_t d;
            char mac[20];
            formatMac(r.addr, mac);
            d.addr           = mac;
            d.addrType       = r.addrType;
            d.name           = r.name;
            d.mfgId          = r.mfgId;
            d.mfgName        = mfgNameFromId(r.mfgId);
            d.mfgHex         = hexBytes(r.mfgData, r.mfgLen, 12);
            d.rssi           = r.rssi;
            d.extAdv         = r.extAdv;
            d.leAudio        = r.leAudio;
            d.audioCandidate = r.audioCandidate;
            d.appearance     = r.appearance;
            for (uint8_t k = 0; k < r.nUuids16; k++) {
                d.uuids16.push_back(r.uuids16[k]);
            }
            // Earbuds out of the case advertise from a private address - their IRK from the bond resolves it
            // (automatic connection)
            const int bond   = app::btbond::findBond(g_bt.bonds, r.addrType, r.addr, aes128Be);
            d.bonded         = bond >= 0;
            d.inConnectedSet = bond >= 0 && (bond == mainBond || bond == secondBond);
            out.push_back(std::move(d));
        }
    }

    // One line per scan for bonded-earbud discovery: what the radio heard, how many private (resolvable) addresses,
    // and which of them resolved to a saved bond, with the flags the connect filter uses (leAudio / audioCandidate).
    {
        int rpa = 0, bonded = 0, leAudio = 0;
        for (const auto& d : out) {
            uint8_t a[6] = {};
            if (parseMac(d.addr, a) && app::btbond::isResolvablePrivate(d.addrType, a)) {
                rpa++;
            }
            bonded += d.bonded ? 1 : 0;
            leAudio += d.leAudio ? 1 : 0;
        }
        uint32_t reports = 0;
        size_t bondsN    = 0;
        {
            std::lock_guard<std::mutex> l(g_bt.stateLock);
            reports = g_bt.advReports;
            bondsN  = app::btbond::bondCount(g_bt.bonds);
        }
        ESP_LOGI(kTag, "scan %d s (%s): %lu advertising reports, %u devices, %d resolvable private, %d LE Audio, "
                 "%d bonded (saved bonds %u)", durationS, extOk ? "extended" : "legacy", (unsigned long)reports,
                 (unsigned)out.size(), rpa, leAudio, bonded, (unsigned)bondsN);
        for (const auto& d : out) {
            if (!d.bonded) {
                continue;
            }
            uint8_t a[6] = {};
            parseMac(d.addr, a);
            int idx = -1;
            {
                std::lock_guard<std::mutex> l(g_bt.stateLock);
                idx = app::btbond::findBond(g_bt.bonds, d.addrType, a, aes128Be);
            }
            ESP_LOGI(kTag, "  bonded: ..:%02X:%02X type %u -> bond %d, %d dBm, leAudio %d, audioCandidate %d, name '%s'%s",
                     a[1], a[0], d.addrType, idx, d.rssi, d.leAudio, d.audioCandidate, d.name.c_str(),
                     d.leAudio || d.audioCandidate ? "" : " (skipped by the connect filter)");
        }
    }
    std::sort(out.begin(), out.end(), [](const BtDevice_t& a, const BtDevice_t& b) {
        const int scoreA = (a.leAudio ? 4 : 0) + (a.audioCandidate ? 2 : 0) + (!a.name.empty() ? 1 : 0);
        const int scoreB = (b.leAudio ? 4 : 0) + (b.audioCandidate ? 2 : 0) + (!b.name.empty() ? 1 : 0);
        if (scoreA != scoreB) {
            return scoreA > scoreB;
        }
        return a.rssi > b.rssi;
    });
    return out;
}

void joinSetMember();  // below, after hal_bt_pair_second
// Encryption of the second connection in hal_bt_pair_second
constexpr uint8_t kSecondPair         = 0;  // pair from scratch
constexpr uint8_t kSecondBondThenPair = 1;  // saved key, and when it does not work - pairing
constexpr uint8_t kSecondBondOnly     = 2;  // saved key only

bool lehost::btConnect(const std::string& addr, uint8_t addrType, const std::string& knownName)
{
    std::lock_guard<std::recursive_mutex> op(g_bt.opLock);
    uint8_t peerAddr[6] = {};
    if (!parseMac(addr, peerAddr)) {
        return false;
    }

    {
        std::lock_guard<std::mutex> l(g_bt.stateLock);
        if (!g_bt.bondOnly) {
            g_bt.radioOn = true;  // manual connect turns the radio on; btReconnect (background) checks it is on
        }
    }
    if (!ensureRadioReady()) {
        return false;
    }

    const uint8_t extScanDis[6] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    sendHciCmd(0x2042, extScanDis, sizeof(extScanDis), 200);

    uint16_t prevHandle      = 0xFFFF;
    uint16_t prevSecond      = 0xFFFF;
    std::string resolvedName = knownName;
    int scanRssi             = -65;
    int adFlags              = -1;
    {
        std::lock_guard<std::mutex> l(g_bt.stateLock);
        prevHandle = g_bt.status.connHandle;
        prevSecond = g_bt.secondHandle;
        for (int i = 0; i < g_bt.nScanDevs; i++) {
            if (std::memcmp(g_bt.scanDevs[i].addr, peerAddr, 6) == 0) {
                scanRssi = g_bt.scanDevs[i].rssi;
                adFlags  = g_bt.scanDevs[i].adFlags;
                if (resolvedName.empty() && g_bt.scanDevs[i].name[0] != '\0') {
                    resolvedName = g_bt.scanDevs[i].name;
                }
                break;
            }
        }
        g_bt.status.state      = BtState::Connecting;
        g_bt.status.error      = BtError::None;
        g_bt.status.addr       = addr;
        g_bt.status.addrType   = addrType & 0x01;
        g_bt.status.name       = resolvedName;
        g_bt.status.rssi       = scanRssi;
        g_bt.status.connHandle = 0xFFFF;
        g_bt.status.gatt       = {};
        g_bt.connDone          = false;
        g_bt.connStatus        = 0xFF;
        g_bt.connHandle        = 0xFFFF;
    }

    // New main connection: the previous one and the second earbud of the previous set go away (otherwise LE Audio would
    // play to the new device and the old earbud, and joining the new set would be skipped)
    for (const uint16_t h : {prevHandle, prevSecond}) {
        if (h != 0xFFFF) {
            const uint8_t discParams[3] = {(uint8_t)(h & 0xFF), (uint8_t)((h >> 8) & 0x0F), 0x13};
            sendHciCmd(0x0406, discParams, sizeof(discParams), 400);
            vTaskDelay(pdMS_TO_TICKS(60));
        }
    }

    uint8_t extConn[26] = {
        0x00,
        g_bt.ownAddrType,
        (uint8_t)(addrType & 0x01),
        peerAddr[0], peerAddr[1], peerAddr[2], peerAddr[3], peerAddr[4], peerAddr[5],
        0x01,
        0x60, 0x00,
        0x60, 0x00,
        (uint8_t)app::leaudio::kAclIntervalUnits, 0x00,  // interval min = max: a multiple of 10 ms for CIS emulation
        (uint8_t)app::leaudio::kAclIntervalUnits, 0x00,
        0x00, 0x00,
        0x90, 0x01,
        0x00, 0x00,
        0x00, 0x00,
    };
    bool started = sendHciCmd(0x2043, extConn, sizeof(extConn), 500) && g_bt.cmdStatus == 0;
    if (!started) {
        uint8_t legConn[25] = {
            0x60, 0x00,
            0x60, 0x00,
            0x00,
            (uint8_t)(addrType & 0x01),
            peerAddr[0], peerAddr[1], peerAddr[2], peerAddr[3], peerAddr[4], peerAddr[5],
            g_bt.ownAddrType,
            (uint8_t)app::leaudio::kAclIntervalUnits, 0x00,
            (uint8_t)app::leaudio::kAclIntervalUnits, 0x00,
            0x00, 0x00,
            0x90, 0x01,
            0x00, 0x00,
            0x00, 0x00,
        };
        started = sendHciCmd(0x200D, legConn, sizeof(legConn), 500) && g_bt.cmdStatus == 0;
    }

    if (!started) {
        std::lock_guard<std::mutex> l(g_bt.stateLock);
        g_bt.status.state     = BtState::Failed;
        g_bt.status.error     = BtError::Rejected;
        g_bt.status.lastError = "C6 controller rejected the connection command";
        return false;
    }

    const int64_t deadline = esp_timer_get_time() + 5500LL * 1000LL;
    bool connected         = false;
    uint16_t handle        = 0xFFFF;
    while (esp_timer_get_time() < deadline) {
        {
            std::lock_guard<std::mutex> l(g_bt.stateLock);
            if (g_bt.connDone) {
                connected = (g_bt.connStatus == 0 && g_bt.connHandle != 0xFFFF);
                handle    = g_bt.connHandle;
                break;
            }
        }
        vTaskDelay(pdMS_TO_TICKS(15));
    }

    if (!connected) {
        sendHciCmd(0x200E, nullptr, 0, 300);
        vTaskDelay(pdMS_TO_TICKS(40));
        std::lock_guard<std::mutex> l(g_bt.stateLock);
        g_bt.status.state     = BtState::Failed;
        g_bt.status.error     = BtError::Timeout;
        g_bt.status.lastError = "Device did not answer the BLE connection request";
        return false;
    }

    {
        std::lock_guard<std::mutex> l(g_bt.stateLock);
        g_bt.status.connHandle = handle;
        g_bt.status.state      = BtState::Connected;
    }

    const uint8_t dleParams[6] = {(uint8_t)(handle & 0xFF), (uint8_t)((handle >> 8) & 0x0F), 251, 0, 0x48, 0x08};
    sendHciCmd(0x2022, dleParams, sizeof(dleParams), 250);

    const uint8_t rssiParams[2] = {(uint8_t)(handle & 0xFF), (uint8_t)((handle >> 8) & 0x0F)};
    if (sendHciCmd(0x1405, rssiParams, sizeof(rssiParams), 250) && g_bt.cmdStatus == 0 && g_bt.cmdRetLen >= 3) {
        std::lock_guard<std::mutex> l(g_bt.stateLock);
        g_bt.status.rssi = (int8_t)g_bt.cmdRet[2];
    }

    // A set in pairing mode: the "first earbud" (to be paired first) advertises LE Limited Discoverable (SIG
    // "Discovery, Pairing, and Connection for Earbuds" 3.5), the second is only connectable - flags for the log
    ESP_LOGI(kTag, "Connect %s: AD Flags %s0x%02X%s", addr.c_str(), adFlags < 0 ? "none " : "", adFlags < 0 ? 0 : adFlags,
             adFlags >= 0 && (adFlags & 0x01) ? " (Limited Discoverable)" : "");

    BtGattInfo_t gatt;
    inspectGattAndLeAudio(handle, peerAddr, (uint8_t)(addrType & 0x01), gatt);

    {
        std::lock_guard<std::mutex> l(g_bt.stateLock);
        if (!gatt.deviceName.empty()) {
            g_bt.status.name = gatt.deviceName;
        }
        g_bt.status.gatt = std::move(gatt);
    }

    ESP_LOGI(kTag, "Connected to %s (%s), GATT services: %u, PACS=%d, ASCS=%d, paired=%d", addr.c_str(),
             g_bt.status.name.c_str(), (unsigned)g_bt.status.gatt.services.size(), g_bt.status.gatt.hasPacs,
             g_bt.status.gatt.hasAscs, g_bt.status.gatt.paired);

    // CSIP: a set (size >= 2) - the second earbud joins in the background, like on Android (the coordinator connects
    // all members); LE Audio start waits for it (setJoining) so that stereo is set up right away
    const bool leOn = leAudioEnabled();
    bool join       = false;
    {
        std::lock_guard<std::mutex> l(g_bt.stateLock);
        join = leOn && g_bt.set.valid && g_bt.set.size >= 2 && g_bt.set.conn == handle &&
               !aclUpLocked(g_bt.secondHandle);
        g_bt.setJoining = join;
    }
    if (join) {
        leport::spawn("bt-set", 16 * 1024, [] { joinSetMember(); });
    }
    return true;
}

// Pairing a second device without disconnecting the first (diagnostics, POST /api/diag ble_probe.pair_second).
// Hypothesis H2 (docs/c6-link-layer.md): a Buds3 Pro set persists the second earbud's pairing only when the
// first one is connected - the phone connects both at once. Connect, encrypt with a saved key or pair, 3 s for
// set synchronization, disconnect only the second. No GATT and LE Audio: the first step of two connections (S2).
// setJoin (joining a set, joinSetMember): never takes over the main role, requires the same SIRK as the main one
// and disconnects on failure - otherwise an unencrypted link would be left unused and the earbud would stop
// advertising. The caller holds opLock.
bool connectSecondLocked(const char* addr, uint8_t addrType, bool keep, uint8_t auth, bool setJoin)
{
    uint8_t peer[6] = {};
    if (!parseMac(addr, peer) || !ensureRadioReady()) {
        return false;
    }
    uint16_t first = 0xFFFF;
    {
        std::lock_guard<std::mutex> l(g_bt.stateLock);
        first = g_bt.status.connHandle;
        if (setJoin && !aclUpLocked(first)) {
            return false;  // the main one vanished during the scan - nothing to join
        }
        g_bt.connDone   = false;
        g_bt.connStatus = 0xFF;
        g_bt.connHandle = 0xFFFF;
    }
    const uint8_t extScanDis[6] = {};
    sendHciCmd(0x2042, extScanDis, sizeof(extScanDis), 200);
    uint8_t extConn[26] = {
        0x00, g_bt.ownAddrType, (uint8_t)(addrType & 0x01), peer[0], peer[1], peer[2], peer[3], peer[4], peer[5], 0x01,
        0x60, 0x00,
        0x60, 0x00, (uint8_t)app::leaudio::kAclIntervalUnits, 0x00, (uint8_t)app::leaudio::kAclIntervalUnits, 0x00,
        0x00, 0x00, 0x90, 0x01, 0x00, 0x00, 0x00, 0x00,
    };
    if (!sendHciCmd(0x2043, extConn, sizeof(extConn), 500) || g_bt.cmdStatus != 0) {
        ESP_LOGW(kTag, "Second connection: the controller rejected the connection (status 0x%02X)", g_bt.cmdStatus);
        return false;
    }
    uint16_t handle        = 0xFFFF;
    const int64_t deadline = esp_timer_get_time() + 5500LL * 1000LL;
    while (esp_timer_get_time() < deadline && handle == 0xFFFF) {
        {
            std::lock_guard<std::mutex> l(g_bt.stateLock);
            if (g_bt.connDone) {
                handle = g_bt.connStatus == 0 ? g_bt.connHandle : 0xFFFE;
            }
        }
        vTaskDelay(pdMS_TO_TICKS(15));
    }
    if (handle >= 0xFFFE) {
        sendHciCmd(0x200E, nullptr, 0, 300);
        ESP_LOGW(kTag, "Second connection to %s was not established", addr);
        return false;
    }
    ESP_LOGI(kTag, "Second connection: %s handle 0x%03X (first 0x%03X stays)", addr, handle, first);
    // Without a main connection this becomes the main one: waitSmpOpcode and startLinkEncryption abort when
    // status.connHandle = 0xFFFF (they detect link loss only on the main one), and with keep it stays an ordinary
    // connection.
    const bool adopted = first == 0xFFFF;
    if (adopted) {
        std::lock_guard<std::mutex> l(g_bt.stateLock);
        g_bt.status.connHandle = handle;
        g_bt.status.state      = BtState::Connected;
        g_bt.status.addr       = addr;
        g_bt.status.addrType   = addrType & 0x01;
    }
    const uint8_t dle[6] = {(uint8_t)(handle & 0xFF), (uint8_t)((handle >> 8) & 0x0F), 251, 0, 0x48, 0x08};
    sendHciCmd(0x2022, dle, sizeof(dle), 250);
    // ATT MTU 247 like on the main one (inspectGattAndLeAudio): with the default 23 B the Sink PAC read and ASE
    // notifications in Codec Configured are truncated - the LE Audio procedure of the second earbud would not pass
    {
        const uint8_t mtuReq[3] = {0x02, 0xF7, 0x00};
        uint8_t rsp[32]         = {};
        uint16_t rspLen         = 0;
        const bool mtuOk = attRequest(handle, mtuReq, sizeof(mtuReq), rsp, rspLen, 600) && rspLen >= 3 && rsp[0] == 0x03;
        ESP_LOGI(kTag, "Second connection: ATT MTU %u", mtuOk ? (unsigned)(rsp[1] | (rsp[2] << 8)) : 23u);
    }

    // auth (kSecond*): saved key (reconnecting a set member), pairing from scratch, or both in turn. After a
    // failed encryption start the earbuds often drop the link, then pairing fails too. putBond replaces the entry
    // of the same device.
    bool ok = false;
    if (auth == kSecondBondOnly || auth == kSecondBondThenPair) {
        // Key only (second one with the main one from a saved key): a rejected key is not erased - there will be no
        // pairing
        ok = encryptWithBond(handle, peer, (uint8_t)(addrType & 0x01), auth != kSecondBondOnly) == BondEnc::Ok;
    }
    if (!ok && auth != kSecondBondOnly) {
        BtGattInfo_t gatt;
        ok = performSmpScJustWorks(handle, peer, (uint8_t)(addrType & 0x01), gatt);
        if (!ok) {
            ESP_LOGW(kTag, "Second connection: pairing failed at stage %s (Pairing Failed 0x%02X)", g_smpStage,
                     g_smpFailReason);
        }
    }
    if (ok) {
        const int sameSet = readCsis(handle, peer, (uint8_t)(addrType & 0x01), false);
        if (setJoin && sameSet != 1) {
            ESP_LOGW(kTag, "Second connection: not the same pair (SIRK %s) - dropping", sameSet == 0 ? "different" : "unknown");
            ok = false;
        }
    }
    if (setJoin && !ok) {
        const uint8_t d[3] = {(uint8_t)(handle & 0xFF), (uint8_t)((handle >> 8) & 0x0F), 0x13};
        sendHciCmd(0x0406, d, sizeof(d), 400);
        return false;
    }
    if (keep) {
        // Like Android with a set: the main one stays connected and the second joins in the same session. An encrypted
        // second with a connected main one = second member for LE Audio (stereo, halBtLeAudioLink().acl2).
        uint16_t prev = 0xFFFF;
        if (ok && !adopted) {
            std::lock_guard<std::mutex> l(g_bt.stateLock);
            if (aclUpLocked(handle)) {
                prev              = g_bt.secondHandle;
                g_bt.secondHandle = handle;
                std::memcpy(g_bt.secondPeer, peer, 6);
            }
        }
        if (prev != 0xFFFF && prev != handle) {
            const uint8_t d[3] = {(uint8_t)(prev & 0xFF), (uint8_t)((prev >> 8) & 0x0F), 0x13};
            sendHciCmd(0x0406, d, sizeof(d), 400);
        }
        ESP_LOGI(kTag, "Second connection: %s - stays connected (handle 0x%03X%s)", ok ? "encrypted" : "unencrypted",
                 handle, ok && !adopted ? ", second set member" : "");
        return ok;
    }
    ESP_LOGI(kTag, "Second connection: %s - waiting 3 s for set synchronization", ok ? "paired" : "unencrypted");
    vTaskDelay(pdMS_TO_TICKS(3000));
    const uint8_t disc[3] = {(uint8_t)(handle & 0xFF), (uint8_t)((handle >> 8) & 0x0F), 0x13};
    sendHciCmd(0x0406, disc, sizeof(disc), 400);
    if (adopted) {
        std::lock_guard<std::mutex> l(g_bt.stateLock);
        if (g_bt.status.connHandle == handle) {
            g_bt.status.connHandle = 0xFFFF;
            g_bt.status.state      = g_bt.radioOn ? BtState::Idle : BtState::Off;
        }
    }
    return ok;
}

// Second earbud of the set in a scan (under opLock): RSI resolved with the set's SIRK (CSIP 4.7), and without RSI - an
// address resolved with the IRK of a bond other than the main connection. The main one (same bond) is skipped - it also
// advertises the set's RSI.
bool findSetMemberLocked(const uint8_t sirk[16], int mainBond, int ms, uint8_t outAddr[6], uint8_t& outType)
{
    const uint8_t extScanParam[8] = {0x01, 0x00, 0x01, 0x01, 0x60, 0x00, 0x30, 0x00};
    const uint8_t extScanEn[6]    = {0x01, 0x00, 0x00, 0x00, 0x00, 0x00};
    const uint8_t extScanDis[6]   = {};
    {
        std::lock_guard<std::mutex> l(g_bt.stateLock);
        g_bt.nScanDevs = 0;
    }
    if (!(sendHciCmd(0x2041, extScanParam, sizeof(extScanParam)) && g_bt.cmdStatus == 0 &&
          sendHciCmd(0x2042, extScanEn, sizeof(extScanEn)) && g_bt.cmdStatus == 0)) {
        ESP_LOGW(kTag, "Set: scan did not start (status 0x%02X)", g_bt.cmdStatus);
        return false;
    }
    bool byRsi = false, byBond = false;
    const int64_t deadline = esp_timer_get_time() + (int64_t)ms * 1000;
    // Entries with RSI already compared against the SIRK (the result will not change). The whole table in chunks of 16
    // - previously only the first 16, and in a crowd (Tab5 2026-11-02, dozens of phones with a private address) the
    // earbud was further down
    uint64_t rsiChecked = 0;
    while (!byRsi && esp_timer_get_time() < deadline) {
        vTaskDelay(pdMS_TO_TICKS(150));
        for (int base = 0; base < 64 && !byRsi; base += 16) {
            AdvRecord recs[16];
            int n = 0;
            {
                std::lock_guard<std::mutex> l(g_bt.stateLock);
                for (int i = base; i < g_bt.nScanDevs && i < base + 16; i++) {
                    const AdvRecord& r = g_bt.scanDevs[i];
                    if (r.hasRsi && (rsiChecked >> i & 1u) == 0) {
                        rsiChecked |= 1ull << i;
                        recs[n++] = r;
                    } else if (!r.hasRsi && app::btbond::isResolvablePrivate(r.addrType, r.addr)) {
                        recs[n++] = r;
                    }
                }
            }
            for (int i = 0; i < n && !byRsi; i++) {
                int bond = -1;
                {
                    std::lock_guard<std::mutex> l(g_bt.stateLock);
                    bond = app::btbond::findBond(g_bt.bonds, recs[i].addrType, recs[i].addr, aes128Be);
                }
                if (mainBond >= 0 && bond == mainBond) {
                    continue;
                }
                if (recs[i].hasRsi && app::csip::rsiMatches(sirk, recs[i].rsi, aes128Be)) {
                    byRsi = true;
                } else if (bond >= 0 && !byBond && !recs[i].hasRsi) {
                    byBond = true;  // fallback (no RSI in advertisement): keep looking for RSI to the window end
                } else {
                    continue;
                }
                std::memcpy(outAddr, recs[i].addr, 6);
                outType = recs[i].addrType;
            }
        }
    }
    sendHciCmd(0x2042, extScanDis, sizeof(extScanDis));
    if (byRsi || byBond) {
        int flags = -1;
        {
            std::lock_guard<std::mutex> l(g_bt.stateLock);
            for (int i = 0; i < g_bt.nScanDevs; i++) {
                if (std::memcmp(g_bt.scanDevs[i].addr, outAddr, 6) == 0) {
                    flags = g_bt.scanDevs[i].adFlags;
                }
            }
        }
        ESP_LOGI(kTag, "Set: second earbud %02X:%02X:%02X:%02X:%02X:%02X recognized by %s, AD Flags %d",
                 outAddr[5], outAddr[4], outAddr[3], outAddr[2], outAddr[1], outAddr[0],
                 byRsi ? "RSI (SIRK)" : "IRK of a saved bond", flags);
    }
    return byRsi || byBond;
}

void joinSetMember()
{
    uint8_t sirk[16] = {}, mainAddr[6] = {};
    uint8_t mainType = 0;
    bool ok          = false;
    bool fresh       = false;
    {
        std::lock_guard<std::mutex> l(g_bt.stateLock);
        ok = g_bt.set.valid && g_bt.set.size >= 2 && g_bt.set.conn == g_bt.status.connHandle &&
             !aclUpLocked(g_bt.secondHandle) && parseMac(g_bt.status.addr, mainAddr);
        std::memcpy(sirk, g_bt.set.sirk, 16);
        mainType = g_bt.status.addrType;
        fresh    = g_bt.set.fresh;
    }
    uint8_t addr[6] = {};
    uint8_t type    = 0;
    bool found      = false;
    bool joined     = false;
    // Scan and connection under one opLock: btDisconnect / btConnect cannot change the main connection between them
    std::unique_lock<std::recursive_mutex> op(g_bt.opLock, std::defer_lock);
    if (ok) {
        op.lock();
        int mainBond = -1;
        {
            std::lock_guard<std::mutex> l(g_bt.stateLock);
            mainBond = app::btbond::findBond(g_bt.bonds, mainType, mainAddr, aes128Be);
        }
        found = findSetMemberLocked(sirk, mainBond, 4000, addr, type);
        if (found) {
            std::lock_guard<std::mutex> l(g_bt.stateLock);
            found = g_bt.set.valid && g_bt.set.conn == g_bt.status.connHandle && !aclUpLocked(g_bt.secondHandle);
        }
    }
    std::memset(sirk, 0, sizeof(sirk));
    if (found) {
        char mac[20];
        formatMac(addr, mac);
        // A set is one record in the earbuds (pairing one invalidates the other's key, docs/c6-link-layer.md): main
        // just paired - pair the second right away in the same session (CSIP 4.6.2, SIG "Discovery, Pairing, and
        // Connection for Earbuds" table 3.3); main with a saved key - the second only with its own key, never from
        // scratch alone (SIG ibid. 3.8: on error pair the whole set).
        joined = connectSecondLocked(mac, type, true, fresh ? kSecondPair : kSecondBondOnly, true);
        ESP_LOGI(kTag, "Set: second earbud %s", joined ? "joined"
                     : fresh ? "NOT joined - one plays"
                             : "NOT joined - one plays (cause above: dropped link or key rejected 0x06 - "
                               "only then pair both again in pairing mode)");
    } else if (ok) {
        ESP_LOGW(kTag, "Set: second earbud not in range (4 s scan) or the main one disconnected - one plays");
    }
    std::lock_guard<std::mutex> l(g_bt.stateLock);
    g_bt.setJoining = false;
}

bool lehost::btTakeMediaCommand(BtMediaCmd& out)
{
    std::lock_guard<std::mutex> l(g_bt.stateLock);
    if (g_bt.mediaCmdCount == 0) {
        return false;
    }
    constexpr uint8_t kCap = sizeof(g_bt.mediaCmds) / sizeof(g_bt.mediaCmds[0]);
    out                    = g_bt.mediaCmds[g_bt.mediaCmdHead];
    g_bt.mediaCmdHead      = (uint8_t)((g_bt.mediaCmdHead + 1) % kCap);
    g_bt.mediaCmdCount--;
    return true;
}

void lehost::btSetMediaState(bool active, bool playing, const std::string& title, uint32_t durationMs)
{
    // Media State (MCS 3.18): Inactive without a track, otherwise Playing / Paused
    const uint8_t state = !active ? 0x00 : playing ? 0x01 : 0x02;
    bool stateChanged = false, trackChanged = false;
    uint16_t conns[2] = {0xFFFF, 0xFFFF};
    {
        std::lock_guard<std::mutex> l(g_bt.stateLock);
        stateChanged          = state != g_bt.localMediaState;
        trackChanged          = title != g_bt.localTrackTitle;
        g_bt.localMediaState  = state;
        g_bt.localTrackTitle  = title;
        g_bt.localTrackDurCs  = durationMs == 0 ? -1 : (int32_t)std::min<uint32_t>(durationMs / 10, INT32_MAX);
        if ((g_bt.mediaStateNtfMask & 1) && g_bt.status.state == BtState::Connected) {
            conns[0] = g_bt.status.connHandle;
        }
        if ((g_bt.mediaStateNtfMask & 2) && aclUpLocked(g_bt.secondHandle)) {
            conns[1] = g_bt.secondHandle;
        }
    }
    // Notifications only to earbuds that enabled the Media State CCCD (MCP client); Track Changed also then, because
    // the client reads title and duration after it (MCS 3.4)
    const uint8_t stNtf[4] = {0x1B, (uint8_t)(kMediaState & 0xFF), (uint8_t)(kMediaState >> 8), state};
    const uint8_t tcNtf[3] = {0x1B, (uint8_t)(kTrackChanged & 0xFF), (uint8_t)(kTrackChanged >> 8)};
    for (const uint16_t c : conns) {
        if (c == 0xFFFF) {
            continue;
        }
        if (trackChanged) {
            sendAclL2cap(c, 0x0004, tcNtf, sizeof(tcNtf));
        }
        if (stateChanged) {
            sendAclL2cap(c, 0x0004, stNtf, sizeof(stNtf));
        }
    }
}

lehost::BtReconnect lehost::btReconnect(const std::string& addr, uint8_t addrType, const std::string& name)
{
    std::lock_guard<std::recursive_mutex> op(g_bt.opLock);
    {
        std::lock_guard<std::mutex> l(g_bt.stateLock);
        // During the auto-connect scan (opLock) the user may have connected something manually or turned Bluetooth off
        // - then nothing
        if (!g_bt.radioOn || g_bt.status.connHandle != 0xFFFF || g_bt.status.state == BtState::Connecting) {
            return BtReconnect::Failed;
        }
        g_bt.bondOnly     = true;
        g_bt.bondRejected = false;
    }
    const bool ok        = btConnect(addr, addrType, name);
    bool paired          = false;
    bool rejected        = false;
    bool connected       = false;
    {
        std::lock_guard<std::mutex> l(g_bt.stateLock);
        g_bt.bondOnly = false;
        connected     = g_bt.status.connHandle != 0xFFFF;
        paired        = connected && g_bt.status.gatt.paired;
        rejected      = g_bt.bondRejected;
    }
    if (ok && paired) {
        return BtReconnect::Connected;
    }
    if (connected) {
        btDisconnect();  // unencrypted link: LE Audio will not start anyway, and it would block auto-connect
    }
    return rejected ? BtReconnect::KeyRejected : BtReconnect::Failed;
}

int lehost::btBondCount()
{
    std::lock_guard<std::mutex> l(g_bt.stateLock);
    return g_bt.initialized ? (int)app::btbond::bondCount(g_bt.bonds) : -1;
}

// Is the missing second earbud of the set advertising now? Scan only (RSI / IRK, like joinSetMember), no
// connection: while a CIG streams, connecting the second earbud fails (0x3E) - the caller stops the stream first,
// and only when the earbud is really there (otherwise every probe would cut the audio).
bool lehost::btFindSetMember(int ms)
{
    uint8_t sirk[16] = {}, mainAddr[6] = {};
    uint8_t mainType = 0;
    {
        std::lock_guard<std::mutex> l(g_bt.stateLock);
        if (!(g_bt.set.valid && g_bt.set.size >= 2 && g_bt.set.conn == g_bt.status.connHandle) ||
            aclUpLocked(g_bt.secondHandle) || g_bt.setJoining || !parseMac(g_bt.status.addr, mainAddr)) {
            return false;
        }
        std::memcpy(sirk, g_bt.set.sirk, 16);
        mainType = g_bt.status.addrType;
    }
    std::lock_guard<std::recursive_mutex> op(g_bt.opLock);
    int mainBond = -1;
    {
        std::lock_guard<std::mutex> l(g_bt.stateLock);
        mainBond = app::btbond::findBond(g_bt.bonds, mainType, mainAddr, aes128Be);
    }
    uint8_t addr[6] = {};
    uint8_t type    = 0;
    const bool found = findSetMemberLocked(sirk, mainBond, ms, addr, type);
    std::memset(sirk, 0, sizeof(sirk));
    return found;
}

bool lehost::btJoinSet()
{
    {
        std::lock_guard<std::mutex> l(g_bt.stateLock);
        if (g_bt.setJoining || aclUpLocked(g_bt.secondHandle)) {
            return aclUpLocked(g_bt.secondHandle);
        }
        if (!(g_bt.set.valid && g_bt.set.size >= 2 && g_bt.set.conn == g_bt.status.connHandle)) {
            return false;
        }
        g_bt.setJoining = true;  // joinSetMember clears it at the end; LE Audio start waits meanwhile
    }
    joinSetMember();
    std::lock_guard<std::mutex> l(g_bt.stateLock);
    return aclUpLocked(g_bt.secondHandle);
}

void lehost::btDisconnect()
{
    std::lock_guard<std::recursive_mutex> op(g_bt.opLock);
    uint16_t h = 0xFFFF, h2 = 0xFFFF;
    {
        std::lock_guard<std::mutex> l(g_bt.stateLock);
        h  = g_bt.status.connHandle;
        h2 = g_bt.secondHandle;
    }
    if (h2 != 0xFFFF) {
        // The second set member leaves together with the main one (one device for the user)
        const uint8_t d2[3] = {(uint8_t)(h2 & 0xFF), (uint8_t)((h2 >> 8) & 0x0F), 0x13};
        sendHciCmd(0x0406, d2, sizeof(d2), 400);
    }
    if (h != 0xFFFF) {
        const uint8_t discParams[3] = {(uint8_t)(h & 0xFF), (uint8_t)((h >> 8) & 0x0F), 0x13};
        sendHciCmd(0x0406, discParams, sizeof(discParams), 400);
        vTaskDelay(pdMS_TO_TICKS(50));
    } else {
        sendHciCmd(0x200E, nullptr, 0, 200);
    }
    std::lock_guard<std::mutex> l(g_bt.stateLock);
    dropLinkMetaLocked(h);
    dropLinkMetaLocked(h2);
    g_bt.status.connHandle = 0xFFFF;
    g_bt.encEnabled        = 0;
    g_bt.status.state      = g_bt.radioOn ? BtState::Idle : BtState::Off;
    g_bt.status.addr.clear();
    g_bt.status.name.clear();
    g_bt.status.rssi = 0;
    g_bt.status.gatt = {};
}
