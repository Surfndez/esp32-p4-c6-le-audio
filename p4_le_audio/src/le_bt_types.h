#pragma once
// Internal types of the LE Audio host (taken from the HAL interface of the firmware this project was extracted from,
// unchanged in meaning).
// The public C API (include/le_audio.h) maps them to plain C structs.

#include <cstdint>
#include <string>
#include <vector>

namespace lehost {

enum class BtState : uint8_t { Off, Idle, Scanning, Connecting, Connected, Failed };
enum class BtError : uint8_t { None, Timeout, Rejected, Lost, RadioDown, Other };

struct BtGattService_t {
    uint16_t startHandle = 0;
    uint16_t endHandle   = 0;
    uint16_t uuid16      = 0;
    std::string uuidHex;
    std::string name;
};

struct BtGattInfo_t {
    uint16_t mtu           = 23;
    bool hasPacs           = false;  // 0x1850 Published Audio Capabilities Service
    bool hasAscs           = false;  // 0x184E Audio Stream Control Service
    bool hasCas            = false;  // 0x1853 Common Audio Service
    bool hasVcs            = false;  // 0x1844 Volume Control Service
    bool hasCsis           = false;  // 0x1846 Coordinated Set Identification Service
    bool hasTmAp           = false;  // 0x1854 HAP / 0x1855 TMAP
    bool authRequired      = false;  // earbuds require encryption/pairing to read values
    bool paired            = false;  // link encrypted (fresh SMP pairing or saved bond)
    uint8_t smpAuthReq     = 0;      // AuthReq flags from SMP Pairing Response
    int batteryPct         = -1;     // 0-100 %, -1 = unknown
    int volume             = -1;     // 0-255 (VCS), -1 = unknown
    int sinkAseCount       = 0;      // Sink ASE characteristics (0x2BC4)
    int srcAseCount        = 0;      // Source ASE characteristics (0x2BC5)
    uint32_t sinkLocations = 0;      // PACS Sink Audio Locations (0x2BCA)
    uint32_t srcLocations  = 0;      // PACS Source Audio Locations (0x2BCC)
    std::string deviceName;          // GAP Device Name (0x2A00)
    std::string sinkPacSummary;
    std::string srcPacSummary;
    std::string sinkPacHex;
    std::string srcPacHex;
    std::vector<BtGattService_t> services;
};

struct BtStatus_t {
    BtState state       = BtState::Off;
    BtError error       = BtError::None;
    std::string bdAddr;         // controller (C6) address
    std::string coprocessorFw;  // e.g. "1.4.82"
    std::string addr;           // connected / connecting device
    uint8_t addrType    = 0;
    std::string name;
    int rssi            = 0;
    uint16_t connHandle = 0xFFFF;
    uint32_t rxPkts     = 0;
    uint32_t txPkts     = 0;
    std::string lastError;
    BtGattInfo_t gatt;
};

struct BtDevice_t {
    std::string addr;
    uint8_t addrType    = 0;  // 0 = public, 1 = random
    std::string name;
    std::string mfgName;
    uint16_t mfgId      = 0xFFFF;
    std::string mfgHex;
    int rssi            = -127;
    bool extAdv         = false;
    bool leAudio        = false;
    bool audioCandidate = false;
    uint16_t appearance = 0;
    std::vector<uint16_t> uuids16;
    bool bonded         = false;  // saved device: identity address or RPA resolved with its IRK
    bool inConnectedSet = false;
};

// Light connection state for once-per-second polling (no GATT copy).
struct BtLink_t {
    BtState state       = BtState::Off;
    uint16_t connHandle = 0xFFFF;
    bool paired         = false;
    bool leAudio        = false;  // earbuds with ASCS
    bool streamActive   = false;  // LE Audio stream configuring or streaming
    bool setMissing     = false;  // CSIP set of >= 2, second member not connected and not joining
    int batteryPct      = -1;
};

enum class BtMediaCmd : uint8_t { Play, Pause, Stop, Next, Previous };
enum class BtReconnect : uint8_t { Connected, Failed, KeyRejected };

enum class LeAudioPhase : uint8_t { Off, Configuring, Streaming, Failed };
struct LeAudioStatus_t {
    LeAudioPhase phase = LeAudioPhase::Off;
    bool mic           = false;
    int sinkRate = 0, sinkOctets = 0, srcRate = 0, srcOctets = 0;
    uint32_t presentationDelayUs = 0;
    uint32_t txSdu = 0, txDropped = 0, rxSdu = 0, rxLost = 0;
    uint32_t lateWakes = 0, slowSends = 0, pcmShort = 0;
    uint32_t maxLateUs = 0, maxEncUs = 0, maxSendUs = 0;
    uint32_t maxStallUs = 0, resyncs = 0, skippedFrames = 0;
    uint32_t dropNoCredit = 0, dropSendFail = 0;
    int creditsMin = -1, creditsTotal = 0;
    uint32_t dropDeadline = 0, txqSent = 0;
    int txqMax = 0;
    uint32_t creditsRecovered = 0;
    int members = 0;
    bool stereo = false;
    std::string lastError;
    // Per stream member (filled by btLeAudioStatus from the live stream state): Audio Location of its Sink ASE
    // (0x1 front left, 0x2 front right), CIS up, CIS handle (0xFFFF = none)
    uint32_t location[2] = {0, 0};
    bool cisUp[2]        = {false, false};
    uint16_t cisHandle[2] = {0xFFFF, 0xFFFF};
};

/* ---------------- host functions (le_host.cpp) ---------------- */
void btSetRadio(bool on);
BtStatus_t btStatus();
BtLink_t btLink();
std::vector<BtDevice_t> btScan(int durationS);
bool btConnect(const std::string& addr, uint8_t addrType, const std::string& knownName);
void btDisconnect();
bool btTakeMediaCommand(BtMediaCmd& out);
void btSetMediaState(bool active, bool playing, const std::string& title, uint32_t durationMs);
BtReconnect btReconnect(const std::string& addr, uint8_t addrType, const std::string& name);
int btBondCount();
bool btJoinSet();
bool btFindSetMember(int ms);  // scan only: is the missing second earbud of the set advertising?

/* ---------------- stream functions (le_stream.cpp) ---------------- */
bool btLeAudioStart(bool mic);
bool btLeAudioRestart(bool mic);
void btLeAudioStop();
LeAudioStatus_t btLeAudioStatus();

}  // namespace lehost
