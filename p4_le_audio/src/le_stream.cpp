// BAP unicast client and ISO stream. Taken from the hal_le_audio.cpp of the firmware this project was extracted from
// (Tab5 HAL) with the same protocol paths; HAL glue replaced (le_port.h), blocking PCM input added for applications
// without an I2S clock.
//
// LE Audio unicast (BAP Unicast Client) on the Tab5: PACS -> ASCS (Config Codec, Config QoS, Enable) ->
// HCI CIG/CIS -> ISO path over HCI -> LC3 (liblc3) in both directions.
//
// Layout: P4 = host, BLE controller = C6 (esp_hosted, hal_bt.cpp) - only standard HCI LE ISO commands and HCI ISO
// packets (0x05), so the same code works with a real ISO controller. btLeAudioStart blocks (caller's thread, PSRAM);
// the stream is served by the "le-audio" thread (PSRAM): every 10 ms it encodes an LC3 frame from the PCM queue of the
// audio path (hal_audio.cpp, output kOutBtLe) and sends the SDU, and decodes the microphone SDU into the 16 kHz queue
// (kMicBtLe).
//
// Set members (CSIP): m[0] = primary connection (PACS/ASCS handles from the GATT inspection in hal_bt), m[1] = second
// member, when hal_bt has it connected and encrypted (halBtLeAudioLink().acl2; PACS/ASCS are discovered here via ATT).
// One CIG, CIS_ID = member index, each CIS unidirectional to the earbud except the microphone of m[0]. Two earbuds
// with different locations (0x1 / 0x2) each get their own channel, a single one gets mono (L + R) / 2.
// docs/c6-link-layer.md. Data from the air and from the controller is checked by app/bt/le_audio_model (lengths,
// offsets); no raw reads here.

#include "le_bt_types.h"
#include "le_host_internal.h"
#include "le_port.h"

#include <bt/le_audio_model.h>

#include <algorithm>
#include <atomic>
#include <cstdarg>
#include <cmath>
#include <condition_variable>
#include <cstdio>
#include <cstring>
#include <functional>
#include <mutex>
#include <new>

#include "esp_attr.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "lc3.h"

namespace {

using namespace app::leaudio;
using Phase = lehost::LeAudioPhase;

constexpr const char* kTag    = "le_audio";
constexpr int kSinkRate       = 48000;  // sample rate of the Tab5 output path
constexpr int kFrameUs        = 10000;
constexpr int kSinkFrame      = 480;    // 10 ms at 48 kHz
constexpr int kMicRate        = 16000;  // sample rate of the Tab5 speech path (micRead)
constexpr uint8_t kCigId      = 1;
constexpr int kMaxMembers     = 2;      // earbuds of the set (CIS in the CIG; the CIS engine in the C6 has two)
constexpr size_t kDownRingFr  = kSinkFrame * 12;  // 120 ms stereo 48 kHz
constexpr size_t kDownMaxFr   = kSinkFrame * 6;   // above 60 ms queued - the frame is dropped (clock drift)
constexpr size_t kMicRingSmp  = kMicRate;          // 1 s mono 16 kHz
constexpr int kRxQueue        = 8;

// Simple sample ring in PSRAM (one writer, one reader, under the caller's lock).
struct PcmRing {
    int16_t* buf = nullptr;
    size_t cap = 0, head = 0, fill = 0;
    bool alloc(size_t n)
    {
        free();
        buf = static_cast<int16_t*>(heap_caps_calloc(n, sizeof(int16_t), MALLOC_CAP_SPIRAM));
        cap = buf != nullptr ? n : 0;
        return buf != nullptr;
    }
    void free()
    {
        heap_caps_free(buf);
        buf = nullptr;
        cap = head = fill = 0;
    }
    size_t push(const int16_t* s, size_t n)
    {
        n = std::min(n, cap - fill);
        for (size_t i = 0; i < n; i++) {
            buf[(head + fill + i) % cap] = s[i];
        }
        fill += n;
        return n;
    }
    size_t pop(int16_t* d, size_t n)
    {
        n = std::min(n, fill);
        for (size_t i = 0; i < n; i++) {
            if (d != nullptr) {
                d[i] = buf[(head + i) % cap];
            }
        }
        head = (head + n) % (cap == 0 ? 1 : cap);
        fill -= n;
        return n;
    }
};

struct RxSdu {
    uint16_t seq    = 0;
    IsoStatus status = IsoStatus::Valid;
    uint16_t len    = 0;
    uint8_t data[160];
};

// One earbud of the set: its ACL, ASCS handles, latest notifications and CIS.
struct Member {
    uint16_t acl = 0xFFFF;
    bool aclLost = false;
    uint16_t pacsStart = 0, pacsEnd = 0;
    uint16_t cpH = 0, sinkH = 0, srcH = 0;  // ATT value handles; srcH only for m[0] with the microphone
    uint8_t sinkId = 0, srcId = 0;          // ASE IDs after the state read (0 = not known yet - teardown skips)
    // Latest notifications (the sequence grows with each one - the waiter compares)
    AseInfo sinkAse, srcAse;
    uint32_t sinkSeq = 0, srcSeq = 0;
    CpResult cp;
    uint32_t cpSeq = 0;
    CisEstablished cis;
    uint32_t cisSeq = 0;
    uint16_t cisHandle = 0xFFFF;
    bool cisUp     = false;
    bool sinkOff   = false;  // the earbud ended the Sink ASE itself during playback - the encoder skips it
    std::string protoError;  // illegal ASE state transition - ends the procedure for this earbud
    bool pathIn = false, pathOut = false;  // ISO paths set up on the CIS (LE Setup ISO Data Path) - removed by teardown
    CodecCfg sinkCfg;
    QosCfg sinkQos;
};

struct LeAudio {
    std::mutex lock;  // guards everything below except the atomics; never held across hal_bt calls
    std::condition_variable cv;
    std::recursive_mutex opLock;  // start / stop - one procedure at a time (restart: stop + start under one lock)

    lehost::LeAudioStatus_t st;
    // ATT handles for recognizing notifications (halLeAudioOwnsAttr runs under the hal_bt lock - atomics only). Earbuds
    // of the same model usually share handles - the member is selected by the connection handle (halLeAudioOnAttValue).
    std::atomic<uint16_t> attrCp[kMaxMembers]{}, attrSink[kMaxMembers]{}, attrSrc[kMaxMembers]{};
    Member m[kMaxMembers];
    int nm      = 0;      // members of the current start / stream
    bool stereo = false;  // two earbuds with locations 0x1 and 0x2 - each gets its own channel
    // ISO
    IsoCredits credits;
    uint16_t isoLen = 0;
    uint32_t nocpSeq = 0;  // successive NOCPs - the stream thread waits for a change (txq), not for the pool state
    // SDUs waiting for a credit (txq switch): stream thread only, one queue per member and its CIS handle
    IsoTxQueue txq[kMaxMembers];
    uint16_t txqH[kMaxMembers] = {};
    // Stream
    std::atomic<bool> running{false}, sinkReady{false}, micReady{false};
    std::atomic<bool> threadDone{true};
    PcmRing down;  // 48 kHz stereo from the output path
    PcmRing mic;   // 16 kHz mono for micRead (from m[0])
    RxSdu rxq[kRxQueue];
    int rxHead = 0, rxCount = 0;
    std::condition_variable micCv;
    std::condition_variable pcmCv;  // le_audio_write_pcm: room in `down` (the stream thread took a frame)
    std::atomic<int> volume{60};
    CodecCfg srcCfg;
};

// Stream state (with the txq queues ~7 kB) and the ISO reassembler live in PSRAM instead of internal .bss - like g_bt
// in hal_bt.cpp (noinit + construction during global constructors, PSRAM already mapped; tasks only, no interrupts).
EXT_RAM_NOINIT_ATTR alignas(LeAudio) uint8_t g_mem[sizeof(LeAudio)];
EXT_RAM_NOINIT_ATTR alignas(IsoReassembler) uint8_t g_reasmMem[sizeof(IsoReassembler)];
LeAudio& g = *new (g_mem) LeAudio();
IsoReassembler& g_reasm = *new (g_reasmMem) IsoReassembler();  // SDIO receive thread only
std::atomic<uint8_t> g_labPhy{0};  // CIS PHY for the next start from a measurement session (0 = preference)
// Credit-based SDU queue (LeAudioFix in le_host_internal.h) - after a restart as LeAudioFix{} (enabled).
std::atomic<bool> g_fixTxq{LeAudioFix{}.txq};
std::atomic<int> g_fixTxqDeadlineMs{LeAudioFix{}.txqDeadlineMs};
// Gaps between NOCPs in the stream (under g.lock): last time and the maximum since the previous halLeAudioNocpGapTake
int64_t g_lastNocpUs   = 0;
uint32_t g_nocpGapMaxUs = 0;

std::string fmt(const char* f, ...) __attribute__((format(printf, 1, 2)));
std::string fmt(const char* f, ...)
{
    char b[160];
    va_list ap;
    va_start(ap, f);
    std::vsnprintf(b, sizeof(b), f, ap);
    va_end(ap);
    return b;
}

/* ------------------------------ waiting for events ------------------------------ */

// The primary or earbud mi lost its ACL or broke the ASE state machine (the procedure makes no sense for it).
bool brokenLocked(int mi)
{
    return g.m[0].aclLost || !g.m[0].protoError.empty() || g.m[mi].aclLost || !g.m[mi].protoError.empty();
}

// Waits (without holding the lock across hal_bt calls) until pred() returns true under g.lock. false = timed out or
// brokenLocked(mi) - the second earbud does not interrupt waiting for the primary.
bool waitFor(int ms, const std::function<bool()>& pred, int mi = 0)
{
    std::unique_lock<std::mutex> l(g.lock);
    return g.cv.wait_for(l, std::chrono::milliseconds(ms), [&] { return pred() || brokenLocked(mi); }) &&
           !brokenLocked(mi);
}

std::string lastErrorOr(const std::string& dflt)
{
    std::lock_guard<std::mutex> l(g.lock);
    return g.st.lastError.empty() ? dflt : g.st.lastError;
}

void fail(const std::string& why)
{
    ESP_LOGW(kTag, "LE Audio: %s", why.c_str());
    std::lock_guard<std::mutex> l(g.lock);
    g.st.phase     = Phase::Failed;
    g.st.lastError = why;
}

/* ------------------------------------ ATT ------------------------------------ */

// Timeout of one ATT request. ATT allows one request at a time, and a late response after the timeout would land on
// the next request - with a 30 ms interval and Wi-Fi coexistence 800 ms was sometimes too little; Core allows 30 s.
constexpr int kAttTimeoutMs = 3000;

bool attWrite(uint16_t acl, uint16_t h, const uint8_t* v, size_t n)
{
    uint8_t req[200];
    if (n + 3 > sizeof(req)) {
        return false;
    }
    req[0] = 0x12;  // Write Request
    req[1] = (uint8_t)(h & 0xFF);
    req[2] = (uint8_t)(h >> 8);
    std::memcpy(req + 3, v, n);
    uint8_t rsp[8];
    uint16_t rl = 0;
    return halBtAtt(acl, req, (uint16_t)(n + 3), rsp, sizeof(rsp), &rl, kAttTimeoutMs) && rl >= 1 && rsp[0] == 0x13;
}

// Value of a characteristic with a UUID16 in the range (Read By Type + Read, when the value may have been truncated).
bool attReadUuid(uint16_t acl, uint16_t s, uint16_t e, uint16_t uuid, uint8_t* out, size_t cap, size_t& n)
{
    n = 0;
    const uint8_t req[7] = {0x08, (uint8_t)(s & 0xFF), (uint8_t)(s >> 8), (uint8_t)(e & 0xFF), (uint8_t)(e >> 8),
                            (uint8_t)(uuid & 0xFF), (uint8_t)(uuid >> 8)};
    uint8_t rsp[256];
    uint16_t rl = 0;
    if (!halBtAtt(acl, req, sizeof(req), rsp, sizeof(rsp), &rl, kAttTimeoutMs) || rl < 4 || rsp[0] != 0x09) {
        return false;
    }
    const uint8_t item = rsp[1];
    if (item < 2 || 2u + item > rl) {
        return false;
    }
    const uint16_t vh = (uint16_t)(rsp[2] | (rsp[3] << 8));
    const uint8_t rd[3] = {0x0A, (uint8_t)(vh & 0xFF), (uint8_t)(vh >> 8)};
    uint8_t r2[256];
    uint16_t l2 = 0;
    if (halBtAtt(acl, rd, sizeof(rd), r2, sizeof(r2), &l2, kAttTimeoutMs) && l2 >= 1 && r2[0] == 0x0B) {
        n = std::min<size_t>(l2 - 1, cap);
        std::memcpy(out, r2 + 1, n);
        return true;
    }
    n = std::min<size_t>(item - 2u, cap);
    std::memcpy(out, rsp + 4, n);
    return true;
}

// A characteristic's CCCD = descriptor 0x2902 right after the value (Find Information in a narrow range); enables
// notifications.
bool subscribe(uint16_t acl, uint16_t valH)
{
    const uint16_t s = (uint16_t)(valH + 1), e = (uint16_t)(valH + 3);
    const uint8_t req[5] = {0x04, (uint8_t)(s & 0xFF), (uint8_t)(s >> 8), (uint8_t)(e & 0xFF), (uint8_t)(e >> 8)};
    uint8_t rsp[64];
    uint16_t rl = 0;
    uint16_t cccd = 0;
    if (halBtAtt(acl, req, sizeof(req), rsp, sizeof(rsp), &rl, kAttTimeoutMs) && rl >= 6 && rsp[0] == 0x05 && rsp[1] == 0x01) {
        for (uint16_t off = 2; off + 4 <= rl; off = (uint16_t)(off + 4)) {
            const uint16_t h = (uint16_t)(rsp[off] | (rsp[off + 1] << 8));
            const uint16_t u = (uint16_t)(rsp[off + 2] | (rsp[off + 3] << 8));
            if (u == 0x2803) {
                break;  // next characteristic - this one has no CCCD
            }
            if (u == 0x2902) {
                cccd = h;
                break;
            }
        }
    }
    if (cccd == 0) {
        return false;
    }
    const uint8_t on[2] = {0x01, 0x00};
    return attWrite(acl, cccd, on, sizeof(on));
}

// PACS and ASCS of the second earbud: primary services (Read By Group Type 0x2800), then the ASCS characteristic
// declarations (Read By Type 0x2803). Core Vol 3 Part G 4.4.1 / 4.6.1; parsing in le_audio_model (host test).
bool discoverMember(uint16_t acl, uint16_t& pacsStart, uint16_t& pacsEnd, AscsHandles& h)
{
    uint16_t ascsStart = 0, ascsEnd = 0;
    for (uint16_t start = 0x0001; start != 0 && (pacsStart == 0 || ascsStart == 0);) {
        const uint8_t req[7] = {0x10, (uint8_t)(start & 0xFF), (uint8_t)(start >> 8), 0xFF, 0xFF, 0x00, 0x28};
        uint8_t rsp[256];
        uint16_t rl = 0;
        if (!halBtAtt(acl, req, sizeof(req), rsp, sizeof(rsp), &rl, kAttTimeoutMs)) {
            return false;
        }
        uint16_t next = 0, s = 0, e = 0;
        if (pacsStart == 0 && findServiceInGroups(rsp, rl, 0x1850, s, e, next)) {
            pacsStart = s;
            pacsEnd   = e;
        }
        if (ascsStart == 0 && findServiceInGroups(rsp, rl, 0x184E, s, e, next)) {
            ascsStart = s;
            ascsEnd   = e;
        }
        start = next;
    }
    if (pacsStart == 0 || ascsStart == 0) {
        return false;
    }
    for (uint16_t start = ascsStart; start != 0 && start <= ascsEnd;) {
        const uint8_t req[7] = {0x08, (uint8_t)(start & 0xFF), (uint8_t)(start >> 8), (uint8_t)(ascsEnd & 0xFF),
                                (uint8_t)(ascsEnd >> 8), 0x03, 0x28};
        uint8_t rsp[256];
        uint16_t rl = 0;
        if (!halBtAtt(acl, req, sizeof(req), rsp, sizeof(rsp), &rl, kAttTimeoutMs)) {
            return false;
        }
        uint16_t next = 0;
        collectAscsHandles(rsp, rl, h, next);
        start = next;
    }
    return h.cp != 0 && h.sink[0] != 0;
}

// Write to the member's ASE Control Point and the response (CP notification) for that operation; code != 0 = error
// with a reason.
bool cpWrite(int mi, const uint8_t* v, size_t n, const char* what)
{
    uint32_t seq0;
    uint16_t acl, cpH;
    {
        std::lock_guard<std::mutex> l(g.lock);
        seq0 = g.m[mi].cpSeq;
        acl  = g.m[mi].acl;
        cpH  = g.m[mi].cpH;
    }
    if (!attWrite(acl, cpH, v, n)) {
        fail(fmt("ASE CP %s (earbud %d): write rejected", what, mi));
        return false;
    }
    if (!waitFor(1500, [&] { return g.m[mi].cpSeq != seq0 && g.m[mi].cp.op == v[0]; }, mi)) {
        fail(fmt("ASE CP %s (earbud %d): no response", what, mi));
        return false;
    }
    std::lock_guard<std::mutex> l(g.lock);
    const CpResult& cp = g.m[mi].cp;
    for (int i = 0; i < cp.n; i++) {
        if (cp.items[i].code != 0) {
            const auto it = cp.items[i];
            g.st.phase     = Phase::Failed;
            g.st.lastError = fmt("ASE CP %s (earbud %d): ASE %u - %s (reason 0x%02X)", what, mi, it.aseId,
                                 cpCodeText(it.code), it.reason);
            ESP_LOGW(kTag, "%s", g.st.lastError.c_str());
            return false;
        }
    }
    return true;
}

AseInfo& aseOf(int mi, bool sink)
{
    return sink ? g.m[mi].sinkAse : g.m[mi].srcAse;
}

bool waitAse(int mi, bool sink, AseState want, int ms)
{
    return waitFor(ms, [&] { return aseOf(mi, sink).state == want; }, mi);
}

uint32_t aseSeq(int mi, bool sink)
{
    std::lock_guard<std::mutex> l(g.lock);
    return sink ? g.m[mi].sinkSeq : g.m[mi].srcSeq;
}

// Like waitAse, but only after a new ASE notification (since seq0): after Release the server may keep Codec Configured
// from the previous session - a state without a new notification is the old QoS preference.
bool waitAseNew(int mi, bool sink, AseState want, uint32_t seq0, int ms)
{
    return waitFor(
        ms, [&] { return (sink ? g.m[mi].sinkSeq : g.m[mi].srcSeq) != seq0 && aseOf(mi, sink).state == want; }, mi);
}

bool readAse(int mi, uint16_t h, bool sink)
{
    uint16_t acl;
    {
        std::lock_guard<std::mutex> l(g.lock);
        acl = g.m[mi].acl;
    }
    const uint8_t rd[3] = {0x0A, (uint8_t)(h & 0xFF), (uint8_t)(h >> 8)};
    uint8_t r[128];
    uint16_t rl = 0;
    AseInfo a;
    if (!halBtAtt(acl, rd, sizeof(rd), r, sizeof(r), &rl, kAttTimeoutMs) || rl < 3 || r[0] != 0x0B ||
        !parseAse(r + 1, rl - 1u, a)) {
        return false;
    }
    std::lock_guard<std::mutex> l(g.lock);
    aseOf(mi, sink) = a;
    return true;
}

/* ------------------------------------ HCI ------------------------------------ */

bool hci(uint16_t op, const uint8_t* p, size_t n, const char* what, uint8_t* ret = nullptr, uint8_t cap = 0,
         uint8_t* retLen = nullptr)
{
    uint8_t status = 0xFF;
    if (!halBtHciCmd(op, p, (uint8_t)n, 800, &status, ret, cap, retLen) || status != 0) {
        fail(fmt("HCI %s (0x%04X): status 0x%02X", what, op, status));
        return false;
    }
    return true;
}

// Ends the earbud's ASEs (Disable, Receiver Stop Ready, Release) - each step on a best-effort basis.
void releaseMemberAses(int mi)
{
    uint16_t acl, cpH;
    uint8_t sinkId, srcId;
    {
        std::lock_guard<std::mutex> l(g.lock);
        const Member& m = g.m[mi];
        acl    = m.aclLost ? 0xFFFF : m.acl;
        cpH    = m.cpH;
        sinkId = m.sinkId;
        srcId  = m.srcId;
    }
    if (acl == 0xFFFF || cpH == 0) {
        return;
    }
    uint8_t b[8];
    if (srcId != 0 && cpSimple(b, sizeof(b), CpOp::Disable, srcId)) {
        attWrite(acl, cpH, b, 3);
        waitAse(mi, false, AseState::Disabling, 500);
        if (cpSimple(b, sizeof(b), CpOp::RecvStopReady, srcId)) {
            attWrite(acl, cpH, b, 3);
        }
    }
    if (sinkId != 0 && cpSimple(b, sizeof(b), CpOp::Disable, sinkId)) {
        attWrite(acl, cpH, b, 3);
    }
    for (uint8_t id : {sinkId, srcId}) {
        if (id != 0 && cpSimple(b, sizeof(b), CpOp::Release, id)) {
            attWrite(acl, cpH, b, 3);
        }
    }
}

// Removes the ISO paths and disconnects the member's CIS; releases its ISO credits.
void closeMemberCis(int mi)
{
    uint16_t cisH;
    bool pathIn, pathOut, up;
    {
        std::lock_guard<std::mutex> l(g.lock);
        cisH    = g.m[mi].cisHandle;
        pathIn  = g.m[mi].pathIn;
        pathOut = g.m[mi].pathOut;
        up      = g.m[mi].cisUp;
    }
    uint8_t status = 0;
    if (cisH != 0xFFFF) {
        // Only the directions that were set up: for a direction without a path the controller answers 0x0C and removes
        // none
        uint8_t rm[3];
        if (buildRemoveIsoPath(rm, sizeof(rm), cisH, pathIn, pathOut) != 0) {
            halBtHciCmd(0x206F, rm, sizeof(rm), 400, &status, nullptr, 0, nullptr);
        }
        if (up) {
            const uint8_t d[3] = {(uint8_t)(cisH & 0xFF), (uint8_t)(cisH >> 8), 0x13};
            halBtHciCmd(0x0406, d, sizeof(d), 400, &status, nullptr, 0, nullptr);
            waitFor(1000, [mi] { return !g.m[mi].cisUp; }, mi);
        }
    }
    std::lock_guard<std::mutex> l(g.lock);
    g.credits.dropHandle(cisH);
    g.m[mi].cisHandle = 0xFFFF;
    g.m[mi].cisUp     = false;
    g.m[mi].pathIn = g.m[mi].pathOut = false;
}

// Cleanup after the stream or a failed procedure - each step best-effort, without aborting on an error.
void teardown(bool cig)
{
    int nm;
    {
        std::lock_guard<std::mutex> l(g.lock);
        nm = g.nm;
    }
    for (int i = 0; i < nm; i++) {
        releaseMemberAses(i);
    }
    for (int i = 0; i < nm; i++) {
        closeMemberCis(i);
    }
    if (cig) {
        uint8_t status    = 0;
        const uint8_t c[1] = {kCigId};
        halBtHciCmd(0x2065, c, sizeof(c), 400, &status, nullptr, 0, nullptr);
    }
}

/* --------------------------------------- ISO TX --------------------------------------- */

// How long the stream thread waits for a credit before dropping the SDU: the return (Number Of Completed Packets) goes
// over the SDIO bus shared with the SD card and Wi-Fi and can be late by a few ms, while the C6 needs the SDU only 2+
// events ahead. Two earbuds x 3 ms + encoding fit in the 10 ms interval.
constexpr int kCreditWaitMs = 3;

enum class IsoSend { Sent, Down, NoCredit, Failed };

// One SDU per CIS: all fragments or nothing (no credits = SDU dropped, the sequence number advances anyway).
// waitCredit = false (txq queue): no waiting for a credit and no dropNoCredit counting - the SDU waits at the caller.
IsoSend isoSend(uint16_t cisH, uint16_t seq, const uint8_t* sdu, uint16_t len, bool waitCredit)
{
    uint16_t isoLen;
    uint16_t need;
    {
        std::unique_lock<std::mutex> l(g.lock);
        auto up = [cisH] {
            bool u = false;
            for (int i = 0; i < g.nm; i++) {
                u = u || (g.m[i].cisHandle == cisH && g.m[i].cisUp);
            }
            return u;
        };
        if (!up()) {
            return IsoSend::Down;  // CIS gone after the thread's read - take() would take credits of a freed handle
        }
        isoLen              = g.isoLen;
        const uint16_t room = isoLen > 4 ? (uint16_t)(isoLen - 4) : 0;
        need                = room == 0 ? 0xFFFF : (uint16_t)((len + room - 1) / room);
        if (waitCredit && need != 0xFFFF && g.credits.available() < need) {
            // NOCP wakes g.cv (halLeAudioOnHciEvent); the CIS may have gone away meanwhile - check again
            g.cv.wait_for(l, std::chrono::milliseconds(kCreditWaitMs),
                          [&] { return g.credits.available() >= need || !g.running.load(); });
            if (!up()) {
                return IsoSend::Down;
            }
        }
        if (need == 0xFFFF) {
            g.st.dropNoCredit++;
            return IsoSend::Failed;
        }
        if (g.credits.available() < need) {
            if (waitCredit) {
                g.st.dropNoCredit++;
            }
            return IsoSend::NoCredit;
        }
        const uint32_t nowUs = (uint32_t)esp_timer_get_time();
        for (uint16_t i = 0; i < need; i++) {
            g.credits.take(cisH, nowUs);
        }
        const int left = g.credits.available();
        g.st.creditsMin = g.st.creditsMin < 0 ? left : std::min(g.st.creditsMin, left);
    }
    uint8_t pkt[1 + 4 + 8 + 400];
    uint16_t off  = 0;
    uint16_t sent = 0;
    do {
        const size_t n = buildIsoPacket(pkt, sizeof(pkt), cisH, seq, false, 0, sdu, len, isoLen, off);
        if (n == 0 || !halBtSendRaw(pkt, (uint16_t)n)) {
            // The controller did not get these fragments, so it will not return NOCP for them - credits come back at
            // once (otherwise after a dozen SDIO errors every following SDU would be dropped for good).
            std::lock_guard<std::mutex> l(g.lock);
            g.credits.completed(cisH, (uint16_t)(need - sent));
            g.st.dropSendFail++;
            return IsoSend::Failed;
        }
        sent++;
    } while (off < len);
    return IsoSend::Sent;
}

// A credit without NOCP for longer than this is lost: the C6 returns each one within (1 + buffer + 1 + FT) events,
// about 100 ms, and the longest measured NOCP gap during an SD card stall is about 130 ms
constexpr uint32_t kCreditLostUs = 500000;

/* ------------------------------ credit-based SDU queue (txq) ------------------------------ */
// SDU deadline in the queue: the C6 maps an on-time SDU to the CIG event 1 + jitter-buffer intervals after it is
// accepted (cis_engine.c enqueue_sdu, buffer = C6 switch jitter_min 4), and accepts it at the latest two events before
// that - a margin of (buffer - 1) x 10 ms = 30 ms from the SDU time. Default 25 ms: 5 ms for the trip over SDIO
// (max_send_us up to 14 ms with an SD card is an extreme). The C6 would drop a late SDU anyway (sdu_late), and two
// lateness episodes within 64 SDUs shift the CIG mapping (1.4.77) - so it is better to drop it here.

struct TxqCount {
    uint32_t sent = 0, dropped = 0, deadline = 0;
};

// Sends from the queue of member i as far as credits allow; first drops SDUs past their deadline.
void txqPump(int i, int64_t nowUs, TxqCount& c)
{
    IsoTxQueue& q    = g.txq[i];
    const uint32_t x = q.expire(nowUs);
    c.deadline += x;
    c.dropped += x;
    while (const IsoTxQueue::Sdu* f = q.front()) {
        const IsoSend r = isoSend(g.txqH[i], f->seq, f->data, f->len, false);
        if (r == IsoSend::NoCredit) {
            return;
        }
        q.pop();
        if (r == IsoSend::Sent) {
            c.sent++;
        } else {
            c.dropped++;  // CIS disconnected or SDIO error - same as without the queue
        }
    }
}

void txqAddStats(const TxqCount& c, int depth)
{
    std::lock_guard<std::mutex> l(g.lock);
    g.st.txSdu += c.sent;
    g.st.txqSent += c.sent;
    g.st.txDropped += c.dropped;
    g.st.dropDeadline += c.deadline;
    g.st.txqMax = std::max(g.st.txqMax, depth);
}

enum class Offer { Sent, Queued, Dropped };

// New SDU of member i: behind the backlog from the queue (FIFO - number = interval); without a credit it goes to the
// queue with deadline dueUs.
Offer txqOffer(int i, uint16_t cisH, uint16_t seq, const uint8_t* sdu, uint16_t len, int64_t dueUs)
{
    IsoTxQueue& q = g.txq[i];
    TxqCount c;
    if (g.txqH[i] != cisH) {
        c.dropped += (uint32_t)q.size();  // a different CIS (member restart) - the old SDUs do not belong to it
        q.clear();
        g.txqH[i] = cisH;
    }
    txqPump(i, esp_timer_get_time(), c);
    Offer o = Offer::Queued;
    if (q.size() == 0) {
        const IsoSend r = isoSend(cisH, seq, sdu, len, false);
        o               = r == IsoSend::Sent ? Offer::Sent : r == IsoSend::NoCredit ? Offer::Queued : Offer::Dropped;
    }
    if (o == Offer::Queued) {
        uint32_t lost = 0;
        if (!q.push(seq, sdu, len, dueUs, lost)) {
            o = Offer::Dropped;
        }
        c.deadline += lost;  // queue full: the oldest one will not make it anyway
        c.dropped += lost;
    }
    txqAddStats(c, q.size());
    return o;
}

bool txqAnyQueued(int nm)
{
    for (int i = 0; i < nm; i++) {
        if (g.txq[i].size() > 0) {
            return true;
        }
    }
    return false;
}

// Waiting for the next interval with a non-empty queue: each NOCP (g.nocpSeq) wakes up and drains the queues at once,
// instead of only at the next SDU 10 ms later.
void txqWaitUntil(int nm, int64_t next)
{
    for (;;) {
        const int64_t now = esp_timer_get_time();
        if (next - now <= 1000) {
            return;
        }
        if (!txqAnyQueued(nm)) {
            vTaskDelay(pdMS_TO_TICKS((uint32_t)((next - now) / 1000)));
            return;
        }
        {
            std::unique_lock<std::mutex> l(g.lock);
            const uint32_t seen = g.nocpSeq;
            g.cv.wait_for(l, std::chrono::microseconds(next - now),
                          [&] { return g.nocpSeq != seen || !g.running.load(); });
        }
        if (!g.running.load()) {
            return;
        }
        TxqCount c;
        int depth = 0;
        for (int i = 0; i < nm; i++) {
            txqPump(i, esp_timer_get_time(), c);
            depth = std::max(depth, g.txq[i].size());
        }
        txqAddStats(c, depth);
    }
}

// txq switched off during the stream or end of the stream: the backlogged SDUs are lost.
void txqDropAll(int nm, bool count)
{
    TxqCount c;
    for (int i = 0; i < nm; i++) {
        c.deadline += (uint32_t)g.txq[i].size();
        g.txq[i].clear();
    }
    c.dropped = c.deadline;
    if (count && c.dropped > 0) {
        txqAddStats(c, 0);
    }
}

/* ------------------------------------ stream thread ------------------------------------ */

void streamThread()
{
    // Full power for the whole stream, also in silence (the player holds it only while playing): during screen standby
    // light sleep drives esp_timer from the slow RTC clock - SDUs ran ~3 % too fast relative to the CIS events (Tab5
    // 2026-11-02: 6-18 % late, the second earbud did not connect at 90 MHz); at full power 0.9991 and 0.01-0.55 %
    leport::Boost boost;
    int nm;
    bool stereo;
    CodecCfg sinkCfg[kMaxMembers];
    CodecCfg srcCfg;
    {
        std::lock_guard<std::mutex> l(g.lock);
        nm     = g.nm;
        stereo = g.stereo;
        g_lastNocpUs = 0;  // the NOCP gap counts from the first return of this stream
        for (int i = 0; i < nm; i++) {
            sinkCfg[i] = g.m[i].sinkCfg;
        }
        srcCfg = g.srcCfg;
    }
    const bool mic = srcCfg.rateHz != 0;
    // A separate encoder per earbud: the LC3 state depends on the previous frames of the channel
    void* encMem[kMaxMembers]     = {};
    lc3_encoder_t enc[kMaxMembers] = {};
    bool encOk                    = nm > 0;
    for (int i = 0; i < nm; i++) {
        encMem[i] = heap_caps_malloc(lc3_encoder_size(kFrameUs, kSinkRate), MALLOC_CAP_SPIRAM);
        // The encoder gets 48 kHz from the output path and resamples down itself when the earbuds want less (sr_pcm_hz)
        enc[i] = encMem[i] ? lc3_setup_encoder(kFrameUs, sinkCfg[i].rateHz, kSinkRate, encMem[i]) : nullptr;
        encOk  = encOk && enc[i] != nullptr;
    }
    void* decMem = mic ? heap_caps_malloc(lc3_decoder_size(kFrameUs, kMicRate), MALLOC_CAP_SPIRAM) : nullptr;
    lc3_decoder_t dec = decMem ? lc3_setup_decoder(kFrameUs, srcCfg.rateHz, kMicRate, decMem) : nullptr;
    const int decSamples = lc3_frame_samples(kFrameUs, kMicRate);
    if (!encOk || (mic && dec == nullptr) || decSamples <= 0) {
        fail("LC3: out of memory or bad encoder config");
        g.running = false;
    }
    int16_t* pcm   = static_cast<int16_t*>(heap_caps_malloc(kSinkFrame * 2 * sizeof(int16_t), MALLOC_CAP_SPIRAM));
    int16_t* mono  = static_cast<int16_t*>(heap_caps_malloc(kSinkFrame * sizeof(int16_t), MALLOC_CAP_SPIRAM));
    int16_t* micPcm = static_cast<int16_t*>(heap_caps_malloc(480 * sizeof(int16_t), MALLOC_CAP_SPIRAM));
    if (pcm == nullptr || mono == nullptr || micPcm == nullptr) {
        fail("LE Audio: no PSRAM for stream buffers");
        g.running = false;
    }
    uint8_t frame[400];
    RxSdu rx;
    uint16_t seq        = 0;
    int lastRxSeq       = -1;
    int64_t next        = esp_timer_get_time();
    uint32_t underruns  = 0;
    for (int i = 0; i < kMaxMembers; i++) {
        g.txq[i].clear();
        g.txqH[i] = 0xFFFF;
    }
    while (g.running.load()) {
        next += kFrameUs;
        const int64_t wait = next - esp_timer_get_time();
        int64_t stall      = 0;  // lateness before re-anchoring (late is ~0 after it)
        uint32_t skipped   = 0;
        if (wait > 1000) {
            if (g_fixTxq.load() && txqAnyQueued(nm)) {
                txqWaitUntil(nm, next);
            } else {
                vTaskDelay(pdMS_TO_TICKS((uint32_t)(wait / 1000)));
            }
        } else if (wait < -5 * kFrameUs) {
            // The thread stalled (e.g. flash) - no catching up with a burst of frames, but the SDU number advances by
            // the skipped intervals: number = interval (the C6 maps it to the CIG event). Without this the next SDU
            // was "late", the C6 re-anchored the queue and both earbuds lost another 30-50 ms.
            const int64_t now = esp_timer_get_time();
            stall   = now - next;
            skipped = (uint32_t)(stall / kFrameUs);
            seq     = (uint16_t)(seq + (uint16_t)skipped);
            next    = now;
        }
        const int64_t woke = esp_timer_get_time();
        const int64_t late = woke - next;  // positive = the thread woke after its deadline
        uint16_t cisH[kMaxMembers];
        bool on[kMaxMembers];
        int nOn = 0;
        {
            std::lock_guard<std::mutex> l(g.lock);
            bool anyUp = false;
            for (int i = 0; i < nm; i++) {
                cisH[i] = g.m[i].cisHandle;
                anyUp   = anyUp || g.m[i].cisUp;
                on[i]   = g.m[i].cisUp && !g.m[i].sinkOff;
                nOn += on[i] ? 1 : 0;
            }
            if (!anyUp) {
                break;
            }
            // Queue from the output path: too much (I2S clock faster than the CIS clock) - the frame is dropped
            while (g.down.fill / 2 > kDownMaxFr) {
                g.down.pop(nullptr, kSinkFrame * 2);
            }
            const size_t got = g.down.pop(pcm, kSinkFrame * 2);
            g.pcmCv.notify_all();
            if (got < kSinkFrame * 2) {
                std::memset(pcm + got, 0, (kSinkFrame * 2 - got) * sizeof(int16_t));
                underruns += got == 0 ? 0 : 1;  // empty queue = silence (nothing playing), partial = shortfall
                g.st.pcmShort += got == 0 ? 0 : 1;
            }
        }
        const float gain = g.volume.load() <= 0 ? 0.0f : std::pow(10.0f, (float)(g.volume.load() - 100) * 0.4f / 20.0f);
        const bool txq = g_fixTxq.load();
        if (!txq && txqAnyQueued(nm)) {
            txqDropAll(nm, true);  // txq switched off during the stream
        }
        const int64_t dueUs = next + (int64_t)g_fixTxqDeadlineMs.load() * 1000;  // deadline of this interval's SDU
        uint32_t sentN = 0, droppedN = 0, slowN = 0, encUs = 0, sendUs = 0;
        for (int k = 0; k < nm; k++) {
            // Alternating order: when credits run out it is not always the same earbud that loses
            const int i = (k + (seq & 1)) % nm;
            if (!on[i]) {
                if (g.txq[i].size() > 0) {
                    // the earbud disabled its Sink ASE / the CIS went down: backlogged SDUs go no further
                    // (txqWaitUntil pumps all)
                    TxqCount c;
                    c.deadline = c.dropped = (uint32_t)g.txq[i].size();
                    g.txq[i].clear();
                    txqAddStats(c, 0);
                }
                continue;
            }
            // Two earbuds L/R: each its own channel; one alone (or the other dropped out) - mono (L + R) / 2, nothing
            // of the mix is lost
            pickChannel(pcm, kSinkFrame, channelFor(sinkCfg[i].location, stereo ? nOn : 1), gain, mono);
            bool sent        = false;
            bool queued      = false;  // txq: the outcome (sent / past deadline) is counted when the queue is drained
            const int64_t t0 = esp_timer_get_time();
            int64_t t1       = t0;
            if (lc3_encode(enc[i], LC3_PCM_FORMAT_S16, mono, 1, sinkCfg[i].octets, frame) == 0) {
                t1 = esp_timer_get_time();
                if (txq) {
                    const Offer o = txqOffer(i, cisH[i], seq, frame, sinkCfg[i].octets, dueUs);
                    sent          = o == Offer::Sent;
                    queued        = o == Offer::Queued;
                } else {
                    sent = isoSend(cisH[i], seq, frame, sinkCfg[i].octets, true) == IsoSend::Sent;
                }
            }
            const int64_t t2 = esp_timer_get_time();
            if (!queued) {
                (sent ? sentN : droppedN)++;
            }
            slowN += t2 - t1 > 5000 ? 1 : 0;
            encUs  = std::max<uint32_t>(encUs, (uint32_t)(t1 - t0));
            sendUs = std::max<uint32_t>(sendUs, (uint32_t)(t2 - t1));
        }
        seq++;  // SDU number = interval number, also when the SDU was not sent (the controller sees which was skipped)
        {
            std::lock_guard<std::mutex> l(g.lock);
            // Lost NOCPs (aborted SDIO transfer): without this the credit never came back until the end of the stream -
            // Tab5 2026-11-02 CIS1 got 68 % of SDUs until the stream was restarted
            if (const uint16_t back = g.credits.sweep((uint32_t)esp_timer_get_time(), kCreditLostUs); back > 0) {
                g.st.creditsRecovered = g.credits.recovered();
                g.nocpSeq++;  // the txq queue waits for a credit change
                g.cv.notify_all();
            }
            g.st.txSdu += sentN;
            g.st.txDropped += droppedN;
            g.st.lateWakes += late > 5000 ? 1 : 0;
            g.st.slowSends += slowN;
            g.st.maxLateUs = std::max<uint32_t>(g.st.maxLateUs, (uint32_t)std::max<int64_t>(late, 0));
            g.st.maxEncUs  = std::max<uint32_t>(g.st.maxEncUs, encUs);
            g.st.maxSendUs = std::max<uint32_t>(g.st.maxSendUs, sendUs);
            g.st.maxStallUs = std::max<uint32_t>(g.st.maxStallUs, (uint32_t)std::max<int64_t>({stall, late, 0}));
            g.st.resyncs += skipped > 0 ? 1 : 0;
            g.st.skippedFrames += skipped;
        }
        // Microphone: SDU from the receive queue -> 16 kHz PCM; numbering gaps and lost SDUs -> liblc3 PLC
        while (mic) {
            {
                std::lock_guard<std::mutex> l(g.lock);
                if (g.rxCount == 0) {
                    break;
                }
                rx = g.rxq[g.rxHead];
                g.rxHead = (g.rxHead + 1) % kRxQueue;
                g.rxCount--;
            }
            int gap = lastRxSeq < 0 ? 0 : (int)(uint16_t)(rx.seq - (uint16_t)lastRxSeq) - 1;
            gap     = std::clamp(gap, 0, 5);
            lastRxSeq = rx.seq;
            for (int k = 0; k <= gap; k++) {
                const bool real = k == gap && rx.status == IsoStatus::Valid && rx.len == srcCfg.octets;
                lc3_decode(dec, real ? rx.data : nullptr, real ? rx.len : 0, LC3_PCM_FORMAT_S16, micPcm, 1);
                std::lock_guard<std::mutex> l(g.lock);
                if (!real) {
                    g.st.rxLost++;
                }
                if (g.mic.cap - g.mic.fill < (size_t)decSamples) {
                    g.mic.pop(nullptr, (size_t)decSamples);  // nobody is reading - the oldest samples are dropped
                }
                g.mic.push(micPcm, (size_t)decSamples);
            }
            g.micCv.notify_all();
        }
    }
    txqDropAll(nm, true);  // tx_sdu + tx_dropped = SDUs produced, also at the end of the stream
    if (underruns > 0) {
        ESP_LOGI(kTag, "LE Audio: PCM underruns from the output path: %lu", (unsigned long)underruns);
    }
    for (void* p : encMem) {
        heap_caps_free(p);
    }
    heap_caps_free(decMem);
    heap_caps_free(pcm);
    heap_caps_free(mono);
    heap_caps_free(micPcm);
    g.sinkReady = false;
    g.micReady  = false;
    g.micCv.notify_all();
    {
        std::lock_guard<std::mutex> l(g.lock);  // a writer between its check and its wait must not miss this
        g.pcmCv.notify_all();
    }
    // Ended without btLeAudioStop (no CIS is playing any more): running must not stay set - an ASE notification at the
    // next start would look like the earbud ending the stream (sinkOff)
    g.running    = false;
    g.threadDone = true;
    g.cv.notify_all();
}

/* ------------------------------------ BAP procedure ------------------------------------ */

uint32_t pickLocation(uint16_t acl, uint16_t s, uint16_t e, uint16_t uuid)
{
    uint8_t v[8];
    size_t n = 0;
    if (!attReadUuid(acl, s, e, uuid, v, sizeof(v), n) || n < 4) {
        return 0;
    }
    const uint32_t loc = (uint32_t)v[0] | ((uint32_t)v[1] << 8) | ((uint32_t)v[2] << 16) | ((uint32_t)v[3] << 24);
    if (loc & 0x1) {
        return 0x1;  // Front Left
    }
    if (loc & 0x2) {
        return 0x2;  // Front Right
    }
    return loc & (~loc + 1);  // lowest set bit (0 = no location)
}

// The second earbud failed during the procedure (no services, no encryption, ASE refusal, CIS): its ASE and CIS are
// released, only the primary (mono) continues - as before the second joined. The second's error does not end the start.
void dropSecond(const std::string& why)
{
    ESP_LOGW(kTag, "LE Audio: second earbud skipped - %s; one plays (mono)", why.c_str());
    releaseMemberAses(1);
    closeMemberCis(1);
    std::lock_guard<std::mutex> l(g.lock);
    g.nm   = 1;
    g.m[1] = Member{};
    g.attrCp[1] = g.attrSink[1] = g.attrSrc[1] = 0;
    if (g.st.phase == Phase::Failed) {
        g.st.phase = Phase::Configuring;  // fail() from the second's step (cpWrite, lost ACL)
    }
    g.st.lastError.clear();
}

// forcePhy: 0 = PHY from the earbuds' preferences, 1/2 = forced (measurement session, hal_le_audio_lab_phy).
bool runStart(bool wantMic, uint8_t forcePhy)
{
    // The second earbud of the set is joining right now (hal_bt joinSetMember: RSI scan, encryption or pairing) - we
    // wait so that stereo is set up at once instead of mono plus a restart
    for (int i = 0; i < 300 && halBtLeAudioLink().setJoining; i++) {
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    const HalBtLeAudioLink link = halBtLeAudioLink();
    if (link.acl == 0xFFFF) {
        fail("no connected earbuds");
        return false;
    }
    if (!link.leAudioOn) {
        fail("LE Audio disabled (bt.leaudio setting) - experimental, enable manually and restart");
        return false;
    }
    if (!link.cisCentral) {
        fail("controller has no CIS Central (LE feature bit 28) - needs C6 CIS image");
        return false;
    }
    if (!link.encrypted) {
        fail("link not encrypted - LE Audio needs bonded earbuds");
        return false;
    }
    if (link.ascsCp == 0 || link.sinkAse[0] == 0 || link.pacsStart == 0) {
        fail("earbuds lack PACS/ASCS (Sink ASE, ASE Control Point)");
        return false;
    }
    const bool mic = wantMic && link.srcAse[0] != 0;
    {
        std::lock_guard<std::mutex> l(g.lock);
        // A stream that died on its own (ACLs of both earbuds lost) did not go through btLeAudioStop and left running
        // set. Then a "QoS Configured" notification during this configuration looked like the earbud ending the
        // stream (sinkOff) and the new stream played silence (Tab5 2026-11-01: after taking them out of the case
        // tx_sdu stayed 0 until a restart). The previous thread has already finished (btLeAudioStart checks
        // threadDone).
        g.running = false;
        for (Member& m : g.m) {
            m = Member{};
        }
        Member& m0   = g.m[0];
        m0.acl       = link.acl;
        m0.pacsStart = link.pacsStart;
        m0.pacsEnd   = link.pacsEnd;
        m0.cpH       = link.ascsCp;
        m0.sinkH     = link.sinkAse[0];
        m0.srcH      = mic ? link.srcAse[0] : 0;
        g.m[1].acl   = link.acl2;
        g.nm         = link.acl2 != 0xFFFF ? 2 : 1;
        g.stereo     = false;
        g.st        = lehost::LeAudioStatus_t{};
        g.st.phase  = Phase::Configuring;
    }
    g.attrCp[0]   = link.ascsCp;
    g.attrSink[0] = link.sinkAse[0];
    g.attrSrc[0]  = mic ? link.srcAse[0] : 0;
    g.attrCp[1] = g.attrSink[1] = g.attrSrc[1] = 0;

    // 0. Second set member: its PACS and ASCS (hal_bt only knows the handles of the primary connection)
    if (link.acl2 != 0xFFFF) {
        AscsHandles h2;
        uint16_t ps = 0, pe = 0;
        if (!halBtAcl(link.acl2).encrypted) {
            dropSecond("link not encrypted");
        } else if (!discoverMember(link.acl2, ps, pe, h2)) {
            dropSecond("no PACS/ASCS (Sink ASE, ASE Control Point)");
        } else {
            {
                std::lock_guard<std::mutex> l(g.lock);
                g.m[1].pacsStart = ps;
                g.m[1].pacsEnd   = pe;
                g.m[1].cpH       = h2.cp;
                g.m[1].sinkH     = h2.sink[0];
            }
            g.attrCp[1]   = h2.cp;
            g.attrSink[1] = h2.sink[0];
            ESP_LOGI(kTag, "LE Audio: second earbud 0x%03X - PACS 0x%04X-0x%04X, ASE CP 0x%04X, Sink ASE 0x%04X",
                     link.acl2, ps, pe, h2.cp, h2.sink[0]);
        }
    }
    int nm;
    {
        std::lock_guard<std::mutex> l(g.lock);
        nm = g.nm;
    }
    // Error of earbud i: the second (1) drops out and we play on with the primary (true - break the loop over members),
    // the primary (0) - end of the start with cleanup: 0 nothing, 1 ASE, 2 ASE + CIS + CIG (false - leave runStart).
    auto memberFailed = [&](int i, const std::string& why, int cleanup) {
        if (i == 1) {
            dropSecond(why);
            nm = 1;
            return true;
        }
        fail(why);
        if (cleanup > 0) {
            teardown(cleanup > 1);
        }
        return false;
    };

    // 1. Controller ISO buffers. First LE Read Buffer Size v1 (0x2002): the CIS emulation on the C6 copies the ACL
    //    fields of version 2 from the overheard v1 response - without it v2 would have empty ACL fields. From v2 we
    //    take only the ISO fields.
    uint8_t ret[16];
    uint8_t rl = 0;
    IsoBuffers ib;
    if (!hci(0x2002, nullptr, 0, "LE Read Buffer Size", ret, sizeof(ret), &rl) ||
        !hci(0x2060, nullptr, 0, "LE Read Buffer Size v2", ret, sizeof(ret), &rl)) {
        return false;
    }
    if (!parseReadBufferSizeV2(ret, rl, ib) || ib.isoLen < 8 || ib.isoNum == 0) {
        fail("controller gave no ISO buffers (LE Read Buffer Size v2)");
        return false;
    }
    {
        std::lock_guard<std::mutex> l(g.lock);
        g.isoLen = ib.isoLen;
        g.credits.reset(ib.isoNum);
        g.st.creditsTotal = ib.isoNum;
    }

    // 2. PACS: capabilities and locations of each earbud
    uint8_t pacv[240];
    size_t pn = 0;
    CodecCfg sinkCfg[kMaxMembers], srcCfg;
    for (int i = 0; i < nm; i++) {
        Member m;
        {
            std::lock_guard<std::mutex> l(g.lock);
            m = g.m[i];
        }
        PacSet sinkPac;
        if (!attReadUuid(m.acl, m.pacsStart, m.pacsEnd, 0x2BC9, pacv, sizeof(pacv), pn) ||
            !parsePac(pacv, pn, sinkPac)) {
            if (!memberFailed(i, fmt("PACS: cannot read Sink PAC (earbud %d)", i), 0)) {
                return false;
            }
            break;
        }
        const uint32_t sinkLoc = pickLocation(m.acl, m.pacsStart, m.pacsEnd, 0x2BCA);
        if (!chooseSinkConfig(sinkPac, sinkLoc, sinkCfg[i])) {
            if (!memberFailed(
                    i, fmt("PACS: earbud %d supports no LC3 10 ms BAP config (48/32/24/16 kHz)", i),
                    0)) {
                return false;
            }
            break;
        }
    }
    if (mic) {
        PacSet srcPac;
        const Member& m0      = g.m[0];  // fields constant since the start - only this thread changes them
        const uint32_t srcLoc = pickLocation(m0.acl, m0.pacsStart, m0.pacsEnd, 0x2BCC);
        if (!attReadUuid(m0.acl, m0.pacsStart, m0.pacsEnd, 0x2BCB, pacv, sizeof(pacv), pn) ||
            !parsePac(pacv, pn, srcPac) || !chooseSourceConfig(srcPac, srcLoc, srcCfg) || srcCfg.rateHz != kMicRate) {
            // The liblc3 decoder can only resample upwards; the Tab5 speech path is 16 kHz (16_2 is mandatory in BAP)
            fail("PACS: earbud mic without LC3 16 kHz 10 ms");
            return false;
        }
    }
    // Stereo only with different left / right locations; otherwise (e.g. both 0x3) each plays mono
    const bool stereo = nm == 2 && sinkCfg[0].location != sinkCfg[1].location &&
                        (sinkCfg[0].location == 0x1 || sinkCfg[0].location == 0x2) &&
                        (sinkCfg[1].location == 0x1 || sinkCfg[1].location == 0x2);
    if (nm == 2 && !stereo) {
        ESP_LOGW(kTag, "LE Audio: earbud locations 0x%lX / 0x%lX - no L/R split, both mono",
                 (unsigned long)sinkCfg[0].location, (unsigned long)sinkCfg[1].location);
    }

    // 3. ASE and CP notifications, ASE state, Release after the previous session
    uint8_t b[64];
    for (int i = 0; i < nm; i++) {
        Member m;
        {
            std::lock_guard<std::mutex> l(g.lock);
            m = g.m[i];
        }
        const bool withSrc = m.srcH != 0;
        if (!subscribe(m.acl, m.cpH) || !subscribe(m.acl, m.sinkH) || (withSrc && !subscribe(m.acl, m.srcH))) {
            if (!memberFailed(i, fmt("ASCS: cannot enable notifications (CCCD, earbud %d)", i), 0)) {
                return false;
            }
            break;
        }
        if (!readAse(i, m.sinkH, true) || (withSrc && !readAse(i, m.srcH, false))) {
            if (!memberFailed(i, fmt("ASCS: cannot read ASE state (earbud %d)", i), 0)) {
                return false;
            }
            break;
        }
        uint8_t sinkId, srcId = 0;
        AseState s0, s1 = AseState::Idle;
        {
            std::lock_guard<std::mutex> l(g.lock);
            sinkId = g.m[i].sinkId = g.m[i].sinkAse.id;
            s0     = g.m[i].sinkAse.state;
            if (withSrc) {
                srcId = g.m[i].srcId = g.m[i].srcAse.id;
                s1    = g.m[i].srcAse.state;
            }
        }
        // ASE left over from the previous session (another client, aborted stream): Release first
        const struct {
            uint8_t id;
            AseState st;
            bool sink;
        } ases[2] = {{sinkId, s0, true}, {srcId, s1, false}};
        bool released = true;
        for (const auto& a : ases) {
            if (a.id != 0 && a.st != AseState::Idle && a.st != AseState::CodecConfigured &&
                a.st != AseState::QosConfigured) {
                // Releasing (e.g. right after the stop of the previous run) finishes by itself - Release would be
                // rejected by CP (0x04)
                if (a.st != AseState::Releasing) {
                    cpSimple(b, sizeof(b), CpOp::Release, a.id);
                    if (!cpWrite(i, b, 3, "Release")) {
                        released = false;
                        break;
                    }
                }
                const bool sink = a.sink;
                waitFor(
                    2000,
                    [&] {
                        const AseState s = aseOf(i, sink).state;
                        return s == AseState::Idle || s == AseState::CodecConfigured;
                    },
                    i);
            }
        }
        if (!released) {
            if (!memberFailed(i, lastErrorOr("ASCS: Release rejected"), 0)) {
                return false;
            }
            break;
        }

        // ACL interval a multiple of 10 ms (CIS emulation in the C6 = pseudo-connection with the ISO interval;
        // otherwise ACL events every few intervals collide with the CIS). We connect with min = max =
        // kAclIntervalUnits, but the earbuds may have changed it.
        if (!aclIntervalCisFriendly(halBtAcl(m.acl).connItvl)) {
            const uint16_t u    = kAclIntervalUnits;
            const uint8_t cu[14] = {(uint8_t)(m.acl & 0xFF), (uint8_t)(m.acl >> 8), (uint8_t)u, (uint8_t)(u >> 8),
                                    (uint8_t)u, (uint8_t)(u >> 8), 0, 0, 0x90, 0x01, 0, 0, 0, 0};
            uint8_t st           = 0xFF;
            halBtHciCmd(0x2013, cu, sizeof(cu), 500, &st, nullptr, 0, nullptr);
            for (int k = 0; k < 60 && !aclIntervalCisFriendly(halBtAcl(m.acl).connItvl); k++) {
                vTaskDelay(pdMS_TO_TICKS(50));
            }
            const uint16_t now = halBtAcl(m.acl).connItvl;
            if (!aclIntervalCisFriendly(now)) {
                if (!memberFailed(i, fmt("ACL interval %u x 1.25 ms (earbud %d) is not a multiple of 10 ms "
                                         "(LE Connection Update: 0x%02X)", (unsigned)now, i, st), 0)) {
                    return false;
                }
                break;
            }
        }
    }

    // 4. Config Codec -> Codec Configured (the server's QoS preferences)
    const uint8_t targetPhy = forcePhy == 1 ? 0x01 : 0x02;  // Target_PHY: a hint for the server's QoS preferences
    for (int i = 0; i < nm; i++) {
        uint32_t seq0 = aseSeq(i, true);
        size_t n      = cpConfigCodec(b, sizeof(b), g.m[i].sinkId, sinkCfg[i], 0x02, targetPhy);
        if (n == 0 || !cpWrite(i, b, n, "Config Codec (Sink)") ||
            !waitAseNew(i, true, AseState::CodecConfigured, seq0, 2000)) {
            if (!memberFailed(i, lastErrorOr(fmt("ASCS: Sink ASE of earbud %d did not reach Codec Configured", i)),
                              1)) {
                return false;
            }
            break;
        }
    }
    if (mic) {
        const uint32_t seq0 = aseSeq(0, false);
        const size_t n      = cpConfigCodec(b, sizeof(b), g.m[0].srcId, srcCfg, 0x02, targetPhy);
        if (n == 0 || !cpWrite(0, b, n, "Config Codec (Source)") ||
            !waitAseNew(0, false, AseState::CodecConfigured, seq0, 2000)) {
            fail("ASCS: Source ASE did not reach Codec Configured");
            teardown(false);
            return false;
        }
    }

    // 5. Config QoS: CIS_ID = earbud index; the microphone on the same (bidirectional) CIS as Sink m[0]
    QosCfg sinkQos[kMaxMembers], srcQos;
    for (int i = 0; i < nm; i++) {
        AsePref pref;
        {
            std::lock_guard<std::mutex> l(g.lock);
            pref = g.m[i].sinkAse.pref;
        }
        if (!chooseQos(pref, sinkCfg[i], kCigId, (uint8_t)i, sinkQos[i])) {
            if (!memberFailed(
                    i, fmt("ASCS: earbud %d QoS preferences out of range (framed or no delay range)", i),
                    1)) {
                return false;
            }
            break;
        }
    }
    if (mic) {
        AsePref pref;
        {
            std::lock_guard<std::mutex> l(g.lock);
            pref = g.m[0].srcAse.pref;
        }
        if (!chooseQos(pref, srcCfg, kCigId, 0, srcQos)) {
            fail("ASCS: mic QoS preferences out of range (framed or no delay range)");
            teardown(false);
            return false;
        }
        unifyCisPhy(sinkQos[0], srcQos);  // one CIS for both directions, and the C6 has one PHY per link
    }
    if (forcePhy != 0) {
        // The same PHY in Config QoS (earbuds) and in LE Set CIG Parameters (controller) - the silence hypothesis from
        // --sweep.
        for (int i = 0; i < nm; i++) {
            sinkQos[i].phy = forcePhy;
        }
        srcQos.phy = forcePhy;
    }
    for (int i = 0; i < nm; i++) {
        const size_t n = cpConfigQos(b, sizeof(b), g.m[i].sinkId, sinkQos[i]);
        if (n == 0 || !cpWrite(i, b, n, "Config QoS (Sink)") || !waitAse(i, true, AseState::QosConfigured, 2000)) {
            if (!memberFailed(i, lastErrorOr(fmt("ASCS: Sink ASE of earbud %d did not reach QoS Configured", i)), 1)) {
                return false;
            }
            break;
        }
    }
    if (mic) {
        const size_t n = cpConfigQos(b, sizeof(b), g.m[0].srcId, srcQos);
        if (n == 0 || !cpWrite(0, b, n, "Config QoS (Source)") || !waitAse(0, false, AseState::QosConfigured, 2000)) {
            fail("ASCS: Source ASE did not reach QoS Configured");
            teardown(false);
            return false;
        }
    }

    // 6. Enable (context: Media, with a microphone - Conversational), within the Available Audio Contexts (PACS 0x2BCD:
    // Sink u16, Source u16) of each earbud - earbuds reject a context outside them
    const uint16_t ctx = mic ? 0x0002 : 0x0004;
    for (int i = 0; i < nm; i++) {
        const Member& m    = g.m[i];  // acl and PACS constant since the start
        uint16_t availSink = 0, availSrc = 0;
        uint8_t av[4];
        size_t an = 0;
        if (attReadUuid(m.acl, m.pacsStart, m.pacsEnd, 0x2BCD, av, sizeof(av), an) && an >= 4) {
            availSink = (uint16_t)(av[0] | (av[1] << 8));
            availSrc  = (uint16_t)(av[2] | (av[3] << 8));
        }
        const uint16_t sinkCtx = pickStreamingContext(ctx, availSink), srcCtx = pickStreamingContext(ctx, availSrc);
        ESP_LOGI(kTag, "LE Audio: earbud %d - available contexts Sink 0x%04X Source 0x%04X -> Enable 0x%04X / 0x%04X",
                 i, availSink, availSrc, sinkCtx, srcCtx);
        size_t n = cpEnable(b, sizeof(b), m.sinkId, sinkCtx, mic ? kGtbsCcid : kGmcsCcid);
        if (!cpWrite(i, b, n, "Enable (Sink)") || !waitAse(i, true, AseState::Enabling, 2000)) {
            if (!memberFailed(i, lastErrorOr(fmt("ASCS: Sink ASE of earbud %d did not reach Enabling", i)), 1)) {
                return false;
            }
            break;
        }
        if (i == 0 && mic) {
            n = cpEnable(b, sizeof(b), m.srcId, srcCtx, kGtbsCcid);
            if (!cpWrite(0, b, n, "Enable (Source)") || !waitAse(0, false, AseState::Enabling, 2000)) {
                fail("ASCS: Source ASE did not reach Enabling");
                teardown(false);
                return false;
            }
        }
    }

    // 7. LE Data Length >= 36 B on each ACL before the CIS (LL_CIS_REQ cannot be fragmented)
    for (int i = 0; i < nm; i++) {
        const uint16_t acl = g.m[i].acl;
        auto dleOk         = [acl] { return halBtAcl(acl).dleTxOctets >= 36; };
        for (int attempt = 0; attempt < 2 && !dleOk(); attempt++) {
            if (attempt > 0 || halBtAcl(acl).dleTxOctets == 0) {
                halBtRequestDataLength(acl);
            }
            for (int k = 0; k < 30 && !dleOk(); k++) {
                vTaskDelay(pdMS_TO_TICKS(50));
            }
        }
        if (!dleOk()) {
            if (!memberFailed(i, fmt("earbud %d ACL without LE Data Length >= 36 B (is %u) - CIS impossible", i,
                                     halBtAcl(acl).dleTxOctets), 1)) {
                return false;
            }
            break;
        }
    }

    // 8. CIG (a CIS per earbud) + CIS. CIG transport latency = the smallest of the preferences - it must suit each.
    CigParams cig;
    cig.cigId         = kCigId;
    cig.sduIntervalUs = kFrameUs;
    cig.latencyCtoPMs = sinkQos[0].maxLatencyMs;
    for (int i = 1; i < nm; i++) {
        cig.latencyCtoPMs = std::min(cig.latencyCtoPMs, sinkQos[i].maxLatencyMs);
    }
    cig.latencyPtoCMs = mic ? srcQos.maxLatencyMs : cig.latencyCtoPMs;
    cig.nCis          = nm;
    for (int i = 0; i < nm; i++) {
        const bool withSrc = i == 0 && mic;
        cig.cis[i]         = CisParams{(uint8_t)i,
                                       sinkQos[i].maxSdu,
                                       (uint16_t)(withSrc ? srcQos.maxSdu : 0),
                                       sinkQos[i].phy,
                                       withSrc ? srcQos.phy : sinkQos[i].phy,
                                       sinkQos[i].rtn,
                                       withSrc ? srcQos.rtn : sinkQos[i].rtn};
    }
    size_t n = buildSetCigParams(b, sizeof(b), cig);
    if (n == 0 || !hci(0x2062, b, n, "LE Set CIG Parameters", ret, sizeof(ret), &rl)) {
        teardown(false);
        return false;
    }
    if (rl < 2u + 2u * (unsigned)nm || ret[0] != kCigId || ret[1] != nm) {
        fail("LE Set CIG Parameters: bad controller response");
        teardown(true);
        return false;
    }
    uint16_t cisHs[kMaxMembers], aclHs[kMaxMembers];
    uint32_t cisSeq0[kMaxMembers];
    {
        std::lock_guard<std::mutex> l(g.lock);
        for (int i = 0; i < nm; i++) {
            cisHs[i]           = (uint16_t)((ret[2 + 2 * i] | (ret[3 + 2 * i] << 8)) & 0x0FFF);
            aclHs[i]           = g.m[i].acl;
            g.m[i].cisHandle   = cisHs[i];
            g.m[i].sinkCfg     = sinkCfg[i];
            g.m[i].sinkQos     = sinkQos[i];
            cisSeq0[i]         = g.m[i].cisSeq;
        }
    }
    n = buildCreateCis(b, sizeof(b), cisHs, aclHs, nm);
    if (!hci(0x2064, b, n, "LE Create CIS") || !waitFor(42000, [&] {
            for (int i = 0; i < nm; i++) {
                if (g.m[i].cisSeq == cisSeq0[i]) {
                    return false;
                }
            }
            return true;
        })) {
        // The controller ends the attempt itself: CIS Established with status 0x22 after 40 s without an LL response;
        // this is only a margin
        if (g.st.lastError.empty()) {
            fail("CIS: no LE CIS Established (42 s)");
        }
        teardown(true);
        return false;
    }
    // 0x1A earbuds without CIS Peripheral, 0x3E not established within 6 events, 0x22 no LL response,
    // 0x0D no LE Data Length >= 36 B
    auto cisErr = [](uint8_t s) {
        return fmt("status 0x%02X%s", s,
                   s == 0x1A   ? " (no CIS on earbuds)"
                   : s == 0x3E ? " (not set up)"
                   : s == 0x22 ? " (no earbud response)"
                   : s == 0x0D ? " (LE Data Length < 36 B)"
                               : "");
    };
    uint8_t cisSt[kMaxMembers] = {};
    {
        std::lock_guard<std::mutex> l(g.lock);
        for (int i = 0; i < nm; i++) {
            cisSt[i] = g.m[i].cis.status;
        }
    }
    if (cisSt[0] != 0) {
        fail("CIS not established: " + cisErr(cisSt[0]));
        teardown(true);
        return false;
    }
    if (nm == 2 && cisSt[1] != 0) {
        // The primary plays, the second does not (the second's CIS stays unused in the CIG)
        memberFailed(1, "CIS not established: " + cisErr(cisSt[1]), 0);
    }
    for (int i = 0; i < nm; i++) {
        CisEstablished c;
        {
            std::lock_guard<std::mutex> l(g.lock);
            c = g.m[i].cis;
        }
        ESP_LOGI(kTag, "CIS 0x%03X (earbud %d) established: ISO interval %u x 1.25 ms, NSE %u, BN %u/%u, FT %u/%u, "
                 "PDU %u/%u", c.handle, i, c.isoInterval, c.nse, c.bnCtoP, c.bnPtoC, c.ftCtoP, c.ftPtoC, c.maxPduCtoP,
                 c.maxPduPtoC);
    }

    // 9. ISO paths over HCI: input on every CIS, output (microphone) only m[0]
    for (int i = 0; i < nm; i++) {
        n = buildSetupIsoPath(b, sizeof(b), cisHs[i], false);
        if (!hci(0x206E, b, n, "LE Setup ISO Data Path (input)")) {
            if (!memberFailed(i, lastErrorOr("LE Setup ISO Data Path (input)"), 2)) {
                return false;
            }
            break;
        }
        std::lock_guard<std::mutex> l(g.lock);
        g.m[i].pathIn = true;
    }
    if (mic) {
        n = buildSetupIsoPath(b, sizeof(b), cisHs[0], true);
        if (!hci(0x206E, b, n, "LE Setup ISO Data Path (output)")) {
            teardown(true);
            return false;
        }
        {
            std::lock_guard<std::mutex> l(g.lock);
            g.m[0].pathOut = true;
        }
        // The client is the sink of the Source ASE: ready to receive -> Streaming
        cpSimple(b, sizeof(b), CpOp::RecvStartReady, g.m[0].srcId);
        if (!cpWrite(0, b, 3, "Receiver Start Ready") || !waitAse(0, false, AseState::Streaming, 2000)) {
            fail("ASCS: Source ASE did not reach Streaming");
            teardown(true);
            return false;
        }
    }
    for (int i = 0; i < nm; i++) {
        if (!waitAse(i, true, AseState::Streaming, 3000)) {
            if (!memberFailed(i, lastErrorOr(fmt("ASCS: Sink ASE of earbud %d did not reach Streaming", i)), 2)) {
                return false;
            }
            break;
        }
    }

    // 10. Stream
    const bool stereoNow = stereo && nm == 2;
    {
        std::lock_guard<std::mutex> l(g.lock);
        if (!g.down.alloc(kDownRingFr * 2) || (mic && !g.mic.alloc(kMicRingSmp))) {
            g.st.phase     = Phase::Failed;
            g.st.lastError = "no PSRAM for audio queues";
        }
        g.rxHead = g.rxCount = 0;
        g.stereo          = stereoNow;
        g.srcCfg          = mic ? srcCfg : CodecCfg{};
        g.st.sinkRate     = sinkCfg[0].rateHz;
        g.st.sinkOctets   = sinkCfg[0].octets;
        g.st.srcRate      = mic ? srcCfg.rateHz : 0;
        g.st.srcOctets    = mic ? srcCfg.octets : 0;
        g.st.presentationDelayUs = sinkQos[0].pdUs;
        g.st.mic          = mic;
        g.st.members      = nm;
        g.st.stereo       = stereoNow;
    }
    if (g.st.phase == Phase::Failed) {
        teardown(true);
        return false;
    }
    g_reasm.reset();
    g.running    = true;
    g.threadDone = false;
    g.sinkReady  = true;
    g.micReady   = mic;
    {
        std::lock_guard<std::mutex> l(g.lock);
        g.st.phase = Phase::Streaming;
    }
    // Priority above lwIP (18), like the audio output task: the 10 ms encoder cannot wait for the network. Core 1: on
    // core 0 (taskLVGL pinned) a screen redraw (~110 ms, LVGL at priority 23 inherited through a mutex) held the
    // thread up - while scrolling 58 % of CIS events had no SDU (Tab5 2026-11-01, diag redraw 20).
    leport::spawn("le-audio", 12 * 1024, streamThread, 19, 1);
    ESP_LOGI(kTag, "LE Audio: LC3 stream %d Hz / %u B, %s (locations 0x%lX%s, PD %lu us)%s", sinkCfg[0].rateHz,
             sinkCfg[0].octets, stereoNow ? "stereo L/R" : nm == 2 ? "two earbuds mono" : "mono",
             (unsigned long)sinkCfg[0].location, nm == 2 ? fmt(" / 0x%lX", (unsigned long)sinkCfg[1].location).c_str() : "",
             (unsigned long)sinkQos[0].pdUs,
             mic ? fmt(", microphone %d Hz / %u B", srcCfg.rateHz, srcCfg.octets).c_str() : "");
    return true;
}

// Index of the member with this ACL connection (under g.lock); -1 = foreign connection.
int memberByAclLocked(uint16_t acl)
{
    for (int i = 0; i < g.nm; i++) {
        if (g.m[i].acl == acl) {
            return i;
        }
    }
    return -1;
}

}  // namespace

/* ------------------------------ callbacks from hal_bt ------------------------------ */

LeAudioFix halLeAudioFixGet()
{
    LeAudioFix f;
    f.txq           = g_fixTxq.load();
    f.txqDeadlineMs = g_fixTxqDeadlineMs.load();
    return f;
}

void halLeAudioFixSet(const LeAudioFix& f)
{
    // Deadline 5..60 ms: shorter than the trip over SDIO makes no sense, longer than the C6 jitter buffer (40 ms) plus
    // margin - the SDU would be late anyway
    g_fixTxqDeadlineMs = std::clamp(f.txqDeadlineMs, 5, 60);
    g_fixTxq           = f.txq;
    ESP_LOGW(kTag, "LE Audio: credit-based SDU queue %d (deadline %d ms)", f.txq ? 1 : 0, g_fixTxqDeadlineMs.load());
}

uint32_t halLeAudioNocpGapTake()
{
    std::lock_guard<std::mutex> l(g.lock);
    const uint32_t v = g_nocpGapMaxUs;
    g_nocpGapMaxUs   = 0;
    return v;
}

void halLeAudioResetExtremes()
{
    std::lock_guard<std::mutex> l(g.lock);
    g.st.creditsMin = -1;
    g.st.txqMax     = 0;
    g.st.maxLateUs = g.st.maxStallUs = g.st.maxEncUs = g.st.maxSendUs = 0;
    g_nocpGapMaxUs  = 0;
}

bool halLeAudioOwnsAttr(uint16_t h)
{
    if (h == 0) {
        return false;
    }
    for (int i = 0; i < kMaxMembers; i++) {
        if (h == g.attrCp[i].load() || h == g.attrSink[i].load() || h == g.attrSrc[i].load()) {
            return true;
        }
    }
    return false;
}

void halLeAudioOnAttValue(uint16_t conn, uint16_t attr, const uint8_t* val, uint16_t len)
{
    std::lock_guard<std::mutex> l(g.lock);
    const int mi = memberByAclLocked(conn);
    if (mi < 0) {
        return;
    }
    Member& m = g.m[mi];
    if (attr == m.cpH) {
        CpResult r;
        if (parseCpNotify(val, len, r)) {
            m.cp = r;
            m.cpSeq++;
        }
    } else if (attr == m.sinkH || (m.srcH != 0 && attr == m.srcH)) {
        const bool sink = attr == m.sinkH;
        AseInfo a;
        if (!parseAse(val, len, a)) {
            ESP_LOGW(kTag, "ASE 0x%04X (earbud %d): bad value (%u B)", attr, mi, (unsigned)len);
            return;
        }
        AseInfo& cur = sink ? m.sinkAse : m.srcAse;
        if (cur.state != AseState::Invalid && !aseTransitionAllowed(sink, cur.state, a.state)) {
            m.protoError   = fmt("ASE %u (earbud %d): illegal transition %s -> %s", a.id, mi,
                                 aseStateName(cur.state), aseStateName(a.state));
            g.st.lastError = m.protoError;
        }
        // The server ends the stream itself (Releasing / Disabling / QoS during playback) - the encoder stops sending
        if (g.running.load() && (a.state == AseState::Releasing || a.state == AseState::QosConfigured ||
                                 a.state == AseState::Disabling)) {
            if (sink) {
                m.sinkOff     = true;
                bool anyOn    = false;
                for (int i = 0; i < g.nm; i++) {
                    anyOn = anyOn || !g.m[i].sinkOff;
                }
                g.sinkReady = anyOn;
            } else {
                g.micReady = false;
            }
        }
        cur = a;
        (sink ? m.sinkSeq : m.srcSeq)++;
    }
    g.cv.notify_all();
}

void halLeAudioOnHciEvent(const uint8_t* ev, uint32_t len)
{
    if (ev == nullptr || len < 2 || (uint32_t)ev[1] + 2 > len) {
        return;
    }
    const uint8_t* p = ev + 2;
    const uint32_t n = ev[1];
    std::lock_guard<std::mutex> l(g.lock);
    if (ev[0] == 0x13) {
        forEachCompleted(p, n, [](uint16_t h, uint16_t c) { g.credits.completed(h, c); });
        g.nocpSeq++;
        const int64_t now = esp_timer_get_time();
        if (g.running.load() && g_lastNocpUs != 0) {
            g_nocpGapMaxUs = std::max<uint32_t>(g_nocpGapMaxUs, (uint32_t)std::min<int64_t>(now - g_lastNocpUs, UINT32_MAX));
        }
        g_lastNocpUs = g.running.load() ? now : 0;
    } else if (ev[0] == 0x05 && n >= 4) {
        const uint16_t h = (uint16_t)((p[1] | (p[2] << 8)) & 0x0FFF);
        if (p[0] != 0) {
            return;
        }
        for (int i = 0; i < g.nm; i++) {
            Member& m          = g.m[i];
            const bool cisLost = h == m.cisHandle && m.cisHandle != 0xFFFF;
            const bool aclLost = h == m.acl;
            if (!cisLost && !aclLost) {
                continue;
            }
            // CIS credits come back at every end of it - also when the ACL loss came first (cisUp already false)
            g.credits.dropHandle(m.cisHandle);
            if (aclLost) {
                m.aclLost   = true;
                g.attrCp[i] = g.attrSink[i] = g.attrSrc[i] = 0;
            }
            m.cisUp = false;  // ACL down = its CIS too (the controller reports it later, via supervision)
            if (i == 0) {
                g.micReady = false;
            }
            bool anyUp = false;
            for (int k = 0; k < g.nm; k++) {
                anyUp = anyUp || g.m[k].cisUp;
            }
            if (anyUp) {
                ESP_LOGW(kTag, "LE Audio: earbud %d - %s disconnected (reason 0x%02X), the other plays on (mono)", i,
                         aclLost ? "ACL" : "CIS", p[3]);
                // Live status: the status API and the automation (btJoinSet: restart when the other returns) see how
                // many actually play (previously members 2 / stereo true stayed until the end of the stream)
                int live = 0;
                for (int k = 0; k < g.nm; k++) {
                    live += g.m[k].cisUp ? 1 : 0;
                }
                g.st.members = live;
                g.st.stereo  = g.st.stereo && live >= 2;
                continue;
            }
            g.sinkReady = false;  // the output task sees the route change and pauses (halAudioBtLost)
            g.pcmCv.notify_all();  // le_audio_write_pcm: stream gone (under g.lock - no lost wakeup)
            // The stream thread ends by itself (no CIS); phase Failed so that the status API and the next start do not
            // take a dead stream for "streaming". During the procedure a lost ACL ends it (waitFor).
            if (g.st.phase != Phase::Off && (g.running.load() || (aclLost && i == 0))) {
                g.st.phase     = Phase::Failed;
                g.st.lastError = aclLost ? fmt("earbud %d disconnected (reason 0x%02X)", i, p[3])
                                         : fmt("CIS disconnected (reason 0x%02X)", p[3]);
            }
        }
    } else if (ev[0] == 0x3E && n >= 1 && p[0] == 0x19) {
        CisEstablished c;
        if (parseCisEstablished(p + 1, n - 1, c)) {
            for (int i = 0; i < g.nm; i++) {
                if (c.handle == g.m[i].cisHandle) {
                    g.m[i].cis   = c;
                    g.m[i].cisUp = c.status == 0;
                    g.m[i].cisSeq++;
                }
            }
        }
    }
    g.cv.notify_all();
    g.micCv.notify_all();
}

void halLeAudioOnIsoRx(const uint8_t* pkt, uint32_t len)
{
    IsoSdu s;
    if (g_reasm.feed(pkt, len, s) != IsoReassembler::Result::Sdu) {
        return;
    }
    std::lock_guard<std::mutex> l(g.lock);
    if (!g.micReady.load() || s.handle != g.m[0].cisHandle) {
        return;
    }
    g.st.rxSdu++;
    if (g.rxCount == kRxQueue) {
        g.rxHead = (g.rxHead + 1) % kRxQueue;  // the stream thread cannot keep up - the oldest SDU is dropped
        g.rxCount--;
        g.st.rxLost++;
    }
    RxSdu& r = g.rxq[(g.rxHead + g.rxCount) % kRxQueue];
    r.seq    = s.seq;
    r.status = s.status;
    r.len    = std::min<uint16_t>(s.len, sizeof(r.data));
    std::memcpy(r.data, s.data, r.len);
    g.rxCount++;
}

/* ------------------------------------ audio path ------------------------------------ */

bool halLeAudioSinkReady()
{
    return g.sinkReady.load();
}

void halLeAudioPushPcm(const int16_t* stereo, size_t frames, int volumePct)
{
    if (!g.sinkReady.load()) {
        return;
    }
    g.volume = volumePct;
    std::lock_guard<std::mutex> l(g.lock);
    if (g.down.cap - g.down.fill < frames * 2) {
        g.down.pop(nullptr, frames * 2);  // the encoder is stalled - old samples are dropped, latency does not grow
    }
    g.down.push(stereo, frames * 2);
}

// Blocking variant for producers without their own audio clock (le_audio_write_pcm): the stream thread takes one
// 10 ms frame per ISO interval, so waiting for room paces the producer to the CIS clock. Target: at most 40 ms
// queued (the thread drops frames above kDownMaxFr = 60 ms).
constexpr size_t kDownTargetFr = kSinkFrame * 4;

size_t halLeAudioWritePcm(const int16_t* stereo, size_t frames, int volumePct, int timeoutMs)
{
    if (stereo == nullptr || frames == 0 || !g.sinkReady.load()) {
        return 0;
    }
    frames   = std::min(frames, kDownTargetFr);
    g.volume = volumePct;
    std::unique_lock<std::mutex> l(g.lock);
    const bool room = g.pcmCv.wait_for(l, std::chrono::milliseconds(timeoutMs), [&] {
        return !g.sinkReady.load() || g.down.fill / 2 + frames <= kDownTargetFr;
    });
    if (!room || !g.sinkReady.load()) {
        return 0;
    }
    return g.down.push(stereo, frames * 2) / 2;
}

bool halLeAudioMicReady()
{
    return g.micReady.load();
}

size_t halLeAudioMicRead(int16_t* out, size_t n, int timeoutMs)
{
    std::unique_lock<std::mutex> l(g.lock);
    if (!g.micCv.wait_for(l, std::chrono::milliseconds(timeoutMs),
                          [&] { return g.mic.fill >= n || !g.micReady.load(); }) ||
        !g.micReady.load()) {
        return 0;
    }
    return g.mic.pop(out, n);
}

/* ------------------------------------ HAL API ------------------------------------ */

extern "C" void hal_le_audio_lab_phy(uint8_t phy)
{
    g_labPhy.store(phy);
}

bool lehost::btLeAudioStart(bool mic)
{
    std::lock_guard<std::recursive_mutex> op(g.opLock);
    const uint8_t labPhy = g_labPhy.exchange(0);  // this start only, never a later one
    {
        std::lock_guard<std::mutex> l(g.lock);
        if (g.running.load() && g.st.phase == Phase::Streaming && g.nm > 0 && g.m[0].cisUp) {
            return true;
        }
    }
    if (!g.threadDone.load()) {
        fail("previous stream is still ending");
        return false;
    }
    const bool ok = runStart(mic, labPhy);
    if (!ok) {
        for (int i = 0; i < kMaxMembers; i++) {
            g.attrSrc[i] = 0;
        }
    }
    return ok;
}

// Restart from the HAL (btLeAudioRestart) - the output task does not treat the gap as the earbuds going away.
static std::atomic<bool> g_restarting{false};

bool halLeAudioRestarting()
{
    return g_restarting.load();
}

bool lehost::btLeAudioRestart(bool mic)
{
    // Under one opLock (recursive): a stop requested in between the state check and the restart is not overwritten
    std::lock_guard<std::recursive_mutex> op(g.opLock);
    {
        std::lock_guard<std::mutex> l(g.lock);
        if (g.st.phase != Phase::Streaming) {
            return false;
        }
    }
    g_restarting = true;
    btLeAudioStop();
    const bool ok = btLeAudioStart(mic);
    g_restarting = false;
    if (!ok) {
        halAudioBtLost();  // the stream did not come back - the player pauses as when the earbuds disappear
    }
    return ok;
}

void lehost::btLeAudioStop()
{
    std::lock_guard<std::recursive_mutex> op(g.opLock);
    {
        std::lock_guard<std::mutex> l(g.lock);
        if (g.st.phase == Phase::Off) {
            return;
        }
    }
    g.sinkReady = false;
    g.micReady  = false;
    g.running   = false;
    g.micCv.notify_all();
    {
        std::lock_guard<std::mutex> l(g.lock);  // a writer between its check and its wait must not miss this
        g.pcmCv.notify_all();
    }
    // The stream thread ends within one interval (10 ms); the queues must not be freed while it still runs
    for (int i = 0; i < 100 && !g.threadDone.load(); i++) {
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    teardown(true);
    std::lock_guard<std::mutex> l(g.lock);
    if (g.threadDone.load()) {
        g.down.free();
        g.mic.free();
    } else {
        ESP_LOGE(kTag, "LE Audio: stream thread did not finish within 1 s - queues are kept (not freed while in use)");
    }
    g.st.phase = Phase::Off;
    g.st.mic   = false;
    ESP_LOGI(kTag, "LE Audio: stream stopped (SDUs sent %lu, dropped %lu, received %lu, lost %lu)",
             (unsigned long)g.st.txSdu, (unsigned long)g.st.txDropped, (unsigned long)g.st.rxSdu,
             (unsigned long)g.st.rxLost);
}

lehost::LeAudioStatus_t lehost::btLeAudioStatus()
{
    std::lock_guard<std::mutex> l(g.lock);
    lehost::LeAudioStatus_t s = g.st;
    for (int i = 0; i < kMaxMembers; i++) {
        const bool in = i < g.nm && s.phase != Phase::Off;
        s.location[i]  = in ? g.m[i].sinkCfg.location : 0;
        s.cisUp[i]     = in && g.m[i].cisUp && !g.m[i].sinkOff;
        s.cisHandle[i] = in ? g.m[i].cisHandle : 0xFFFF;
    }
    return s;
}

bool halLeAudioActive()
{
    std::lock_guard<std::mutex> l(g.lock);
    return g.st.phase == lehost::LeAudioPhase::Configuring || g.st.phase == lehost::LeAudioPhase::Streaming;
}
