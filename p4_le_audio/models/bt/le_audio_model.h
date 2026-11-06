#pragma once
// LE Audio (BAP unicast client), pure part: HCI ISO frames, ISO buffer credits, PACS, ASCS (ASE states,
// control point), HCI CIG/CIS parameters and LC3 configuration choice. No HAL, no UI - host tests
// (test/host/test_le_audio.cpp). Used by the firmware's LE Audio HAL.
//
// Everything that arrives over the air or from the controller is untrusted: parsers check every length and return
// false instead of reading out of bounds. Sources: Core 5.4 Vol 4 Part E (HCI: 5.4.5 ISO packets, 7.8.97-7.8.109 ISO
// commands, 7.7.65.25 LE CIS Established), BAP 1.0.1, ASCS 1.0, PACS 1.0, Assigned Numbers (Generic Audio LTV).

#include <cstddef>
#include <cstdint>

namespace app::leaudio {

/* ------------------------------------------ HCI ISO ------------------------------------------ */

constexpr uint8_t kH4Iso = 0x05;
// Packet_Boundary_Flag (PB) in the HCI ISO packet header
enum class IsoPb : uint8_t { First = 0, Continuation = 1, Complete = 2, Last = 3 };
// Packet_Status_Flag in an SDU from the controller to the host
enum class IsoStatus : uint8_t { Valid = 0, PossiblyInvalid = 1, Lost = 2 };

// One HCI ISO packet (with the H4 type byte 0x05) carrying the next fragment of an SDU. Call in a loop from offset = 0
// until offset == sduLen; each packet takes one controller buffer (credit). maxDataLen = ISO_Data_Packet_Length from LE
// Read Buffer Size v2 (the largest ISO_Data_Load). The timestamp (hasTs) and sequence number go only in the first
// fragment. Returns the packet size in bytes, or 0 (bad arguments, buffer too small, maxDataLen < SDU header).
size_t buildIsoPacket(uint8_t* out, size_t cap, uint16_t handle, uint16_t seq, bool hasTs, uint32_t ts,
                      const uint8_t* sdu, uint16_t sduLen, uint16_t maxDataLen, uint16_t& offset);

// SDU assembled from HCI ISO packets (controller -> host). data points into the assembler buffer (valid until feed).
struct IsoSdu {
    uint16_t handle   = 0;
    bool hasTs        = false;
    uint32_t ts       = 0;  // us, controller clock (SDU reference point)
    uint16_t seq      = 0;
    IsoStatus status  = IsoStatus::Valid;
    const uint8_t* data = nullptr;
    uint16_t len      = 0;
};

// SDU reassembly from fragments, separately for each CIS handle (up to kSlots at a time). No allocation.
class IsoReassembler {
public:
    static constexpr int kSlots      = 4;
    static constexpr uint16_t kMaxSdu = 512;
    enum class Result { None, Sdu, Error };
    // pkt = the whole H4 packet (first byte 0x05). Sdu = out filled; Error = packet rejected (bad length,
    // fragment without a start, SDU longer than kMaxSdu) - the handle state is reset.
    Result feed(const uint8_t* pkt, size_t len, IsoSdu& out);
    void reset();

private:
    struct Slot {
        bool used        = false;
        bool active      = false;  // reassembly in progress (First was seen)
        uint16_t handle  = 0;
        IsoSdu hdr;
        uint16_t want    = 0;
        uint16_t have    = 0;
        uint8_t buf[kMaxSdu] = {};
    };
    Slot* slotFor(uint16_t handle);
    Slot slots_[kSlots];
};

// Controller ISO buffer credits: a shared Total_Num_ISO_Data_Packets pool (LE Read Buffer Size v2), accounted per
// CIS handle by the Number Of Completed Packets event. The controller cannot return more than it was sent - the excess
// (a bug or a hostile controller) is ignored and counted. Lost NOCPs (broken SDIO transfer, observed 2026-11-02) would
// hold a credit to the end of the stream without the sweep: each credit remembers when it was taken, NOCP frees oldest.
class IsoCredits {
public:
    static constexpr int kHandles = 4;
    static constexpr int kTimes   = 64;           // take times per handle (C6 pool: 32 credits)
    void reset(uint16_t total);
    bool take(uint16_t handle, uint32_t nowUs = 0);  // false: no free buffer or too many handles
    void completed(uint16_t handle, uint16_t n);   // Number Of Completed Packets
    void dropHandle(uint16_t handle);              // CIS disconnected / path removed: the controller frees its buffers
    // Credits taken longer ago than lostUs without a NOCP go back to the pool (the controller returns them in ~FT +
    // a queue of intervals); returns how many came back. A late NOCP for them is counted as bogus afterwards.
    uint16_t sweep(uint32_t nowUs, uint32_t lostUs);
    uint16_t available() const { return free_; }
    uint16_t outstanding(uint16_t handle) const;
    uint32_t bogus() const { return bogus_; }
    uint32_t recovered() const { return recovered_; }

private:
    struct H {
        uint16_t handle = 0xFFFF;
        uint16_t out    = 0;
        uint8_t head    = 0;              // oldest of the out entries in t (ring)
        uint32_t t[kTimes] = {};
    };
    void popOldest(H& h, uint16_t n);
    uint16_t total_ = 0;
    uint16_t free_  = 0;
    uint32_t bogus_ = 0;
    uint32_t recovered_ = 0;
    H h_[kHandles];
};

// SDUs of one CIS waiting for an ISO credit (the txq option): instead of dropping an SDU when a NOCP is late over the
// shared SDIO bus, the host holds it until the credit returns, but only until its deadline (dueUs) - the C6 drops later
// ones as late anyway (sdu_late), and a run of late SDUs shifts the CIG mapping. FIFO order: the SDU sequence number =
// the interval number, the C6 maps it to a CIG event and drops an SDU for an event already taken by a younger one.
class IsoTxQueue {
public:
    static constexpr int kCap         = 4;    // deadline <= 3 intervals - no more SDUs wait at once
    static constexpr uint16_t kMaxSdu = 400;  // as an LC3 frame (lc3OctetsValid)
    struct Sdu {
        uint16_t seq   = 0;
        uint16_t len   = 0;
        int64_t dueUs  = 0;
        uint8_t data[kMaxSdu] = {};
    };
    // false = SDU longer than kMaxSdu (does not fit). A full queue drops the oldest (dropped + 1).
    bool push(uint16_t seq, const uint8_t* d, uint16_t len, int64_t dueUs, uint32_t& dropped);
    // Drops SDUs past their deadline from the front (nowUs > dueUs); returns how many.
    uint32_t expire(int64_t nowUs);
    const Sdu* front() const { return n_ > 0 ? &q_[head_] : nullptr; }
    void pop();
    void clear() { head_ = n_ = 0; }
    int size() const { return n_; }

private:
    Sdu q_[kCap];
    int head_ = 0, n_ = 0;
};

// Number Of Completed Packets (0x13): (handle, count) pairs - interleaved encoding as in Zephyr and BlueZ.
// Calls fn(handle, count) for each pair; false on a bad length.
template <class F>
bool forEachCompleted(const uint8_t* params, size_t len, F fn)
{
    if (len < 1 || len < 1u + (size_t)params[0] * 4u) {
        return false;
    }
    for (uint8_t i = 0; i < params[0]; i++) {
        const uint8_t* p = params + 1 + i * 4;
        fn((uint16_t)((p[0] | (p[1] << 8)) & 0x0FFF), (uint16_t)(p[2] | (p[3] << 8)));
    }
    return true;
}

/* -------------------------------------------- LC3 / PACS -------------------------------------------- */

// Codes from Assigned Numbers (Codec_Specific_Configuration): Sampling_Frequency and Frame_Duration
int rateFromCode(uint8_t code);     // 0x03 -> 16000 ... 0x08 -> 48000; 0 = unknown
uint8_t codeFromRate(int rateHz);   // the inverse, 0 = no code
// Samples per LC3 frame (channel) - 0 for an unsupported pair (liblc3: 8/16/24/32/48 kHz, 7.5/10 ms).
int lc3FrameSamples(int rateHz, int durUs);
// LC3 frame bytes allowed by liblc3 (20..400).
bool lc3OctetsValid(int octets);

struct Lc3Cap {
    uint16_t freqMask     = 0;     // bit (code-1): 0x0004 16 kHz, 0x0010 24 kHz, 0x0020 32 kHz, 0x0080 48 kHz
    uint8_t durMask       = 0;     // bit0 7.5 ms, bit1 10 ms
    uint8_t chanMask      = 0x01;  // channel counts (bit0 = 1 channel); default 1
    uint16_t octMin       = 0;
    uint16_t octMax       = 0;
    uint8_t maxFramesSdu  = 1;
};
struct PacSet {
    static constexpr int kMax = 6;
    Lc3Cap caps[kMax];
    int n = 0;  // LC3 records only (Codec_ID 0x06); other codecs skipped
};
// Value of the Sink PAC (0x2BC9) / Source PAC (0x2BCB) characteristic. false = malformed structure.
bool parsePac(const uint8_t* d, size_t len, PacSet& out);

struct CodecCfg {
    int rateHz        = 0;
    int durUs         = 10000;
    uint16_t octets   = 0;
    uint32_t location = 0;  // Audio_Channel_Allocation (bit0 Front Left, bit1 Front Right); 0 = mono, no location
};
// Configuration choice from the BAP settings (Table 3.11/3.12) supported by the PAC, always 10 ms:
// earbuds (Sink): 48_4 (120 B) > 48_2 (100 B) > 32_2 (80 B) > 24_2 (60 B) > 16_2 (40 B);
// microphone (Source): 16_2 (40 B) > 32_2 (80 B) > 24_2 (60 B) - 16 kHz is the sample rate of the host speech path.
bool chooseSinkConfig(const PacSet& pac, uint32_t location, CodecCfg& out);
bool chooseSourceConfig(const PacSet& pac, uint32_t location, CodecCfg& out);
// Codec_Specific_Configuration (LTV) for Config Codec. Returns the length or 0.
size_t buildCodecCfgLtv(const CodecCfg& c, uint8_t* out, size_t cap);

/* --------------------------------------------- ASCS --------------------------------------------- */

enum class AseState : uint8_t {
    Idle = 0, CodecConfigured = 1, QosConfigured = 2, Enabling = 3, Streaming = 4, Disabling = 5, Releasing = 6,
    Invalid = 0xFF
};
const char* aseStateName(AseState s);

// Server preferences from an ASE in the Codec Configured state (ASCS 4.1, Table 4.3)
struct AsePref {
    uint8_t framing        = 0;  // 0 = unframed supported, 1 = framed only
    uint8_t phyMask        = 0;  // bit0 1M, bit1 2M, bit2 Coded
    uint8_t rtn            = 0;
    uint16_t maxLatencyMs  = 0;
    uint32_t pdMinUs       = 0, pdMaxUs = 0;
    uint32_t prefPdMinUs   = 0, prefPdMaxUs = 0;
};
struct AseInfo {
    uint8_t id     = 0;
    AseState state = AseState::Invalid;
    AsePref pref;            // Codec Configured
    uint8_t cigId = 0, cisId = 0;  // QoS Configured / Enabling / Streaming / Disabling
};
// Value of the Sink/Source ASE characteristic (read or notification). false = bad length / unknown state.
bool parseAse(const uint8_t* d, size_t len, AseInfo& out);
// Whether the server may go from -> to (ASCS 1.0, ch. 3: Sink and Source ASE state diagrams; same -> same is
// allowed for Codec/QoS Configured and metadata updates). A disallowed transition is a server error, we abort.
bool aseTransitionAllowed(bool sink, AseState from, AseState to);

enum class CpOp : uint8_t {
    ConfigCodec = 1, ConfigQos = 2, Enable = 3, RecvStartReady = 4, Disable = 5, RecvStopReady = 6,
    UpdateMetadata = 7, Release = 8
};
struct QosCfg {
    uint8_t cigId          = 1;
    uint8_t cisId          = 0;
    uint32_t sduIntervalUs = 10000;
    uint8_t framing        = 0;  // unframed
    uint8_t phy            = 0x02;
    uint16_t maxSdu        = 0;
    uint8_t rtn            = 2;
    uint16_t maxLatencyMs  = 20;
    uint32_t pdUs          = 40000;
};
// QoS choice from the server preferences: unframed (framed-only = false), 2M if allowed, RTN and latency from the
// preferences (clamped), presentation delay: the preferred minimum if within range, otherwise the range minimum.
bool chooseQos(const AsePref& p, const CodecCfg& c, uint8_t cigId, uint8_t cisId, QosCfg& out);

// Writes to the ASE Control Point (without the ATT header). They return the length or 0.
size_t cpConfigCodec(uint8_t* out, size_t cap, uint8_t aseId, const CodecCfg& c, uint8_t targetLatency,
                     uint8_t targetPhy);
size_t cpConfigQos(uint8_t* out, size_t cap, uint8_t aseId, const QosCfg& q);
// Enable with Streaming_Audio_Contexts metadata (0x0004 = Media, 0x0002 = Conversational) and - when ccid >= 0 -
// a CCID_List with one CCID (the content control service on our side: GMCS for music, GTBS for calls)
size_t cpEnable(uint8_t* out, size_t cap, uint8_t aseId, uint16_t contexts, int ccid = -1);
// Receiver Start Ready / Disable / Receiver Stop Ready / Release for one ASE
size_t cpSimple(uint8_t* out, size_t cap, CpOp op, uint8_t aseId);

// ASE Control Point notification: a response for each ASE of the operation.
struct CpResult {
    static constexpr int kMax = 8;
    uint8_t op = 0;
    int n      = 0;
    struct Item {
        uint8_t aseId = 0, code = 0, reason = 0;
    } items[kMax];
};
bool parseCpNotify(const uint8_t* d, size_t len, CpResult& out);
const char* cpCodeText(uint8_t code);  // ASCS Table 5.1 (English text, ASCII - for logs and the error message)

/* ------------------------------------------ HCI CIG/CIS ------------------------------------------ */

struct CisParams {
    uint8_t cisId     = 0;
    uint16_t sduCtoP  = 0;  // Max_SDU Central -> Peripheral (0 = no direction)
    uint16_t sduPtoC  = 0;
    uint8_t phyCtoP   = 0x02, phyPtoC = 0x02;
    uint8_t rtnCtoP   = 2, rtnPtoC = 2;
};
struct CigParams {
    uint8_t cigId           = 1;
    uint32_t sduIntervalUs  = 10000;  // both directions
    uint16_t latencyCtoPMs  = 20;
    uint16_t latencyPtoCMs  = 20;
    int nCis                = 0;
    CisParams cis[2];
};
// LE Set CIG Parameters (0x2062) parameters. Returns the length or 0.
size_t buildSetCigParams(uint8_t* out, size_t cap, const CigParams& p);
// LE Create CIS (0x2064) for (CIS, ACL) pairs
size_t buildCreateCis(uint8_t* out, size_t cap, const uint16_t* cisHandles, const uint16_t* aclHandles, int n);

// ACL interval with the earbuds: a multiple of 10 ms (8 x 1.25 ms). The C6 CIS emulation is a pseudo ACL link with
// a 10 ms ISO interval - with another ACL interval the events of the two collide every few intervals
// (docs/c6-link-layer.md). 30 ms at connection time (min = max), in replies - the closest to that value.
constexpr uint16_t kAclIntervalUnits = 24;
inline bool aclIntervalCisFriendly(uint16_t units) { return units != 0 && units % 8 == 0; }

// HCI command to send; opcode 0 = nothing.
struct HciCmd {
    uint16_t opcode = 0;
    uint8_t len     = 0;
    uint8_t p[14]   = {};
};
// Host reply to LE Remote Connection Parameter Request (0x3E, subevent 0x06; ev starts at the subevent code):
// LE Remote Connection Parameter Request Reply (0x2020) with the multiple of 10 ms from the earbuds' range closest to
// kAclIntervalUnits, else a Negative Reply (0x2021, 0x3B Unacceptable Connection Parameters). Without a reply
// the earbuds' LL procedure ends after 40 s with an ACL drop (0x22).
HciCmd connParamRequestReply(const uint8_t* ev, size_t len);
// One CIS carries both directions and the C6 emulation has one PHY per link: different PHYs -> both 1M (always ok).
void unifyCisPhy(QosCfg& sink, QosCfg& src);
// Streaming_Audio_Contexts for Enable: BAP requires a subset of the server's Available Audio Contexts (PACS 0x2BCD),
// otherwise Enable is rejected. The wanted one if available; else Unspecified (0x0001) if available; else the lowest
// available one; available = 0 (read failed or nothing) -> the wanted one.
uint16_t pickStreamingContext(uint16_t want, uint16_t available);
// LE Setup ISO Data Path (0x206E) over HCI (Data_Path_ID 0), transparent codec. dirOutput: controller -> host.
size_t buildSetupIsoPath(uint8_t* out, size_t cap, uint16_t cisHandle, bool dirOutput);
// LE Remove ISO Data Path (0x206F) only for directions that were set up - a direction without a path = Command
// Disallowed and the controller then removes none (Core Vol 4 Part E 7.8.110). Returns the length or 0 (no direction).
size_t buildRemoveIsoPath(uint8_t* out, size_t cap, uint16_t cisHandle, bool input, bool output);

// LE CIS Established (subevent 0x19): parameters after the subevent code
struct CisEstablished {
    uint8_t status          = 0xFF;
    uint16_t handle         = 0;
    uint32_t cigSyncDelayUs = 0, cisSyncDelayUs = 0;
    uint32_t latCtoPUs      = 0, latPtoCUs = 0;
    uint8_t phyCtoP = 0, phyPtoC = 0, nse = 0, bnCtoP = 0, bnPtoC = 0, ftCtoP = 0, ftPtoC = 0;
    uint16_t maxPduCtoP = 0, maxPduPtoC = 0;
    uint16_t isoInterval = 0;  // x 1.25 ms
};
bool parseCisEstablished(const uint8_t* p, size_t len, CisEstablished& out);

// LE Read Buffer Size v2 (0x2060): Return_Parameters after the status
struct IsoBuffers {
    uint16_t aclLen = 0;
    uint8_t aclNum  = 0;
    uint16_t isoLen = 0;
    uint8_t isoNum  = 0;
};
bool parseReadBufferSizeV2(const uint8_t* ret, size_t len, IsoBuffers& out);

/* ------------------------------ GATT: second earbud of the set (stereo) ------------------------------ */
// The main connection has the PACS/ASCS handles from the BT HAL; the second earbud is discovered by the LE Audio
// module (Core Vol 3 Part G 4.4.1, 4.6.1). Responses are fed in a loop from start 0x0001; nextStart = 0 ends it
// (end of the database or a bad response).

// Read By Group Type Response (0x11) for a Primary Service: range of service uuid (UUID16). true = found here.
bool findServiceInGroups(const uint8_t* rsp, size_t len, uint16_t uuid, uint16_t& start, uint16_t& end,
                         uint16_t& nextStart);

// ASCS value handles from a Read By Type Response (0x09) for characteristic declarations (0x2803, UUID16): Sink ASE
// (0x2BC4), Source ASE (0x2BC5) - two of each, ASE Control Point (0x2BC6). Later responses append.
struct AscsHandles {
    uint16_t cp      = 0;
    uint16_t sink[2] = {0, 0};
    uint16_t src[2]  = {0, 0};
};
void collectAscsHandles(const uint8_t* rsp, size_t len, AscsHandles& h, uint16_t& nextStart);

/* ------------------------------------------- Stereo ------------------------------------------- */

enum class Channel : uint8_t { Mono, Left, Right };
// Channel for an earbud with location loc (Audio_Channel_Allocation) when n earbuds play: one - mono (L + R) / 2,
// two - left 0x1, right 0x2; another location with two - mono.
Channel channelFor(uint32_t loc, int n);
// A frame of one channel from interleaved stereo PCM (frames L/R pairs) times gain, clipped to int16.
void pickChannel(const int16_t* stereo, size_t frames, Channel ch, float gain, int16_t* out);

}  // namespace app::leaudio
