// LE Audio (BAP unicast client), pure part. Description and sources: le_audio_model.h.
#include "le_audio_model.h"

#include <algorithm>
#include <cstring>

namespace app::leaudio {

namespace {

inline void put16(uint8_t* p, uint32_t v)
{
    p[0] = (uint8_t)(v & 0xFF);
    p[1] = (uint8_t)((v >> 8) & 0xFF);
}
inline void put24(uint8_t* p, uint32_t v)
{
    put16(p, v);
    p[2] = (uint8_t)((v >> 16) & 0xFF);
}
inline uint16_t get16(const uint8_t* p)
{
    return (uint16_t)(p[0] | (p[1] << 8));
}
inline uint32_t get24(const uint8_t* p)
{
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16);
}
inline uint32_t get32(const uint8_t* p)
{
    return get24(p) | ((uint32_t)p[3] << 24);
}

}  // namespace

/* ------------------------------------------ HCI ISO ------------------------------------------ */

size_t buildIsoPacket(uint8_t* out, size_t cap, uint16_t handle, uint16_t seq, bool hasTs, uint32_t ts,
                      const uint8_t* sdu, uint16_t sduLen, uint16_t maxDataLen, uint16_t& offset)
{
    if (out == nullptr || handle > 0x0EFF || sduLen > 0x0FFF || offset > sduLen || (sduLen > 0 && sdu == nullptr) ||
        maxDataLen > 0x3FFF) {
        return 0;
    }
    const bool first     = offset == 0;
    const size_t loadHdr = first ? (hasTs ? 8u : 4u) : 0u;
    if (maxDataLen <= loadHdr) {
        return 0;
    }
    const size_t room  = maxDataLen - loadHdr;
    const size_t left  = sduLen - offset;
    const size_t chunk = left < room ? left : room;
    const bool last    = offset + chunk == sduLen;
    if (!first && chunk == 0) {
        return 0;  // nothing left to send - the whole SDU is already out
    }
    const size_t load  = loadHdr + chunk;
    const size_t total = 1 + 4 + load;
    if (cap < total) {
        return 0;
    }
    IsoPb pb = first ? (last ? IsoPb::Complete : IsoPb::First) : (last ? IsoPb::Last : IsoPb::Continuation);
    out[0]   = kH4Iso;
    // Connection_Handle (12) | PB_Flag (2) | TS_Flag (1) | RFU
    put16(out + 1, (uint32_t)handle | ((uint32_t)pb << 12) | ((first && hasTs) ? 0x4000u : 0u));
    put16(out + 3, (uint32_t)load);  // ISO_Data_Load_Length (14) | RFU
    uint8_t* p = out + 5;
    if (first) {
        if (hasTs) {
            put16(p, ts & 0xFFFF);
            put16(p + 2, ts >> 16);
            p += 4;
        }
        put16(p, seq);
        put16(p + 2, sduLen);  // ISO_SDU_Length (12) | RFU | Packet_Status_Flag (always 0 from the host)
        p += 4;
    }
    if (chunk > 0) {
        std::memcpy(p, sdu + offset, chunk);
    }
    offset = (uint16_t)(offset + chunk);
    return total;
}

void IsoReassembler::reset()
{
    for (auto& s : slots_) {
        s.used   = false;
        s.active = false;
    }
}

IsoReassembler::Slot* IsoReassembler::slotFor(uint16_t handle)
{
    Slot* freeSlot = nullptr;
    for (auto& s : slots_) {
        if (s.used && s.handle == handle) {
            return &s;
        }
        if (!s.used && freeSlot == nullptr) {
            freeSlot = &s;
        }
    }
    if (freeSlot != nullptr) {
        freeSlot->used   = true;
        freeSlot->active = false;
        freeSlot->handle = handle;
    }
    return freeSlot;
}

IsoReassembler::Result IsoReassembler::feed(const uint8_t* pkt, size_t len, IsoSdu& out)
{
    if (pkt == nullptr || len < 5 || pkt[0] != kH4Iso) {
        return Result::Error;
    }
    const uint16_t hdr    = get16(pkt + 1);
    const uint16_t handle = hdr & 0x0FFF;
    const auto pb         = (IsoPb)((hdr >> 12) & 0x03);
    const bool ts         = (hdr & 0x4000) != 0;
    const uint16_t load   = get16(pkt + 3) & 0x3FFF;
    if ((size_t)load + 5 != len) {
        return Result::Error;  // the header length must match what actually arrived
    }
    Slot* s = slotFor(handle);
    if (s == nullptr) {
        return Result::Error;
    }
    const uint8_t* p = pkt + 5;
    size_t left      = load;
    if (pb == IsoPb::First || pb == IsoPb::Complete) {
        s->active  = false;
        s->hdr     = IsoSdu{};
        s->hdr.handle = handle;
        if (ts) {
            if (left < 4) {
                return Result::Error;
            }
            s->hdr.hasTs = true;
            s->hdr.ts    = get32(p);
            p += 4;
            left -= 4;
        }
        if (left < 4) {
            return Result::Error;
        }
        s->hdr.seq        = get16(p);
        const uint16_t sl = get16(p + 2);
        s->hdr.status     = (IsoStatus)((sl >> 14) & 0x03);
        s->want           = sl & 0x0FFF;
        s->have           = 0;
        p += 4;
        left -= 4;
        if (s->want > kMaxSdu || left > s->want) {
            return Result::Error;
        }
        s->active = true;
    } else if (!s->active || ts) {
        s->active = false;
        return Result::Error;  // continuation without a start, or a timestamp outside the first fragment
    }
    if (s->have + left > s->want) {
        s->active = false;
        return Result::Error;
    }
    if (left > 0) {
        std::memcpy(s->buf + s->have, p, left);
        s->have = (uint16_t)(s->have + left);
    }
    const bool end = pb == IsoPb::Complete || pb == IsoPb::Last;
    if (!end) {
        return Result::None;
    }
    s->active = false;
    if (s->have != s->want) {
        // an SDU flagged "lost" may have length 0; any other incomplete SDU is an error
        if (!(s->hdr.status == IsoStatus::Lost && s->have == 0)) {
            return Result::Error;
        }
    }
    out      = s->hdr;
    out.data = s->buf;
    out.len  = s->have;
    return Result::Sdu;
}

void IsoCredits::reset(uint16_t total)
{
    total_     = total;
    free_      = total;
    bogus_     = 0;
    recovered_ = 0;
    for (auto& h : h_) {
        h = H{};
    }
}

bool IsoCredits::take(uint16_t handle, uint32_t nowUs)
{
    if (free_ == 0) {
        return false;
    }
    H* slot = nullptr;
    for (auto& h : h_) {
        if (h.handle == handle) {
            slot = &h;
            break;
        }
        if (slot == nullptr && h.handle == 0xFFFF) {
            slot = &h;
        }
    }
    if (slot == nullptr) {
        return false;
    }
    slot->handle = handle;
    if (slot->out < kTimes) {
        slot->t[(slot->head + slot->out) % kTimes] = nowUs;
    } else {
        slot->t[(slot->head + kTimes - 1) % kTimes] = nowUs;  // more than kTimes in flight: the youngest overwrites
    }
    slot->out++;
    free_--;
    return true;
}

void IsoCredits::popOldest(H& h, uint16_t n)
{
    h.head = (uint8_t)((h.head + (n < kTimes ? n : kTimes)) % kTimes);
    h.out  = (uint16_t)(h.out - n);
    free_  = (uint16_t)(free_ + n);
    if (h.out == 0) {
        h = H{};
    }
}

uint16_t IsoCredits::sweep(uint32_t nowUs, uint32_t lostUs)
{
    uint16_t back = 0;
    for (auto& h : h_) {
        while (h.handle != 0xFFFF && h.out > 0 && (uint32_t)(nowUs - h.t[h.head]) > lostUs) {
            popOldest(h, 1);
            back++;
        }
    }
    recovered_ += back;
    return back;
}

void IsoCredits::completed(uint16_t handle, uint16_t n)
{
    for (auto& h : h_) {
        if (h.handle == handle) {
            const uint16_t ok = n <= h.out ? n : h.out;
            bogus_ += (uint32_t)(n - ok);
            popOldest(h, ok);
            return;
        }
    }
    bogus_ += n;  // a handle we sent nothing on (e.g. ACL) - not our pool
}

void IsoCredits::dropHandle(uint16_t handle)
{
    for (auto& h : h_) {
        if (h.handle == handle) {
            free_    = (uint16_t)(free_ + h.out);
            h        = H{};
        }
    }
    if (free_ > total_) {
        free_ = total_;
    }
}

uint16_t IsoCredits::outstanding(uint16_t handle) const
{
    for (const auto& h : h_) {
        if (h.handle == handle) {
            return h.out;
        }
    }
    return 0;
}

bool IsoTxQueue::push(uint16_t seq, const uint8_t* d, uint16_t len, int64_t dueUs, uint32_t& dropped)
{
    if (len > kMaxSdu || (len > 0 && d == nullptr)) {
        return false;
    }
    if (n_ == kCap) {
        pop();
        dropped++;
    }
    Sdu& s = q_[(head_ + n_) % kCap];
    s.seq   = seq;
    s.len   = len;
    s.dueUs = dueUs;
    if (len > 0) {
        std::memcpy(s.data, d, len);
    }
    n_++;
    return true;
}

uint32_t IsoTxQueue::expire(int64_t nowUs)
{
    uint32_t k = 0;
    while (n_ > 0 && nowUs > q_[head_].dueUs) {
        pop();
        k++;
    }
    return k;
}

void IsoTxQueue::pop()
{
    if (n_ > 0) {
        head_ = (head_ + 1) % kCap;
        n_--;
    }
}

/* -------------------------------------------- LC3 / PACS -------------------------------------------- */

int rateFromCode(uint8_t code)
{
    switch (code) {
        case 0x01: return 8000;
        case 0x02: return 11025;
        case 0x03: return 16000;
        case 0x04: return 22050;
        case 0x05: return 24000;
        case 0x06: return 32000;
        case 0x07: return 44100;
        case 0x08: return 48000;
        default: return 0;
    }
}

uint8_t codeFromRate(int rateHz)
{
    for (uint8_t c = 1; c <= 8; c++) {
        if (rateFromCode(c) == rateHz) {
            return c;
        }
    }
    return 0;
}

int lc3FrameSamples(int rateHz, int durUs)
{
    if (rateHz != 8000 && rateHz != 16000 && rateHz != 24000 && rateHz != 32000 && rateHz != 48000) {
        return 0;
    }
    if (durUs != 7500 && durUs != 10000) {
        return 0;
    }
    return (int)((int64_t)rateHz * durUs / 1000000);
}

bool lc3OctetsValid(int octets)
{
    return octets >= 20 && octets <= 400;
}

bool parsePac(const uint8_t* d, size_t len, PacSet& out)
{
    out = PacSet{};
    if (d == nullptr || len < 1) {
        return false;
    }
    const uint8_t nRec = d[0];
    size_t off         = 1;
    for (uint8_t r = 0; r < nRec; r++) {
        if (off + 6 > len) {
            return false;
        }
        const uint8_t codingFmt = d[off];
        const uint8_t capLen    = d[off + 5];
        off += 6;
        if (off + capLen + 1 > len) {
            return false;
        }
        const uint8_t* cap = d + off;
        const uint8_t metaLen = d[off + capLen];
        if (off + capLen + 1 + metaLen > len) {
            return false;
        }
        Lc3Cap c;
        bool ok = true;
        for (size_t i = 0; i < capLen;) {
            const uint8_t l = cap[i];
            if (l == 0) {
                i++;
                continue;  // empty LTV - allowed, skip it
            }
            if (i + 1 + l > capLen) {
                ok = false;
                break;
            }
            const uint8_t type = cap[i + 1];
            const uint8_t* v   = cap + i + 2;
            const uint8_t vl   = (uint8_t)(l - 1);
            if (type == 0x01 && vl == 2) {
                c.freqMask = get16(v);
            } else if (type == 0x02 && vl == 1) {
                c.durMask = v[0];
            } else if (type == 0x03 && vl == 1) {
                c.chanMask = v[0];
            } else if (type == 0x04 && vl == 4) {
                c.octMin = get16(v);
                c.octMax = get16(v + 2);
            } else if (type == 0x05 && vl == 1) {
                c.maxFramesSdu = v[0];
            }
            i += 1u + l;
        }
        off += capLen + 1u + metaLen;
        if (!ok) {
            return false;
        }
        if (codingFmt == 0x06 && out.n < PacSet::kMax) {
            out.caps[out.n++] = c;
        }
    }
    return true;
}

namespace {

struct Preset {
    int rate;
    uint16_t octets;
};

bool pickFrom(const PacSet& pac, const Preset* presets, int n, uint32_t location, CodecCfg& out)
{
    for (int i = 0; i < n; i++) {
        const uint8_t code = codeFromRate(presets[i].rate);
        for (int k = 0; k < pac.n; k++) {
            const Lc3Cap& c = pac.caps[k];
            if (!(c.freqMask & (1u << (code - 1))) || !(c.durMask & 0x02) || !(c.chanMask & 0x01)) {
                continue;
            }
            if (presets[i].octets < c.octMin || presets[i].octets > c.octMax) {
                continue;
            }
            out          = CodecCfg{};
            out.rateHz   = presets[i].rate;
            out.durUs    = 10000;
            out.octets   = presets[i].octets;
            out.location = location;
            return true;
        }
    }
    return false;
}

}  // namespace

bool chooseSinkConfig(const PacSet& pac, uint32_t location, CodecCfg& out)
{
    static const Preset kSink[] = {{48000, 120}, {48000, 100}, {32000, 80}, {24000, 60}, {16000, 40}};
    return pickFrom(pac, kSink, (int)(sizeof(kSink) / sizeof(kSink[0])), location, out);
}

bool chooseSourceConfig(const PacSet& pac, uint32_t location, CodecCfg& out)
{
    static const Preset kSrc[] = {{16000, 40}, {32000, 80}, {24000, 60}};
    return pickFrom(pac, kSrc, (int)(sizeof(kSrc) / sizeof(kSrc[0])), location, out);
}

size_t buildCodecCfgLtv(const CodecCfg& c, uint8_t* out, size_t cap)
{
    const uint8_t code = codeFromRate(c.rateHz);
    if (out == nullptr || code == 0 || (c.durUs != 7500 && c.durUs != 10000) || !lc3OctetsValid(c.octets)) {
        return 0;
    }
    const size_t n = c.location != 0 ? 16 : 10;
    if (cap < n) {
        return 0;
    }
    uint8_t* p = out;
    *p++       = 0x02;
    *p++       = 0x01;  // Sampling_Frequency
    *p++       = code;
    *p++       = 0x02;
    *p++       = 0x02;  // Frame_Duration
    *p++       = c.durUs == 10000 ? 0x01 : 0x00;
    if (c.location != 0) {
        *p++ = 0x05;
        *p++ = 0x03;  // Audio_Channel_Allocation
        put16(p, c.location & 0xFFFF);
        put16(p + 2, c.location >> 16);
        p += 4;
    }
    *p++ = 0x03;
    *p++ = 0x04;  // Octets_Per_Codec_Frame
    put16(p, c.octets);
    p += 2;
    return (size_t)(p - out);
}

/* --------------------------------------------- ASCS --------------------------------------------- */

const char* aseStateName(AseState s)
{
    switch (s) {
        case AseState::Idle: return "Idle";
        case AseState::CodecConfigured: return "Codec Configured";
        case AseState::QosConfigured: return "QoS Configured";
        case AseState::Enabling: return "Enabling";
        case AseState::Streaming: return "Streaming";
        case AseState::Disabling: return "Disabling";
        case AseState::Releasing: return "Releasing";
        default: return "?";
    }
}

bool parseAse(const uint8_t* d, size_t len, AseInfo& out)
{
    out = AseInfo{};
    if (d == nullptr || len < 2 || d[0] == 0) {
        return false;
    }
    out.id          = d[0];
    const uint8_t s = d[1];
    const uint8_t* p = d + 2;
    const size_t left = len - 2;
    switch (s) {
        case 0:
        case 6:
            break;
        case 1: {
            // Framing, Preferred_PHY, Preferred_RTN, Max_Transport_Latency(2), PD min(3) max(3), pref min(3) max(3),
            // Codec_ID(5), Codec_Specific_Configuration_Length(1), configuration
            if (left < 23 || left < 23u + p[22]) {
                return false;
            }
            out.pref.framing      = p[0];
            out.pref.phyMask      = p[1];
            out.pref.rtn          = p[2];
            out.pref.maxLatencyMs = get16(p + 3);
            out.pref.pdMinUs      = get24(p + 5);
            out.pref.pdMaxUs      = get24(p + 8);
            out.pref.prefPdMinUs  = get24(p + 11);
            out.pref.prefPdMaxUs  = get24(p + 14);
            break;
        }
        case 2:
            // CIG_ID, CIS_ID, SDU_Interval(3), Framing, PHY, Max_SDU(2), RTN, Max_Transport_Latency(2), PD(3)
            if (left < 15) {
                return false;
            }
            out.cigId = p[0];
            out.cisId = p[1];
            break;
        case 3:
        case 4:
        case 5:
            // CIG_ID, CIS_ID, Metadata_Length, metadata
            if (left < 3 || left < 3u + p[2]) {
                return false;
            }
            out.cigId = p[0];
            out.cisId = p[1];
            break;
        default:
            return false;
    }
    out.state = (AseState)s;
    return true;
}

bool aseTransitionAllowed(bool sink, AseState from, AseState to)
{
    using S = AseState;
    if (to == S::Releasing) {
        return from != S::Idle && from != S::Releasing;  // Release or link loss from every configured state
    }
    switch (from) {
        case S::Idle: return to == S::CodecConfigured;
        case S::CodecConfigured: return to == S::CodecConfigured || to == S::QosConfigured;
        case S::QosConfigured: return to == S::QosConfigured || to == S::CodecConfigured || to == S::Enabling;
        case S::Enabling:
            // Sink: Disable -> QoS Configured at once; Source: Disable -> Disabling. CIS loss: both -> QoS Configured.
            return to == S::Enabling || to == S::Streaming || to == S::QosConfigured || (!sink && to == S::Disabling);
        case S::Streaming: return to == S::Streaming || to == S::QosConfigured || (!sink && to == S::Disabling);
        case S::Disabling: return !sink && (to == S::Disabling || to == S::QosConfigured);
        case S::Releasing: return to == S::Idle || to == S::CodecConfigured;
        default: return false;
    }
}

HciCmd connParamRequestReply(const uint8_t* ev, size_t len)
{
    // ev: subevent 0x06, handle(2), Interval_Min(2), Interval_Max(2), Max_Latency(2), Timeout(2)
    HciCmd r;
    if (ev == nullptr || len < 11 || ev[0] != 0x06) {
        return r;
    }
    const uint16_t mn = get16(ev + 3), mx = get16(ev + 5);
    // Multiples of 8 units within [mn, mx]; the one closest to kAclIntervalUnits (also a multiple of 8) clamps to them.
    const uint16_t lo = (uint16_t)((mn + 7u) / 8u * 8u), hi = (uint16_t)(mx / 8u * 8u);
    if (mn < 6 || mx > 3200 || lo > hi) {
        r.opcode = 0x2021;
        r.len    = 3;
        std::memcpy(r.p, ev + 1, 2);
        r.p[2] = 0x3B;  // Unacceptable Connection Parameters
        return r;
    }
    const uint16_t itvl = std::clamp(kAclIntervalUnits, lo, hi);
    r.opcode            = 0x2020;
    r.len               = 14;
    std::memcpy(r.p, ev + 1, 2);
    put16(r.p + 2, itvl);
    put16(r.p + 4, itvl);
    std::memcpy(r.p + 6, ev + 7, 4);  // Max_Latency and Timeout as in the request; Min/Max_CE_Length = 0
    return r;
}

uint16_t pickStreamingContext(uint16_t want, uint16_t available)
{
    if (available == 0 || (available & want) == want) {
        return want;
    }
    if (available & 0x0001) {
        return 0x0001;  // Unspecified
    }
    return (uint16_t)(available & (~available + 1u));  // lowest set bit
}

void unifyCisPhy(QosCfg& sink, QosCfg& src)
{
    if (sink.phy != src.phy) {
        sink.phy = src.phy = 0x01;
    }
}

bool chooseQos(const AsePref& p, const CodecCfg& c, uint8_t cigId, uint8_t cisId, QosCfg& out)
{
    if (p.framing == 1 || !lc3OctetsValid(c.octets)) {
        return false;  // the server wants framed only - out of scope (unframed, SDU = one 10 ms frame)
    }
    out               = QosCfg{};
    out.cigId         = cigId;
    out.cisId         = cisId;
    out.sduIntervalUs = (uint32_t)c.durUs;
    out.framing       = 0;
    out.phy           = (p.phyMask & 0x02) ? 0x02 : 0x01;
    out.maxSdu        = c.octets;
    out.rtn           = p.rtn > 4 ? 4 : p.rtn;
    // Max_Transport_Latency: the server preference, but at least 10 ms (one interval) and at most 100 ms
    uint16_t lat = p.maxLatencyMs;
    if (lat < 10) {
        lat = 10;
    }
    if (lat > 100) {
        lat = 100;
    }
    out.maxLatencyMs = lat;
    if (p.pdMaxUs == 0 || p.pdMaxUs < p.pdMinUs) {
        return false;  // the presentation delay range is mandatory (ASCS Table 4.3)
    }
    // A minimum of 0 is legal, but 0 us is no delay - 40 ms (BAP: a unicast server supports it) if within the range.
    uint32_t pd = p.pdMinUs != 0 ? p.pdMinUs : std::min<uint32_t>(40000, p.pdMaxUs);
    if (p.prefPdMinUs != 0 && p.prefPdMinUs >= p.pdMinUs && p.prefPdMinUs <= p.pdMaxUs) {
        pd = p.prefPdMinUs;
    }
    out.pdUs = pd;
    return true;
}

size_t cpConfigCodec(uint8_t* out, size_t cap, uint8_t aseId, const CodecCfg& c, uint8_t targetLatency,
                     uint8_t targetPhy)
{
    uint8_t ltv[32];
    const size_t ln = buildCodecCfgLtv(c, ltv, sizeof(ltv));
    if (out == nullptr || ln == 0 || aseId == 0 || cap < 11 + ln) {
        return 0;
    }
    out[0] = (uint8_t)CpOp::ConfigCodec;
    out[1] = 1;  // Number_of_ASEs
    out[2] = aseId;
    out[3] = targetLatency;  // 0x01 low latency, 0x02 balanced, 0x03 high reliability
    out[4] = targetPhy;
    out[5] = 0x06;  // Codec_ID: LC3
    out[6] = out[7] = out[8] = out[9] = 0;
    out[10] = (uint8_t)ln;
    std::memcpy(out + 11, ltv, ln);
    return 11 + ln;
}

size_t cpConfigQos(uint8_t* out, size_t cap, uint8_t aseId, const QosCfg& q)
{
    if (out == nullptr || aseId == 0 || cap < 18 || q.sduIntervalUs < 0xFF || q.sduIntervalUs > 0xFFFFF ||
        q.maxSdu > 0xFFF || q.pdUs > 0xFFFFFF) {
        return 0;
    }
    out[0] = (uint8_t)CpOp::ConfigQos;
    out[1] = 1;
    out[2] = aseId;
    out[3] = q.cigId;
    out[4] = q.cisId;
    put24(out + 5, q.sduIntervalUs);
    out[8] = q.framing;
    out[9] = q.phy;
    put16(out + 10, q.maxSdu);
    out[12] = q.rtn;
    put16(out + 13, q.maxLatencyMs);
    put24(out + 15, q.pdUs);
    return 18;
}

size_t cpEnable(uint8_t* out, size_t cap, uint8_t aseId, uint16_t contexts, int ccid)
{
    const bool withCcid = ccid >= 0 && ccid <= 0xFF;
    if (out == nullptr || aseId == 0 || cap < (withCcid ? 11u : 8u)) {
        return 0;
    }
    out[0] = (uint8_t)CpOp::Enable;
    out[1] = 1;
    out[2] = aseId;
    out[3] = withCcid ? 7 : 4;  // Metadata_Length
    out[4] = 0x03;              // LTV: length 3,
    out[5] = 0x02;              //      type Streaming_Audio_Contexts
    put16(out + 6, contexts);
    if (!withCcid) {
        return 8;
    }
    out[8]  = 0x02;  // LTV: length 2,
    out[9]  = 0x05;  //      type CCID_List (BAP 4.3.3)
    out[10] = (uint8_t)ccid;
    return 11;
}

size_t cpSimple(uint8_t* out, size_t cap, CpOp op, uint8_t aseId)
{
    if (out == nullptr || aseId == 0 || cap < 3 ||
        !(op == CpOp::RecvStartReady || op == CpOp::Disable || op == CpOp::RecvStopReady || op == CpOp::Release)) {
        return 0;
    }
    out[0] = (uint8_t)op;
    out[1] = 1;
    out[2] = aseId;
    return 3;
}

bool parseCpNotify(const uint8_t* d, size_t len, CpResult& out)
{
    out = CpResult{};
    if (d == nullptr || len < 2) {
        return false;
    }
    out.op         = d[0];
    const uint8_t n = d[1];
    // 0xFF = "unsupported opcode / bad length": then a single entry with ASE_ID 0
    if (len < 2u + 3u * (n == 0xFF ? 1u : n)) {
        return false;
    }
    const int cnt = n == 0xFF ? 1 : n;
    for (int i = 0; i < cnt && out.n < CpResult::kMax; i++) {
        out.items[out.n].aseId  = d[2 + i * 3];
        out.items[out.n].code   = d[3 + i * 3];
        out.items[out.n].reason = d[4 + i * 3];
        out.n++;
    }
    return true;
}

const char* cpCodeText(uint8_t code)
{
    switch (code) {
        case 0x00: return "OK";
        case 0x01: return "unsupported opcode";
        case 0x02: return "bad length";
        case 0x03: return "bad ASE_ID";
        case 0x04: return "bad ASE state machine";
        case 0x05: return "bad ASE direction";
        case 0x06: return "unsupported audio config";
        case 0x07: return "unsupported config value";
        case 0x08: return "rejected config value";
        case 0x09: return "bad config value";
        case 0x0A: return "unsupported metadata";
        case 0x0B: return "rejected metadata";
        case 0x0C: return "bad metadata";
        case 0x0D: return "no resources";
        case 0x0E: return "unspecified error";
        default: return "unknown code";
    }
}

/* ------------------------------------------ HCI CIG/CIS ------------------------------------------ */

size_t buildSetCigParams(uint8_t* out, size_t cap, const CigParams& p)
{
    if (out == nullptr || p.nCis < 1 || p.nCis > 2 || p.sduIntervalUs < 0xFF || p.sduIntervalUs > 0xFFFFF) {
        return 0;
    }
    const size_t n = 15 + 9 * (size_t)p.nCis;
    if (cap < n) {
        return 0;
    }
    out[0] = p.cigId;
    put24(out + 1, p.sduIntervalUs);
    put24(out + 4, p.sduIntervalUs);
    out[7] = 0;  // Worst_Case_SCA: 251-500 ppm (worst case, nothing is known about the C6 clock)
    out[8] = 0;  // Packing: sequential
    out[9] = 0;  // Framing: unframed
    put16(out + 10, p.latencyCtoPMs);
    put16(out + 12, p.latencyPtoCMs);
    out[14]    = (uint8_t)p.nCis;
    uint8_t* q = out + 15;
    for (int i = 0; i < p.nCis; i++) {
        const CisParams& c = p.cis[i];
        if (c.sduCtoP > 0xFFF || c.sduPtoC > 0xFFF) {
            return 0;
        }
        q[0] = c.cisId;
        put16(q + 1, c.sduCtoP);
        put16(q + 3, c.sduPtoC);
        q[5] = c.phyCtoP;
        q[6] = c.phyPtoC;
        q[7] = c.rtnCtoP;
        q[8] = c.rtnPtoC;
        q += 9;
    }
    return n;
}

size_t buildCreateCis(uint8_t* out, size_t cap, const uint16_t* cisHandles, const uint16_t* aclHandles, int n)
{
    if (out == nullptr || cisHandles == nullptr || aclHandles == nullptr || n < 1 || n > 2 || cap < 1 + 4u * n) {
        return 0;
    }
    out[0] = (uint8_t)n;
    for (int i = 0; i < n; i++) {
        put16(out + 1 + i * 4, cisHandles[i] & 0x0FFF);
        put16(out + 3 + i * 4, aclHandles[i] & 0x0FFF);
    }
    return 1 + 4u * n;
}

size_t buildSetupIsoPath(uint8_t* out, size_t cap, uint16_t cisHandle, bool dirOutput)
{
    if (out == nullptr || cap < 13) {
        return 0;
    }
    put16(out, cisHandle & 0x0FFF);
    out[2] = dirOutput ? 0x01 : 0x00;  // 0 = Input (host -> controller), 1 = Output (controller -> host)
    out[3] = 0x00;                     // Data_Path_ID: HCI
    out[4] = 0x03;                     // Codec_ID: transparent (the host encodes LC3)
    out[5] = out[6] = out[7] = out[8] = 0;
    put24(out + 9, 0);                 // Controller_Delay
    out[12] = 0;                       // Codec_Configuration_Length
    return 13;
}

size_t buildRemoveIsoPath(uint8_t* out, size_t cap, uint16_t cisHandle, bool input, bool output)
{
    if (out == nullptr || cap < 3 || (!input && !output)) {
        return 0;
    }
    put16(out, cisHandle & 0x0FFF);
    out[2] = (uint8_t)((input ? 0x01 : 0) | (output ? 0x02 : 0));  // bit 0 input, bit 1 output
    return 3;
}

bool parseCisEstablished(const uint8_t* p, size_t len, CisEstablished& out)
{
    out = CisEstablished{};
    if (p == nullptr || len < 28) {
        return false;
    }
    out.status         = p[0];
    out.handle         = get16(p + 1) & 0x0FFF;
    out.cigSyncDelayUs = get24(p + 3);
    out.cisSyncDelayUs = get24(p + 6);
    out.latCtoPUs      = get24(p + 9);
    out.latPtoCUs      = get24(p + 12);
    out.phyCtoP        = p[15];
    out.phyPtoC        = p[16];
    out.nse            = p[17];
    out.bnCtoP         = p[18];
    out.bnPtoC         = p[19];
    out.ftCtoP         = p[20];
    out.ftPtoC         = p[21];
    out.maxPduCtoP     = get16(p + 22);
    out.maxPduPtoC     = get16(p + 24);
    out.isoInterval    = get16(p + 26);
    return true;
}

bool parseReadBufferSizeV2(const uint8_t* ret, size_t len, IsoBuffers& out)
{
    out = IsoBuffers{};
    if (ret == nullptr || len < 6) {
        return false;
    }
    out.aclLen = get16(ret);
    out.aclNum = ret[2];
    out.isoLen = get16(ret + 3);
    out.isoNum = ret[5];
    return true;
}

bool findServiceInGroups(const uint8_t* rsp, size_t len, uint16_t uuid, uint16_t& start, uint16_t& end,
                         uint16_t& nextStart)
{
    nextStart = 0;
    if (len < 2 || rsp[0] != 0x11 || rsp[1] < 4) {
        return false;  // Error Response (Attribute Not Found = end of the database) or a bad response
    }
    const size_t item = rsp[1];
    bool found        = false;
    uint16_t lastEnd  = 0;
    for (size_t off = 2; off + item <= len; off += item) {
        const uint16_t s = (uint16_t)(rsp[off] | (rsp[off + 1] << 8));
        const uint16_t e = (uint16_t)(rsp[off + 2] | (rsp[off + 3] << 8));
        if (item == 6 && (uint16_t)(rsp[off + 4] | (rsp[off + 5] << 8)) == uuid && !found) {
            start = s;
            end   = e;
            found = true;
        }
        lastEnd = e;
    }
    if (lastEnd != 0 && lastEnd != 0xFFFF) {
        nextStart = (uint16_t)(lastEnd + 1);
    }
    return found;
}

void collectAscsHandles(const uint8_t* rsp, size_t len, AscsHandles& h, uint16_t& nextStart)
{
    nextStart = 0;
    if (len < 2 || rsp[0] != 0x09 || rsp[1] != 7) {
        return;  // Error Response or declarations with a 128-bit UUID (ASCS has only UUID16)
    }
    uint16_t lastDecl = 0;
    for (size_t off = 2; off + 7 <= len; off += 7) {
        lastDecl              = (uint16_t)(rsp[off] | (rsp[off + 1] << 8));
        const uint16_t valH   = (uint16_t)(rsp[off + 3] | (rsp[off + 4] << 8));
        const uint16_t chUuid = (uint16_t)(rsp[off + 5] | (rsp[off + 6] << 8));
        uint16_t* slots       = chUuid == 0x2BC4 ? h.sink : chUuid == 0x2BC5 ? h.src : nullptr;
        if (chUuid == 0x2BC6) {
            h.cp = valH;
        } else if (slots != nullptr) {
            uint16_t* free = slots[0] == 0 ? &slots[0] : slots[1] == 0 ? &slots[1] : nullptr;
            if (free != nullptr) {
                *free = valH;
            }
        }
    }
    if (lastDecl != 0 && lastDecl != 0xFFFF) {
        nextStart = (uint16_t)(lastDecl + 1);
    }
}

Channel channelFor(uint32_t loc, int n)
{
    if (n < 2) {
        return Channel::Mono;
    }
    return loc == 0x1 ? Channel::Left : loc == 0x2 ? Channel::Right : Channel::Mono;
}

void pickChannel(const int16_t* stereo, size_t frames, Channel ch, float gain, int16_t* out)
{
    for (size_t i = 0; i < frames; i++) {
        const float l = stereo[2 * i], r = stereo[2 * i + 1];
        const float v = (ch == Channel::Left ? l : ch == Channel::Right ? r : (l + r) * 0.5f) * gain;
        out[i]        = (int16_t)std::clamp(v, -32768.0f, 32767.0f);
    }
}

}  // namespace app::leaudio
