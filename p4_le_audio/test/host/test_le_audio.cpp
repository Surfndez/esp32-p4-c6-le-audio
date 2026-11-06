// Host tests: LE Audio (bt/le_audio_model.cpp) - HCI ISO frames per Core Vol 4 Part E 5.4.5, ISO buffer credits,
// PACS/ASCS per the specification, HCI CIG/CIS parameters - and LC3 (liblc3 v1.1.3): frame sizes and the
// encoder -> decoder path for the configurations used by Tab5 (48 kHz / 120 B downstream, 16 kHz / 40 B mic).

#include <bt/le_audio_model.h>

#include <lc3.h>

#include <cmath>
#include <cstdio>
#include <cstring>
#include <vector>

using namespace app::leaudio;

namespace {

int g_ok = 0, g_fail = 0;

void expect(bool cond, const char* what)
{
    if (cond) {
        g_ok++;
    } else {
        g_fail++;
        std::printf("  FAIL %s\n", what);
    }
}

void testIsoTx()
{
    std::printf("HCI ISO: transmit\n");
    uint8_t sdu[120];
    for (int i = 0; i < 120; i++) {
        sdu[i] = (uint8_t)i;
    }
    uint8_t pkt[300];
    uint16_t off = 0;
    // SDU of 120 B in one packet, no timestamp: PB = 10 (Complete), 4 B header + 4 B SDU header
    size_t n = buildIsoPacket(pkt, sizeof(pkt), 0x0060, 0x1234, false, 0, sdu, 120, 251, off);
    expect(n == 1 + 4 + 4 + 120, "Complete packet: length");
    expect(pkt[0] == 0x05, "typ H4 ISO");
    expect(pkt[1] == 0x60 && pkt[2] == 0x20, "handle 0x060 + PB Complete (0x2000), TS = 0");
    expect(pkt[3] == 124 && pkt[4] == 0, "ISO_Data_Load_Length = 124");
    expect(pkt[5] == 0x34 && pkt[6] == 0x12, "sequence number LE");
    expect(pkt[7] == 120 && pkt[8] == 0, "ISO_SDU_Length = 120, PSF = 0");
    expect(std::memcmp(pkt + 9, sdu, 120) == 0 && off == 120, "SDU data and offset");

    // With a timestamp: TS_Flag (bit 14), 4 B timestamp before the sequence number
    off = 0;
    n   = buildIsoPacket(pkt, sizeof(pkt), 0x0061, 7, true, 0x11223344, sdu, 40, 251, off);
    expect(n == 1 + 4 + 8 + 40, "packet with timestamp: length");
    expect(pkt[2] == (0x20 | 0x40), "PB Complete + TS_Flag");
    expect(pkt[5] == 0x44 && pkt[8] == 0x11, "timestamp LE");

    // Fragmentation: ISO_Data_Packet_Length 60 -> 56 B of SDU in the first, then 60 and the remaining 4 B
    off           = 0;
    size_t parts  = 0;
    uint8_t pbs[4] = {};
    while (off < 120 && parts < 4) {
        n = buildIsoPacket(pkt, sizeof(pkt), 0x0060, 1, false, 0, sdu, 120, 60, off);
        if (n == 0) {
            break;
        }
        pbs[parts++] = (uint8_t)((pkt[2] >> 4) & 0x03);
    }
    expect(parts == 3 && off == 120, "fragmentation into 3 packets");
    expect(pbs[0] == 0 && pbs[1] == 1 && pbs[2] == 3, "PB: First, Continuation, Last");

    // Invalid arguments
    off = 0;
    expect(buildIsoPacket(pkt, 10, 0x0060, 1, false, 0, sdu, 120, 251, off) == 0, "buffer too small = 0");
    off = 0;
    expect(buildIsoPacket(pkt, sizeof(pkt), 0x0F00, 1, false, 0, sdu, 120, 251, off) == 0, "handle > 0x0EFF = 0");
    off = 0;
    expect(buildIsoPacket(pkt, sizeof(pkt), 0x0060, 1, true, 0, sdu, 120, 8, off) == 0,
           "maxDataLen with no room for data = 0");
}

void testIsoRx()
{
    std::printf("HCI ISO: receive and reassembly\n");
    IsoReassembler r;
    IsoSdu s;
    // Complete z TS, PSF = 0, SDU 3 B
    const uint8_t a[] = {0x05, 0x61, 0x60, 15, 0x00, 0x10, 0x27, 0, 0, 0x05, 0x00, 0x03, 0x00, 0xAA, 0xBB, 0xCC, 0, 0, 0};
    // ISO_Data_Load_Length = 4 (TS) + 4 + 3 = 11 -> this packet claims 15: wrong length here, correct one below
    expect(r.feed(a, sizeof(a), s) == IsoReassembler::Result::Error, "header length does not match the packet");
    const uint8_t b[] = {0x05, 0x61, 0x60, 11, 0x00, 0x10, 0x27, 0, 0, 0x05, 0x00, 0x03, 0x00, 0xAA, 0xBB, 0xCC};
    expect(r.feed(b, sizeof(b), s) == IsoReassembler::Result::Sdu, "Complete z TS");
    expect(s.handle == 0x061 && s.hasTs && s.ts == 10000 && s.seq == 5 && s.len == 3 && s.data[2] == 0xCC &&
               s.status == IsoStatus::Valid,
           "SDU fields");
    // Lost SDU: PSF = 10, length 0
    const uint8_t c[] = {0x05, 0x61, 0x20, 4, 0x00, 0x06, 0x00, 0x00, 0x80};
    expect(r.feed(c, sizeof(c), s) == IsoReassembler::Result::Sdu && s.status == IsoStatus::Lost && s.len == 0,
           "lost SDU (PSF = 2)");
    // Fragments: First (2 B of 5), Last (3 B)
    const uint8_t f1[] = {0x05, 0x61, 0x00, 6, 0x00, 0x07, 0x00, 0x05, 0x40, 1, 2};
    const uint8_t f2[] = {0x05, 0x61, 0x30, 3, 0x00, 3, 4, 5};
    expect(r.feed(f1, sizeof(f1), s) == IsoReassembler::Result::None, "First: waits for the rest");
    expect(r.feed(f2, sizeof(f2), s) == IsoReassembler::Result::Sdu && s.len == 5 && s.data[4] == 5 &&
               s.status == IsoStatus::PossiblyInvalid && s.seq == 7,
           "First + Last = SDU 5 B, PSF = 1");
    // Continuation without a start, and an SDU longer than announced
    expect(r.feed(f2, sizeof(f2), s) == IsoReassembler::Result::Error, "Last without First = error");
    const uint8_t big[] = {0x05, 0x61, 0x20, 7, 0x00, 0x01, 0x00, 0x02, 0x00, 9, 9, 9};
    expect(r.feed(big, sizeof(big), s) == IsoReassembler::Result::Error, "data longer than ISO_SDU_Length");
    const uint8_t huge[] = {0x05, 0x61, 0x00, 4, 0x00, 0x01, 0x00, 0xFF, 0x0F};
    expect(r.feed(huge, sizeof(huge), s) == IsoReassembler::Result::Error, "SDU of 4095 B > buffer = error");
    // Two handles reassembled independently
    const uint8_t g1[] = {0x05, 0x62, 0x00, 5, 0x00, 0x01, 0x00, 0x02, 0x00, 0x11};
    const uint8_t h1[] = {0x05, 0x63, 0x20, 5, 0x00, 0x01, 0x00, 0x01, 0x00, 0x22};
    const uint8_t g2[] = {0x05, 0x62, 0x30, 1, 0x00, 0x12};
    expect(r.feed(g1, sizeof(g1), s) == IsoReassembler::Result::None, "handle A: First");
    expect(r.feed(h1, sizeof(h1), s) == IsoReassembler::Result::Sdu && s.handle == 0x063, "handle B: Complete");
    expect(r.feed(g2, sizeof(g2), s) == IsoReassembler::Result::Sdu && s.handle == 0x062 && s.len == 2 &&
               s.data[1] == 0x12,
           "handle A: Last after packet B");
    expect(r.feed(b, 3, s) == IsoReassembler::Result::Error, "truncated packet");
}

void testCredits()
{
    std::printf("ISO credits\n");
    IsoCredits c;
    c.reset(3);
    expect(c.take(0x60) && c.take(0x61) && c.take(0x60), "3 buffers taken");
    expect(!c.take(0x60), "pool empty");
    expect(c.outstanding(0x60) == 2 && c.outstanding(0x61) == 1, "accounting per handle");
    c.completed(0x60, 5);  // the controller returns more than it received
    expect(c.available() == 2 && c.outstanding(0x60) == 0 && c.bogus() == 3, "excess ignored");
    c.completed(0x40, 2);  // NOCP for ACL - not our pool
    expect(c.available() == 2 && c.bogus() == 5, "foreign handle ignored");
    c.dropHandle(0x61);
    expect(c.available() == 3 && c.outstanding(0x61) == 0, "CIS disconnect returns the buffers");
    // Number Of Completed Packets: interleaved pairs
    const uint8_t ev[] = {2, 0x60, 0x00, 0x01, 0x00, 0x61, 0x00, 0x02, 0x00};
    int sum = 0, cnt = 0;
    expect(forEachCompleted(ev, sizeof(ev), [&](uint16_t h, uint16_t n) { sum += h + n; cnt++; }) && cnt == 2 &&
               sum == 0x60 + 1 + 0x61 + 2,
           "NOCP: two pairs");
    expect(!forEachCompleted(ev, 6, [](uint16_t, uint16_t) {}), "truncated NOCP = false");

    // Lost NOCP (measured on Tab5: C6 OTA over SDIO interrupted, CIS1 got 68 % of SDUs until the stream restarted):
    // a credit not returned for longer than lostUs goes back to the pool, a late NOCP does not raise it above total
    IsoCredits k;
    k.reset(4);
    expect(k.take(0x60, 1000) && k.take(0x60, 2000) && k.take(0x60, 3000) && k.take(0x61, 3000), "4 taken, timed");
    k.completed(0x60, 1);  // the oldest (1000) came back
    expect(k.sweep(400000, 500000) == 0 && k.available() == 1, "nothing is recovered before lostUs");
    expect(k.sweep(502500, 500000) == 1 && k.available() == 2 && k.outstanding(0x60) == 1 && k.recovered() == 1,
           "credit 2000 without NOCP returns after 500 ms");
    expect(k.sweep(503500, 500000) == 2 && k.available() == 4 && k.recovered() == 3, "the next lost ones return");
    k.completed(0x60, 1);  // late NOCP for an already recovered credit
    expect(k.available() == 4 && k.bogus() == 1, "late NOCP does not raise above total");
    expect(k.take(0x60, 600000) && k.sweep(600100, 500000) == 0, "a fresh credit is not lost");
}

// SDU queue waiting for credit (txq switch): FIFO with sequence number, deadline, a full queue drops the oldest.
void testTxQueue()
{
    std::printf("SDU queue waiting for credit\n");
    IsoTxQueue q;
    uint32_t dropped = 0;
    const uint8_t a[3] = {1, 2, 3}, b[2] = {9, 8};
    expect(q.front() == nullptr && q.size() == 0, "empty");
    expect(q.push(10, a, 3, 1000, dropped) && q.push(11, b, 2, 2000, dropped), "two SDUs");
    expect(q.front() != nullptr && q.front()->seq == 10 && q.front()->len == 3 && q.front()->data[2] == 3,
           "FIFO: the older one first, with its data");
    expect(q.expire(1000) == 0 && q.size() == 2, "exactly at the deadline it stays");
    expect(q.expire(1500) == 1 && q.front()->seq == 11 && q.front()->data[0] == 9, "past deadline only older drops");
    q.pop();
    expect(q.size() == 0 && q.expire(1 << 30) == 0, "pop down to empty");
    std::vector<uint8_t> big(IsoTxQueue::kMaxSdu + 1, 0);
    expect(!q.push(1, big.data(), (uint16_t)big.size(), 0, dropped) && q.size() == 0, "oversized SDU rejected");
    for (uint16_t i = 0; i < IsoTxQueue::kCap + 2; i++) {
        q.push((uint16_t)(100 + i), a, 3, 5000, dropped);
    }
    expect(q.size() == IsoTxQueue::kCap && dropped == 2 && q.front()->seq == 102, "a full queue drops the oldest");
    // ring wrap-around: order still ascending
    uint16_t last = 0;
    bool order   = true;
    while (const IsoTxQueue::Sdu* f = q.front()) {
        order = order && f->seq > last;
        last  = f->seq;
        q.pop();
    }
    expect(order && last == 100 + IsoTxQueue::kCap + 1, "FIFO after wrap-around");
    q.push(7, a, 3, 0, dropped);
    q.clear();
    expect(q.size() == 0 && q.front() == nullptr, "clear");
}

void testPacs()
{
    std::printf("PACS and configuration choice\n");
    // LC3 record: 16/24/32/48 kHz, 7.5+10 ms, 1 channel, 26-155 B, 1 frame/SDU; 4 B metadata
    const uint8_t pac[] = {0x01, 0x06, 0x00, 0x00, 0x00, 0x00, 0x10, 0x03, 0x01, 0xB4, 0x00, 0x02,
                           0x02, 0x03, 0x02, 0x03, 0x01, 0x05, 0x04, 0x1A, 0x00, 0x9B, 0x00, 0x04,
                           0x03, 0x01, 0x06, 0x00};
    PacSet ps;
    expect(parsePac(pac, sizeof(pac), ps) && ps.n == 1, "PAC: one LC3 record");
    expect(ps.caps[0].freqMask == 0x00B4 && ps.caps[0].durMask == 0x03 && ps.caps[0].octMin == 26 &&
               ps.caps[0].octMax == 155,
           "PAC: LTV fields");
    PacSet cut;
    expect(!parsePac(pac, sizeof(pac) - 3, cut), "truncated PAC = false");
    CodecCfg sink, src;
    expect(chooseSinkConfig(ps, 0x01, sink) && sink.rateHz == 48000 && sink.octets == 120 && sink.location == 1,
           "earbuds: 48_4 (120 B)");
    expect(chooseSourceConfig(ps, 0x01, src) && src.rateHz == 16000 && src.octets == 40, "microphone: 16_2 (40 B)");
    // Upper limit of 100 B -> 48_2
    uint8_t pac2[sizeof(pac)];
    std::memcpy(pac2, pac, sizeof(pac));
    pac2[21] = 100;
    expect(parsePac(pac2, sizeof(pac2), ps) && chooseSinkConfig(ps, 0x02, sink) && sink.octets == 100,
           "earbuds: 48_2 with max 100 B");
    uint8_t ltv[32];
    const size_t n = buildCodecCfgLtv(sink, ltv, sizeof(ltv));
    const uint8_t want[] = {0x02, 0x01, 0x08, 0x02, 0x02, 0x01, 0x05, 0x03, 0x02, 0, 0, 0, 0x03, 0x04, 100, 0};
    expect(n == sizeof(want) && std::memcmp(ltv, want, n) == 0, "Codec_Specific_Configuration LTV");
    // Without 10 ms -> no configuration
    pac2[13] = 0x01;
    expect(parsePac(pac2, sizeof(pac2), ps) && !chooseSinkConfig(ps, 1, sink), "no 10 ms = no configuration");
}

void testAscs()
{
    std::printf("ASCS: states, control point\n");
    // Codec Configured: unframed, PHY 1M|2M, RTN 2, 20 ms, PD 10..40 ms, pref 20..30 ms, LC3, 3 B configuration
    const uint8_t cc[] = {0x01, 0x01, 0x00, 0x03, 0x02, 0x14, 0x00, 0x10, 0x27, 0x00, 0x40, 0x9C, 0x00,
                          0x20, 0x4E, 0x00, 0x30, 0x75, 0x00, 0x06, 0, 0, 0, 0, 0x03, 0x02, 0x01, 0x08};
    AseInfo a;
    expect(parseAse(cc, sizeof(cc), a) && a.state == AseState::CodecConfigured && a.id == 1, "ASE Codec Configured");
    expect(a.pref.phyMask == 3 && a.pref.rtn == 2 && a.pref.maxLatencyMs == 20 && a.pref.pdMinUs == 10000 &&
               a.pref.pdMaxUs == 40000 && a.pref.prefPdMinUs == 20000,
           "server QoS preferences");
    expect(!parseAse(cc, sizeof(cc) - 1, a), "truncated Codec Configured = false");
    const uint8_t st[] = {0x03, 0x04, 0x01, 0x00, 0x04, 0x03, 0x02, 0x04, 0x00};
    expect(parseAse(st, sizeof(st), a) && a.state == AseState::Streaming && a.cigId == 1 && a.cisId == 0,
           "ASE Streaming with metadata");
    const uint8_t bad[] = {0x03, 0x09};
    expect(!parseAse(bad, sizeof(bad), a), "unknown state = false");

    expect(aseTransitionAllowed(true, AseState::Idle, AseState::CodecConfigured), "Idle -> Codec");
    expect(!aseTransitionAllowed(true, AseState::Idle, AseState::Streaming), "Idle -> Streaming forbidden");
    expect(aseTransitionAllowed(true, AseState::Streaming, AseState::QosConfigured), "Sink: Disable -> QoS");
    expect(!aseTransitionAllowed(true, AseState::Streaming, AseState::Disabling), "Sink has no Disabling");
    expect(aseTransitionAllowed(false, AseState::Streaming, AseState::Disabling), "Source: Disable -> Disabling");
    expect(aseTransitionAllowed(false, AseState::Disabling, AseState::QosConfigured), "Source: Stop Ready -> QoS");
    expect(aseTransitionAllowed(true, AseState::Releasing, AseState::CodecConfigured), "Releasing -> Codec (cache)");
    // ASCS: on CIS loss the server itself moves the ASE (Source too) to QoS Configured
    expect(aseTransitionAllowed(false, AseState::Streaming, AseState::QosConfigured), "Source: CIS loss -> QoS");
    expect(aseTransitionAllowed(false, AseState::Enabling, AseState::QosConfigured), "Source Enabling: CIS loss");

    CodecCfg c;
    c.rateHz = 48000;
    c.octets = 120;
    QosCfg q;
    AsePref p;
    p.phyMask = 3, p.rtn = 2, p.maxLatencyMs = 20, p.pdMinUs = 10000, p.pdMaxUs = 40000, p.prefPdMinUs = 20000;
    expect(chooseQos(p, c, 1, 0, q) && q.phy == 2 && q.maxSdu == 120 && q.pdUs == 20000 && q.rtn == 2 &&
               q.maxLatencyMs == 20,
           "QoS from preferences");
    p.prefPdMinUs = 50000;  // outside the range -> range minimum
    expect(chooseQos(p, c, 1, 0, q) && q.pdUs == 10000, "PD outside the range -> minimum");
    p.pdMinUs = 0, p.prefPdMinUs = 0;  // a minimum of 0 is legal; we take 40 ms (BAP: the server supports it)
    expect(chooseQos(p, c, 1, 0, q) && q.pdUs == 40000, "PD min 0 -> 40 ms from the range");
    p.pdMaxUs = 25000;
    expect(chooseQos(p, c, 1, 0, q) && q.pdUs == 25000, "PD min 0, max < 40 ms -> max");
    p.pdMaxUs = 0;
    expect(!chooseQos(p, c, 1, 0, q), "no PD range = no QoS");
    p.pdMinUs = 10000, p.pdMaxUs = 40000;
    p.framing = 1;
    expect(!chooseQos(p, c, 1, 0, q), "framed only = no QoS");

    // Enable: Streaming_Audio_Contexts must be within Available Audio Contexts (PACS 0x2BCD)
    expect(pickStreamingContext(0x0004, 0x0006) == 0x0004, "Media available -> Media");
    expect(pickStreamingContext(0x0004, 0x0003) == 0x0001, "Media unavailable -> Unspecified");
    expect(pickStreamingContext(0x0002, 0x0204) == 0x0004, "no wanted/Unspecified -> lowest available");
    expect(pickStreamingContext(0x0004, 0x0000) == 0x0004, "unknown / empty -> the wanted one");

    // One CIS carries both directions and the C6 emulation has one PHY per link: differing preferences -> both 1M
    QosCfg qs, qm;
    qs.phy = 0x02, qm.phy = 0x01;
    unifyCisPhy(qs, qm);
    expect(qs.phy == 0x01 && qm.phy == 0x01, "PHY Sink 2M + Source 1M -> both 1M");
    qs.phy = 0x02, qm.phy = 0x02;
    unifyCisPhy(qs, qm);
    expect(qs.phy == 0x02 && qm.phy == 0x02, "matching 2M stays");

    uint8_t buf[64];
    size_t n = cpConfigQos(buf, sizeof(buf), 1, QosCfg{1, 0, 10000, 0, 2, 120, 2, 20, 40000});
    const uint8_t wantQ[] = {0x02, 1, 1, 1, 0, 0x10, 0x27, 0x00, 0, 2, 120, 0, 2, 20, 0, 0x40, 0x9C, 0x00};
    expect(n == sizeof(wantQ) && std::memcmp(buf, wantQ, n) == 0, "Config QoS: bytes");
    n = cpEnable(buf, sizeof(buf), 3, 0x0004);
    const uint8_t wantE[] = {0x03, 1, 3, 4, 0x03, 0x02, 0x04, 0x00};
    expect(n == sizeof(wantE) && std::memcmp(buf, wantE, n) == 0, "Enable: Streaming_Audio_Contexts = Media");
    // CCID_List (BAP 4.3.3, metadata 0x05): the stream is controlled by our GMCS - earbuds send gestures there (MCP)
    n = cpEnable(buf, sizeof(buf), 3, 0x0004, 0x01);
    const uint8_t wantEc[] = {0x03, 1, 3, 7, 0x03, 0x02, 0x04, 0x00, 0x02, 0x05, 0x01};
    expect(n == sizeof(wantEc) && std::memcmp(buf, wantEc, n) == 0, "Enable: + CCID_List with the GMCS CCID");
    expect(cpEnable(buf, 10, 3, 0x0004, 0x01) == 0, "Enable z CCID: buffer too small");
    c.location = 1;
    n          = cpConfigCodec(buf, sizeof(buf), 1, c, 0x02, 0x02);
    expect(n == 11 + 16 && buf[0] == 1 && buf[5] == 0x06 && buf[10] == 16, "Config Codec: header and LTV length");
    expect(cpSimple(buf, sizeof(buf), CpOp::RecvStartReady, 3) == 3 && buf[0] == 4, "Receiver Start Ready");
    expect(cpSimple(buf, sizeof(buf), CpOp::Enable, 3) == 0, "cpSimple does not build Enable");

    const uint8_t ntf[] = {0x02, 0x01, 0x01, 0x08, 0x05};
    CpResult r;
    expect(parseCpNotify(ntf, sizeof(ntf), r) && r.op == 2 && r.n == 1 && r.items[0].code == 0x08 &&
               r.items[0].reason == 0x05,
           "CP response: rejected value (PD)");
    const uint8_t ntfBad[] = {0x02, 0x02, 0x01, 0x00, 0x00};
    expect(!parseCpNotify(ntfBad, sizeof(ntfBad), r), "truncated CP response = false");
    const uint8_t ntfFf[] = {0x09, 0xFF, 0x00, 0x01, 0x00};
    expect(parseCpNotify(ntfFf, sizeof(ntfFf), r) && r.n == 1 && r.items[0].code == 1, "CP: unsupported opcode");
}

void testHci()
{
    std::printf("HCI CIG/CIS\n");
    CigParams p;
    p.nCis   = 1;
    p.cis[0] = CisParams{0, 120, 40, 2, 2, 2, 2};
    uint8_t b[64];
    size_t n = buildSetCigParams(b, sizeof(b), p);
    const uint8_t want[] = {1, 0x10, 0x27, 0, 0x10, 0x27, 0, 0, 0, 0, 20, 0, 20, 0, 1, 0, 120, 0, 40, 0, 2, 2, 2, 2};
    expect(n == sizeof(want) && std::memcmp(b, want, n) == 0, "LE Set CIG Parameters: bytes");
    p.nCis = 3;
    expect(buildSetCigParams(b, sizeof(b), p) == 0, "more than 2 CIS = 0");

    // LE Remote Connection Parameter Request (0x3E/0x06) - ACL interval a multiple of 10 ms (CIS emulation in C6)
    {
        const uint8_t ev[] = {0x06, 0x01, 0x00, 24, 0, 40, 0, 0, 0, 0x90, 0x01};  // 30..50 ms, supervision 4 s
        HciCmd r           = connParamRequestReply(ev, sizeof(ev));
        const uint8_t want[] = {0x01, 0x00, 24, 0, 24, 0, 0, 0, 0x90, 0x01, 0, 0, 0, 0};
        expect(r.opcode == 0x2020 && r.len == sizeof(want) && std::memcmp(r.p, want, r.len) == 0,
               "30..50 ms -> 30 ms (Reply 0x2020)");
        const uint8_t ev2[] = {0x06, 0x01, 0x00, 6, 0, 12, 0, 2, 0, 0x2C, 0x01};  // 7,5..15 ms, latency 2
        r = connParamRequestReply(ev2, sizeof(ev2));
        expect(r.opcode == 0x2020 && r.p[2] == 8 && r.p[4] == 8 && r.p[6] == 2 && r.p[8] == 0x2C,
               "7,5..15 ms -> 10 ms, latency and supervision as requested");
        const uint8_t ev3[] = {0x06, 0x01, 0x00, 36, 0, 0x90, 0x01, 0, 0, 0x90, 0x01};  // 45..500 ms
        r = connParamRequestReply(ev3, sizeof(ev3));
        expect(r.opcode == 0x2020 && r.p[2] == 40 && r.p[4] == 40, "45..500 ms -> nearest 30 ms multiple: 50 ms");
        const uint8_t ev4[] = {0x06, 0x02, 0x00, 6, 0, 6, 0, 0, 0, 0x90, 0x01};  // 7.5 ms only
        r = connParamRequestReply(ev4, sizeof(ev4));
        const uint8_t neg[] = {0x02, 0x00, 0x3B};
        expect(r.opcode == 0x2021 && r.len == 3 && std::memcmp(r.p, neg, 3) == 0, "no multiple of 10 ms -> 0x3B");
        expect(connParamRequestReply(ev, sizeof(ev) - 1).opcode == 0, "truncated event = nothing");
        expect(aclIntervalCisFriendly(kAclIntervalUnits) && !aclIntervalCisFriendly(36) &&
                   !aclIntervalCisFriendly(0),
               "ACL interval: 30 ms yes, 45 ms no");
    }
    const uint16_t cis[] = {0x60, 0x61}, acl[] = {0x01, 0x02};
    n = buildCreateCis(b, sizeof(b), cis, acl, 2);
    const uint8_t wantC[] = {2, 0x60, 0, 0x01, 0, 0x61, 0, 0x02, 0};
    expect(n == sizeof(wantC) && std::memcmp(b, wantC, n) == 0, "LE Create CIS");
    n = buildSetupIsoPath(b, sizeof(b), 0x60, true);
    expect(n == 13 && b[2] == 1 && b[3] == 0 && b[4] == 0x03, "Setup ISO Data Path: output, HCI, transparent");
    const uint8_t est[28] = {0x00, 0x60, 0x00, 0x10, 0x27, 0, 0x10, 0x27, 0, 0x20, 0x4E, 0, 0x20, 0x4E,
                             0,    2,    2,    1,    1,    1, 1,    1,    120, 0,   40,   0, 8,    0};
    CisEstablished e;
    expect(parseCisEstablished(est, sizeof(est), e) && e.status == 0 && e.handle == 0x60 && e.latCtoPUs == 20000 &&
               e.maxPduCtoP == 120 && e.isoInterval == 8,
           "LE CIS Established");
    expect(!parseCisEstablished(est, 27, e), "CIS Established truncated = false");
    const uint8_t rb[] = {0xFB, 0x00, 4, 0xFB, 0x00, 8};
    IsoBuffers ib;
    expect(parseReadBufferSizeV2(rb, sizeof(rb), ib) && ib.isoLen == 251 && ib.isoNum == 8, "Read Buffer Size v2");
}

// SNR [dB] of the decoder output against the input shifted by the codec delay (computed after the warm-up)
double lc3RoundTripSnr(int rate, int octets)
{
    const int ns    = lc3_frame_samples(10000, rate);
    const int delay = lc3_delay_samples(10000, rate);
    std::vector<uint8_t> encMem(lc3_encoder_size(10000, rate)), decMem(lc3_decoder_size(10000, rate));
    lc3_encoder_t enc = lc3_setup_encoder(10000, rate, 0, encMem.data());
    lc3_decoder_t dec = lc3_setup_decoder(10000, rate, 0, decMem.data());
    if (enc == nullptr || dec == nullptr) {
        return -100;
    }
    const int frames = 30;
    std::vector<int16_t> in((size_t)ns * frames), out((size_t)ns * frames);
    for (size_t i = 0; i < in.size(); i++) {
        in[i] = (int16_t)(8000 * std::sin(2 * 3.14159265 * 440.0 * (double)i / rate));
    }
    std::vector<uint8_t> frame((size_t)octets);
    for (int f = 0; f < frames; f++) {
        if (lc3_encode(enc, LC3_PCM_FORMAT_S16, &in[(size_t)f * ns], 1, octets, frame.data()) != 0 ||
            lc3_decode(dec, frame.data(), octets, LC3_PCM_FORMAT_S16, &out[(size_t)f * ns], 1) != 0) {
            return -100;
        }
    }
    double sig = 0, err = 0;
    for (size_t i = (size_t)ns * 5; i < out.size(); i++) {
        const double ref = in[i - (size_t)delay];
        sig += ref * ref;
        err += (out[i] - ref) * (out[i] - ref);
    }
    return 10 * std::log10(sig / (err + 1e-9));
}

void testLc3()
{
    std::printf("LC3 (liblc3)\n");
    expect(lc3FrameSamples(48000, 10000) == lc3_frame_samples(10000, 48000) && lc3FrameSamples(48000, 10000) == 480,
           "48 kHz / 10 ms = 480 samples (matches liblc3)");
    expect(lc3FrameSamples(16000, 10000) == lc3_frame_samples(10000, 16000), "16 kHz / 10 ms matches liblc3");
    expect(lc3FrameSamples(44100, 10000) == 0, "44.1 kHz unsupported");
    expect(lc3_resolve_bitrate(10000, 120) == 96000 && lc3_resolve_bitrate(10000, 40) == 32000,
           "120 B = 96 kb/s, 40 B = 32 kb/s");
    const double s48 = lc3RoundTripSnr(48000, 120);
    const double s16 = lc3RoundTripSnr(16000, 40);
    std::printf("  SNR 48 kHz/120 B: %.1f dB, 16 kHz/40 B: %.1f dB\n", s48, s16);
    expect(s48 > 20, "48 kHz / 120 B: encoder -> decoder reproduces a sine (SNR > 20 dB)");
    expect(s16 > 15, "16 kHz / 40 B: encoder -> decoder reproduces a sine (SNR > 15 dB)");
}

// Second earbud of the set: PACS/ASCS discovery (ATT responses as from Buds3 Pro: PACS 78-96, ASCS 65-77), stereo
void testStereo()
{
    std::printf("Stereo: discovering the second earbud and channels\n");
    // Read By Group Type Response: length 6 (handle, end, UUID16)
    const uint8_t groups[] = {0x11, 0x06, 0x01, 0x00, 0x07, 0x00, 0x00, 0x18, 0x41, 0x00, 0x4D, 0x00, 0x4E, 0x18,
                              0x4E, 0x00, 0x60, 0x00, 0x50, 0x18};
    uint16_t s = 0, e = 0, next = 0;
    expect(findServiceInGroups(groups, sizeof(groups), 0x184E, s, e, next) && s == 0x41 && e == 0x4D && next == 0x61,
           "ASCS 0x41-0x4D, next start after the last group");
    expect(findServiceInGroups(groups, sizeof(groups), 0x1850, s, e, next) && s == 0x4E && e == 0x60,
           "PACS 0x4E-0x60");
    expect(!findServiceInGroups(groups, sizeof(groups), 0x1846, s, e, next) && next == 0x61,
           "no CSIS in this response - keep searching from 0x61");
    const uint8_t last[] = {0x11, 0x06, 0xF0, 0x00, 0xFF, 0xFF, 0x46, 0x18};
    expect(findServiceInGroups(last, sizeof(last), 0x1846, s, e, next) && e == 0xFFFF && next == 0,
           "a group up to 0xFFFF ends the search");
    const uint8_t uuid128[] = {0x11, 0x14, 0x33, 0x00, 0x3A, 0x00, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16};
    expect(!findServiceInGroups(uuid128, sizeof(uuid128), 0x184E, s, e, next) && next == 0x3B,
           "128-bit service skipped, next start moves on");
    const uint8_t bad[] = {0x11, 0x06, 0x01, 0x00};
    expect(!findServiceInGroups(bad, sizeof(bad), 0x184E, s, e, next) && next == 0, "truncated response - end");
    const uint8_t err[] = {0x01, 0x10, 0x01, 0x00, 0x0A};
    expect(!findServiceInGroups(err, sizeof(err), 0x184E, s, e, next) && next == 0, "Error Response - end");

    // Read By Type Response for 0x2803: length 7 (declaration handle, properties, value handle, UUID16)
    const uint8_t chars[] = {0x09, 0x07, 0x42, 0x00, 0x12, 0x43, 0x00, 0xC4, 0x2B, 0x45, 0x00, 0x12, 0x46, 0x00, 0xC4,
                             0x2B, 0x48, 0x00, 0x12, 0x49, 0x00, 0xC5, 0x2B, 0x4B, 0x00, 0x1C, 0x4C, 0x00, 0xC6, 0x2B};
    AscsHandles h;
    collectAscsHandles(chars, sizeof(chars), h, next);
    expect(h.sink[0] == 0x43 && h.sink[1] == 0x46 && h.src[0] == 0x49 && h.src[1] == 0 && h.cp == 0x4C && next == 0x4C,
           "Sink ASE x2, Source ASE, ASE CP; next start after the last declaration");
    const uint8_t more[] = {0x09, 0x07, 0x4E, 0x00, 0x12, 0x4F, 0x00, 0xC5, 0x2B};
    collectAscsHandles(more, sizeof(more), h, next);
    expect(h.src[0] == 0x49 && h.src[1] == 0x4F && h.sink[0] == 0x43, "a further response adds the second Source ASE");
    AscsHandles hb;
    collectAscsHandles(err, sizeof(err), hb, next);
    expect(next == 0 && hb.cp == 0, "Error Response (end of attributes) - nothing and end");

    expect(channelFor(0x1, 2) == Channel::Left && channelFor(0x2, 2) == Channel::Right, "two earbuds: L / R");
    expect(channelFor(0x1, 1) == Channel::Mono && channelFor(0x2, 1) == Channel::Mono, "one earbud: mono");
    expect(channelFor(0x4, 2) == Channel::Mono, "different location with two: mono");
    const int16_t st[6] = {100, 300, -200, -400, 32000, 32000};
    int16_t o[3];
    pickChannel(st, 3, Channel::Mono, 1.0f, o);
    expect(o[0] == 200 && o[1] == -300 && o[2] == 32000, "mono = (L + R) / 2");
    pickChannel(st, 3, Channel::Left, 0.5f, o);
    expect(o[0] == 50 && o[1] == -100 && o[2] == 16000, "left channel with volume");
    pickChannel(st, 3, Channel::Right, 2.0f, o);
    expect(o[0] == 600 && o[1] == -800 && o[2] == 32767, "right channel, clipping to int16");
}

}  // namespace

int main()
{
    testIsoTx();
    testIsoRx();
    testCredits();
    testTxQueue();
    testPacs();
    testAscs();
    testHci();
    testLc3();
    testStereo();
    std::printf("le_audio: %d OK, %d FAIL\n", g_ok, g_fail);
    return g_fail == 0 ? 0 : 1;
}
