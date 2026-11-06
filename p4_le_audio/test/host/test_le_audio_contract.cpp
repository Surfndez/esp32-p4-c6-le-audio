// Host tests: the LE Audio HCI contract between the P4 and the C6. The P4 side (bt/le_audio_model.cpp: building
// commands and ISO packets, response parsers; ordering and response handling as in the firmware's HALs) and the
// CIS engine of the C6 image (le_audio/cis_ll) were written separately from Core - here P4 bytes go to the real engine,
// and its events and ISO packets return to the P4 parsers. The radio and the earbuds (Peripheral) are scripted as in
// le_audio/cis_ll/test/test_engine.c (NSE = BN = FT = 1, encryption data from Core Vol 6 Part C 1).
// A mismatch here would fail the first on-device test on Tab5 and cost a C6 or P4 flashing cycle.

#include <bt/le_audio_model.h>

#include <cis_crypto.h>
#include <cis_engine.h>
#include <cis_proto.h>

#include <cstdio>
#include <cstring>
#include <vector>

extern "C" void aes_ref_platform(void* ctx, const uint8_t key[16], const uint8_t in[16], uint8_t out[16]);

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

void expectEq(long got, long want, const char* what)
{
    if (got == want) {
        g_ok++;
    } else {
        g_fail++;
        std::printf("  FAIL %s: got %ld, expected %ld\n", what, got, want);
    }
}

constexpr uint16_t kAcl   = 0x0001;
constexpr uint8_t kCigId  = 1;  // as in the firmware's LE Audio HAL
constexpr uint8_t kCisId  = 0;

/* ------------------------------- engine platform (C6 radio + controller) ------------------------------- */

struct Fake {
    std::vector<std::vector<uint8_t>> host;  // H4 packets to the P4 (from the engine and the controller)
    std::vector<std::vector<uint8_t>> ctrl;  // LL control PDUs sent on the ACL
    cis_acl_timing_t timing{};
    cis_link_params_t lp{};
    const cis_link_cb_t* cb = nullptr;
    void* cbArg             = nullptr;
    cis_radio_caps_t caps{};
    uint64_t rng = 0x1234567890ABCDEFull;
    char linkObj[4] = {};
};
Fake F;

void fHci(void*, const uint8_t* h4, uint16_t len)
{
    F.host.emplace_back(h4, h4 + len);
}
int fCtrl(void*, uint16_t, const uint8_t* ctr, uint8_t len)
{
    F.ctrl.emplace_back(ctr, ctr + len);
    return 0;
}
int fTiming(void*, uint16_t acl, cis_acl_timing_t* out)
{
    if (acl != kAcl) {
        return -1;
    }
    *out = F.timing;
    return 0;
}
bool fCentral(void*, uint16_t)
{
    return true;
}
uint64_t fFeat(void*, uint16_t)
{
    return (1ull << CIS_FEAT_BIT_CIS_PERIPHERAL) | 1ull;
}
int fOpen(void*, const cis_link_params_t* p, const cis_link_cb_t* cb, void* arg, cis_link_t** out)
{
    F.lp    = *p;
    F.cb    = cb;
    F.cbArg = arg;
    *out    = reinterpret_cast<cis_link_t*>(F.linkObj);
    return 0;
}
void fClose(void*, cis_link_t*)
{
    F.cb = nullptr;
}
void fCaps(void*, cis_radio_caps_t* out)
{
    *out = F.caps;
}
uint32_t fNow(void*)
{
    return 900000;
}
void fRand(void*, uint8_t* buf, uint16_t len)
{
    for (uint16_t i = 0; i < len; i++) {
        F.rng ^= F.rng << 13;
        F.rng ^= F.rng >> 7;
        F.rng ^= F.rng << 17;
        buf[i] = (uint8_t)(F.rng >> 32);
    }
}
void fTimer(void*, uint32_t) {}
void fLog(void*, int, const char*) {}

cis_platform_ops_t makeOps()
{
    cis_platform_ops_t o{};
    o.hci_to_host         = fHci;
    o.send_ctrl_pdu       = fCtrl;
    o.acl_timing          = fTiming;
    o.acl_is_central      = fCentral;
    o.acl_remote_features = fFeat;
    o.link_open           = fOpen;
    o.link_close          = fClose;
    o.radio_caps          = fCaps;
    o.now_us              = fNow;
    o.aes128              = aes_ref_platform;
    o.random              = fRand;
    o.timer_start         = fTimer;
    o.log                 = fLog;
    return o;
}
const cis_platform_ops_t kOps = makeOps();
uint64_t g_mem[(64 * 1024) / 8];

/* ------------------------------------------- P4 side ------------------------------------------- */

// P4 state as in the firmware's BT HAL (command response) and LE Audio HAL (credits, CIS, microphone).
struct P4 {
    cis_engine_t* e = nullptr;
    size_t seen     = 0;  // how many packets from F.host have been handled
    // synchronous command (BT HAL sendHciCmd: waitOpcode, cmdStatus, cmdRet without the status byte)
    uint16_t waitOp = 0;
    bool done       = false;
    uint8_t status  = 0xFF;
    uint8_t ret[252] = {};
    uint8_t retLen  = 0;
    // LE Audio HAL state
    uint16_t isoLen = 0;
    IsoCredits credits;
    IsoReassembler reasm;
    uint16_t cisHandle = 0xFFFF;
    bool cisUp         = false;
    int cisSeq         = 0;
    CisEstablished cis;
    int discCis = 0;
    struct Rx {
        uint16_t handle;
        uint16_t seq;
        IsoStatus status;
        std::vector<uint8_t> data;
    };
    std::vector<Rx> rx;
    int isoErrors = 0;
};
P4 g;

// LE Audio HAL event handler (ev = code, length, parameters; without the H4 type byte)
void onEvent(const uint8_t* ev, size_t len)
{
    const uint8_t* p = ev + 2;
    const size_t n   = ev[1];
    if (len < 2 || n + 2 > len) {
        expect(false, "HCI event with a length greater than the packet");
        return;
    }
    if (ev[0] == 0x13) {
        expect(forEachCompleted(p, n, [](uint16_t h, uint16_t c) { g.credits.completed(h, c); }),
               "Number Of Completed Packets: structure");
    } else if (ev[0] == 0x05 && n >= 4) {
        const uint16_t h = (uint16_t)((p[1] | (p[2] << 8)) & 0x0FFF);
        if (p[0] == 0 && h == g.cisHandle && g.cisUp) {
            g.cisUp = false;
            g.credits.dropHandle(h);
            g.discCis++;
        }
    } else if (ev[0] == 0x3E && n >= 1 && p[0] == 0x19) {
        CisEstablished c;
        if (parseCisEstablished(p + 1, n - 1, c) && c.handle == g.cisHandle) {
            g.cis   = c;
            g.cisUp = c.status == 0;
            g.cisSeq++;
        }
    }
}

// BT HAL: handling a controller packet (events 0x0E/0x0F for the pending command, the rest to LE Audio, ISO)
void deliver()
{
    for (; g.seen < F.host.size(); g.seen++) {
        const std::vector<uint8_t>& pkt = F.host[g.seen];
        if (pkt.size() >= 3 && pkt[0] == 0x04) {
            const uint8_t* ev     = pkt.data() + 1;
            const uint8_t* params = ev + 2;
            const size_t avail    = std::min<size_t>(pkt.size() - 3, ev[1]);
            if (ev[0] == 0x0E && avail >= 4) {
                const uint16_t op = (uint16_t)(params[1] | (params[2] << 8));
                if (g.waitOp != 0 && op == g.waitOp) {
                    g.status = params[3];
                    g.retLen = (uint8_t)std::min<size_t>(avail - 4, sizeof(g.ret));
                    std::memcpy(g.ret, params + 4, g.retLen);
                    g.done = true;
                }
            } else if (ev[0] == 0x0F && avail >= 4) {
                const uint16_t op = (uint16_t)(params[2] | (params[3] << 8));
                if (g.waitOp != 0 && op == g.waitOp) {
                    g.status = params[0];
                    g.retLen = 0;
                    g.done   = true;
                }
            } else {
                onEvent(ev, pkt.size() - 1);
            }
        } else if (!pkt.empty() && pkt[0] == 0x05) {
            // LE Audio HAL ISO receive handler
            IsoSdu s;
            const auto r = g.reasm.feed(pkt.data(), pkt.size(), s);
            if (r == IsoReassembler::Result::Error) {
                g.isoErrors++;
            } else if (r == IsoReassembler::Result::Sdu) {
                g.rx.push_back(P4::Rx{s.handle, s.seq, s.status, std::vector<uint8_t>(s.data, s.data + s.len)});
            }
        }
    }
}

// C6 controller event (not the engine): the engine peeks at it; if not consumed, it goes to the P4 unchanged.
void controllerEvent(uint8_t code, const std::vector<uint8_t>& params)
{
    std::vector<uint8_t> h4 = {0x04, code, (uint8_t)params.size()};
    h4.insert(h4.end(), params.begin(), params.end());
    if (!cis_engine_hci_evt_from_controller(g.e, h4.data(), (uint16_t)h4.size())) {
        F.host.push_back(h4);
    }
}

// C6 controller response to a command the engine did not handle (only those the P4 sends in this flow).
void controllerAnswer(uint16_t op)
{
    const uint8_t lo = (uint8_t)op, hi = (uint8_t)(op >> 8);
    switch (op) {
    case 0x2003:  // LE Read Local Supported Features: C6 without the ISO bits (28..32)
        controllerEvent(0x0E, {0x01, lo, hi, 0x00, 0xFF, 0x49, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00});
        break;
    case 0x2002:  // LE Read Buffer Size v1: 251 B x 12
        controllerEvent(0x0E, {0x01, lo, hi, 0x00, 0xFB, 0x00, 0x0C});
        break;
    case 0x2019:  // LE Enable Encryption
        controllerEvent(0x0F, {0x00, 0x01, lo, hi});
        break;
    default:
        std::printf("  (command 0x%04X went to the C6 controller, not to the engine)\n", op);
        expect(false, "LE Audio command did not reach the CIS engine");
        controllerEvent(0x0E, {0x01, lo, hi, 0x01});  // Unknown HCI Command
        break;
    }
}

// BT HAL sendHciCmd + halBtHciCmd: true = a response arrived; status and ret as the LE Audio HAL sees them
bool hci(uint16_t op, const uint8_t* params, size_t n)
{
    std::vector<uint8_t> h4 = {0x01, (uint8_t)op, (uint8_t)(op >> 8), (uint8_t)n};
    h4.insert(h4.end(), params, params + n);
    g.waitOp = op;
    g.done   = false;
    g.status = 0xFF;
    g.retLen = 0;
    if (!cis_engine_hci_cmd(g.e, h4.data(), (uint16_t)h4.size())) {
        controllerAnswer(op);
    }
    deliver();
    g.waitOp = 0;
    return g.done;
}

bool hciOk(uint16_t op, const uint8_t* params, size_t n, const char* what)
{
    const bool ok = hci(op, params, n) && g.status == 0;
    if (!ok) {
        std::printf("  %s (0x%04X): response %s, status 0x%02X\n", what, op, g.done ? "present" : "MISSING", g.status);
    }
    expect(ok, what);
    return ok;
}

/* ------------------------------ ACL encryption and the scripted Peripheral ------------------------------ */

const uint8_t kLtkMsb[16]  = {0x4C, 0x68, 0x38, 0x41, 0x39, 0xF5, 0x74, 0xD8,
                              0x36, 0xBC, 0xF3, 0x4E, 0x9D, 0xFB, 0x01, 0xBF};
const uint8_t kSkdmMsb[8]  = {0xAC, 0xBD, 0xCE, 0xDF, 0xE0, 0xF1, 0x02, 0x13};
const uint8_t kSkdsMsb[8]  = {0x02, 0x13, 0x24, 0x35, 0x46, 0x57, 0x68, 0x79};
const uint8_t kIvmMsb[4]   = {0xBA, 0xDC, 0xAB, 0x24};
const uint8_t kIvsMsb[4]   = {0xDE, 0xAF, 0xBA, 0xBE};
// Result from Core Vol 6 Part C 1 for the data above: SK and IV of the ACL connection
const uint8_t kSk[16]     = {0x99, 0xAD, 0x1B, 0x52, 0x26, 0xA3, 0x7E, 0x3E,
                             0x05, 0x8E, 0x3B, 0x8E, 0x27, 0xC2, 0xC6, 0x66};
const uint8_t kAclIv[8]   = {0x24, 0xAB, 0xDC, 0xBA, 0xBE, 0xBA, 0xAF, 0xDE};

// The P4 encrypts the ACL after pairing (BT HAL: LE Enable Encryption with the LTK in HCI order); LL_ENC_REQ/RSP is
// controller traffic that the C6 adapter shows to the engine (hooks ctrl_enc_req_make / ctrl_rx_pdu).
void encryptAcl()
{
    uint8_t p[28];
    p[0] = (uint8_t)kAcl;
    p[1] = (uint8_t)(kAcl >> 8);
    std::memset(p + 2, 0, 10);  // Random_Number and EDIV = 0 (LE Secure Connections)
    for (int i = 0; i < 16; i++) {
        p[12 + i] = kLtkMsb[15 - i];
    }
    hciOk(0x2019, p, sizeof(p), "LE Enable Encryption");
    uint8_t req[CIS_LL_ENC_REQ_LEN], rsp[CIS_LL_ENC_RSP_LEN];
    req[0] = CIS_LL_ENC_REQ;
    std::memset(req + 1, 0x22, 10);
    for (int i = 0; i < 8; i++) {
        req[11 + i] = kSkdmMsb[7 - i];
    }
    for (int i = 0; i < 4; i++) {
        req[19 + i] = kIvmMsb[3 - i];
    }
    cis_engine_ll_ctrl_tx_seen(g.e, kAcl, req, sizeof(req));
    rsp[0] = CIS_LL_ENC_RSP;
    for (int i = 0; i < 8; i++) {
        rsp[1 + i] = kSkdsMsb[7 - i];
    }
    for (int i = 0; i < 4; i++) {
        rsp[9 + i] = kIvsMsb[3 - i];
    }
    expect(!cis_engine_ll_ctrl_rx(g.e, kAcl, rsp, sizeof(rsp)), "LL_ENC_RSP only peeked at");
    controllerEvent(0x08, {0x00, (uint8_t)kAcl, (uint8_t)(kAcl >> 8), 0x01});
    deliver();
}

cis_ccm_t g_keys;

// Peripheral with FT = 1: SN = event number & 1, NESN toggled when it accepted our PDU (as in test_engine.c).
cis_pdu_t peerPdu(uint32_t ev, bool accepted, const uint8_t* d, uint8_t len)
{
    cis_pdu_t p;
    std::memset(&p, 0, sizeof(p));
    const bool sn   = (ev & 1) != 0;
    const bool nesn = sn ^ accepted;
    if (d == nullptr) {
        p.hdr0 = (uint8_t)(CIS_LLID_START_CONT | CIS_HDR_NPI | (nesn ? CIS_HDR_NESN : 0));
        return p;
    }
    p.hdr0 = (uint8_t)(CIS_LLID_COMPLETE_END | (sn ? CIS_HDR_SN : 0) | (nesn ? CIS_HDR_NESN : 0));
    cis_ccm_encrypt(&g_keys, ev, CIS_DIR_P_TO_C, (uint8_t)(p.hdr0 & CIS_CCM_AAD_MASK), d, len, p.payload);
    p.len = (uint8_t)(len + CIS_MIC_LEN);
    return p;
}

// One CIS event: the radio takes the PDU from the engine, the earbuds respond; returns the decrypted C->P payload
// (empty = null PDU). micData = microphone SDU from the earbuds in this event; rxOk = false - nothing received.
std::vector<uint8_t> radioEvent(uint32_t ev, const uint8_t* micData, uint8_t micLen, bool rxOk = true)
{
    std::vector<uint8_t> out;
    if (F.cb == nullptr) {
        expect(false, "CIS event without an open link");
        return out;
    }
    cis_pdu_t tx;
    std::memset(&tx, 0, sizeof(tx));
    expect(F.cb->prepare_tx(F.cbArg, ev, 0, &tx), "prepare_tx on a live link");
    bool accepted = false;
    if (tx.len > 0) {
        uint8_t plain[CIS_PDU_MAX_PAYLOAD];
        const bool micOk = tx.len > CIS_MIC_LEN &&
                           cis_ccm_decrypt(&g_keys, ev, CIS_DIR_C_TO_P, (uint8_t)(tx.hdr0 & CIS_CCM_AAD_MASK),
                                           tx.payload, tx.len, plain);
        expect(micOk, "C->P PDU: MIC matches the independently derived CIS keys");
        if (micOk) {
            out.assign(plain, plain + tx.len - CIS_MIC_LEN);
        }
        accepted = true;
    }
    if (rxOk) {
        const cis_pdu_t rx = peerPdu(ev, accepted, micData, micLen);
        F.cb->rx_done(F.cbArg, ev, 0, &rx, true, 0);
    } else {
        F.cb->rx_done(F.cbArg, ev, 0, nullptr, false, 0);
    }
    F.cb->event_closed(F.cbArg, ev);
    cis_engine_process(g.e);
    deliver();
    return out;
}

/* ------------------------------------------- scenarios ------------------------------------------- */

struct Scenario {
    const char* name;
    bool mic;
    AsePref sinkPref, srcPref;
};

PacSet pac(uint16_t freqMask, uint16_t octMin, uint16_t octMax)
{
    PacSet s;
    s.caps[0].freqMask     = freqMask;
    s.caps[0].durMask      = 0x02;  // 10 ms
    s.caps[0].chanMask     = 0x01;
    s.caps[0].octMin       = octMin;
    s.caps[0].octMax       = octMax;
    s.caps[0].maxFramesSdu = 1;
    s.n                    = 1;
    return s;
}

// LE Audio HAL isoSend: one SDU per CIS, all fragments or nothing
bool isoSend(uint16_t cisH, uint16_t seq, const uint8_t* sdu, uint16_t len)
{
    const uint16_t room = g.isoLen > 4 ? (uint16_t)(g.isoLen - 4) : 0;
    const uint16_t need = room == 0 ? 0xFFFF : (uint16_t)((len + room - 1) / room);
    if (need == 0xFFFF || g.credits.available() < need) {
        return false;
    }
    for (uint16_t i = 0; i < need; i++) {
        g.credits.take(cisH);
    }
    uint8_t pkt[1 + 4 + 8 + 400];
    uint16_t off = 0;
    do {
        const size_t n = buildIsoPacket(pkt, sizeof(pkt), cisH, seq, false, 0, sdu, len, g.isoLen, off);
        if (n == 0) {
            return false;
        }
        cis_engine_hci_iso(g.e, pkt, (uint16_t)n);
    } while (off < len);
    return true;
}

void run(const Scenario& sc)
{
    std::printf("%s\n", sc.name);
    F       = Fake{};
    g       = P4{};
    F.timing.event_counter      = 100;
    F.timing.anchor_us          = 1000000;
    F.timing.interval_us        = 30000;  // ACL 30 ms
    F.timing.crc_init           = 0x123456;
    const uint8_t map[5]        = {0xFF, 0xFF, 0xFF, 0xFF, 0x1F};
    std::memcpy(F.timing.chan_map, map, 5);
    F.timing.phy_tx = F.timing.phy_rx = CIS_PHY_2M;
    // C6 radio (cis_c6_radio.c radio_caps): NSE 1, SN/NESN from the engine, PDU prepared about 3 ms before the anchor
    F.caps.max_nse                  = 1;
    F.caps.radio_owns_seq           = false;
    F.caps.tx_prepare_lead_us       = 3000;
    F.caps.subevents_prepared_ahead = false;
    g.e = cis_engine_init(g_mem, sizeof(g_mem), &kOps);
    expect(g.e != nullptr, "CIS engine: init");
    if (g.e == nullptr) {
        return;
    }

    // BT HAL radio start: LE features (bit 28 = CIS Central is added by the engine), then CIS Host Support (bit 32)
    expect(hci(0x2003, nullptr, 0) && g.status == 0 && g.retLen >= 8, "LE Read Local Supported Features");
    expect((g.ret[3] & 0x10) != 0, "P4 sees CIS Central (bit 28) in the features from the C6");
    const uint8_t cisHostParam[2] = {32, 1};
    hciOk(0x2074, cisHostParam, sizeof(cisHostParam), "LE Set Host Feature (bit 32)");
    encryptAcl();

    // 1. ISO buffers: v1 first (the engine peeks at the ACL fields), then v2
    IsoBuffers ib;
    hciOk(0x2002, nullptr, 0, "LE Read Buffer Size");
    hciOk(0x2060, nullptr, 0, "LE Read Buffer Size v2");
    expect(parseReadBufferSizeV2(g.ret, g.retLen, ib) && ib.isoLen >= 8 && ib.isoNum > 0,
           "P4 accepts the ISO buffers from LE Read Buffer Size v2");
    expectEq(ib.aclLen, 251, "v2 buffers: ACL length taken from the v1 peek");
    g.isoLen = ib.isoLen;
    g.credits.reset(ib.isoNum);

    // 2-6. PACS / ASCS (the same selection functions as in the LE Audio HAL)
    CodecCfg sinkCfg, srcCfg;
    expect(chooseSinkConfig(pac(0x0084 | 0x0004, 40, 155), 0x01, sinkCfg), "Sink configuration choice");
    expectEq(sinkCfg.octets, 120, "Sink 48_4 (120 B)");
    if (sc.mic) {
        expect(chooseSourceConfig(pac(0x0004 | 0x0020, 40, 80), 0x01, srcCfg), "Source configuration choice");
        expectEq(srcCfg.octets, 40, "Source 16_2 (40 B)");
    }
    QosCfg sinkQos, srcQos;
    expect(chooseQos(sc.sinkPref, sinkCfg, kCigId, kCisId, sinkQos), "QoS Sink");
    if (sc.mic) {
        expect(chooseQos(sc.srcPref, srcCfg, kCigId, kCisId, srcQos), "QoS Source");
    }

    // 8. CIG + CIS exactly as in the LE Audio HAL
    CigParams cig;
    cig.cigId         = kCigId;
    cig.sduIntervalUs = 10000;
    cig.latencyCtoPMs = sinkQos.maxLatencyMs;
    cig.latencyPtoCMs = sc.mic ? srcQos.maxLatencyMs : sinkQos.maxLatencyMs;
    cig.nCis          = 1;
    cig.cis[0] = CisParams{kCisId, sinkQos.maxSdu, (uint16_t)(sc.mic ? srcQos.maxSdu : 0), sinkQos.phy,
                           sc.mic ? srcQos.phy : sinkQos.phy, sinkQos.rtn, sc.mic ? srcQos.rtn : sinkQos.rtn};
    uint8_t b[64];
    size_t n = buildSetCigParams(b, sizeof(b), cig);
    expect(n > 0, "buildSetCigParams");
    if (!hciOk(0x2062, b, n, "LE Set CIG Parameters")) {
        return;
    }
    expect(g.retLen >= 4 && g.ret[0] == kCigId && g.ret[1] == 1, "Set CIG response as the P4 expects");
    const uint16_t cisH = (uint16_t)((g.ret[2] | (g.ret[3] << 8)) & 0x0FFF);
    g.cisHandle         = cisH;

    const int seq0      = g.cisSeq;
    const uint16_t aclH = kAcl;
    n = buildCreateCis(b, sizeof(b), &cisH, &aclH, 1);
    if (!hciOk(0x2064, b, n, "LE Create CIS")) {
        return;
    }
    // LL_CIS_REQ to the earbuds must carry what the P4 asked for
    expectEq((long)F.ctrl.size(), 1, "LL_CIS_REQ sent on the ACL");
    if (F.ctrl.size() != 1) {
        return;
    }
    cis_ll_cis_req_t req;
    std::memset(&req, 0, sizeof(req));
    expect(cis_ll_cis_req_parse(F.ctrl[0].data(), (uint8_t)F.ctrl[0].size(), &req), "LL_CIS_REQ: structure");
    expectEq(req.cig_id, kCigId, "LL_CIS_REQ CIG_ID");
    expectEq(req.cis_id, kCisId, "LL_CIS_REQ CIS_ID");
    expectEq(req.max_sdu_c_to_p, sinkQos.maxSdu, "LL_CIS_REQ Max_SDU C->P = QoS Sink");
    expectEq(req.max_sdu_p_to_c, sc.mic ? srcQos.maxSdu : 0, "LL_CIS_REQ Max_SDU P->C");
    expectEq(req.sdu_interval_c_to_p, 10000, "LL_CIS_REQ SDU_Interval C->P");
    expectEq(req.phy_c_to_p, CIS_PHY_2M, "LL_CIS_REQ PHY C->P");
    expect(!req.framed, "LL_CIS_REQ unframed (as in the ASE QoS)");
    expectEq(req.iso_interval, 8, "LL_CIS_REQ ISO_Interval 10 ms");
    cis_ll_cis_rsp_t rsp{};
    rsp.cis_offset_min_us = req.cis_offset_min_us;
    rsp.cis_offset_max_us = req.cis_offset_max_us;
    rsp.conn_event_count  = req.conn_event_count;
    uint8_t rspB[CIS_LL_CIS_RSP_LEN];
    cis_ll_cis_rsp_pack(&rsp, rspB);
    expect(cis_engine_ll_ctrl_rx(g.e, kAcl, rspB, sizeof(rspB)), "LL_CIS_RSP consumed by the engine");
    cis_ll_cis_ind_t ind;
    std::memset(&ind, 0, sizeof(ind));
    expect(F.ctrl.size() == 2 && cis_ll_cis_ind_parse(F.ctrl[1].data(), (uint8_t)F.ctrl[1].size(), &ind),
           "LL_CIS_IND sent");
    g_keys.aes     = aes_ref_platform;
    g_keys.aes_ctx = nullptr;
    std::memcpy(g_keys.sk, kSk, 16);
    cis_crypto_cis_iv(kAclIv, ind.access_address, g_keys.iv);

    // event 0 -> LE CIS Established -> P4 parser
    radioEvent(0, nullptr, 0);
    expect(g.cisSeq == seq0 + 1 && g.cisUp, "P4 recognised LE CIS Established with status 0");
    expectEq(g.cis.handle, cisH, "CIS Established: handle");
    expectEq(g.cis.isoInterval, 8, "CIS Established: ISO_Interval");
    expectEq(g.cis.nse, 1, "CIS Established: NSE");
    expectEq(g.cis.bnCtoP, 1, "CIS Established: BN C->P");
    expectEq(g.cis.bnPtoC, sc.mic ? 1 : 0, "CIS Established: BN P->C");
    expectEq(g.cis.maxPduCtoP, sinkQos.maxSdu, "CIS Established: Max_PDU C->P = SDU (unframed, BN 1)");
    expectEq(g.cis.maxPduPtoC, sc.mic ? srcQos.maxSdu : 0, "CIS Established: Max_PDU P->C");
    expectEq(g.cis.phyCtoP, 2, "CIS Established: PHY C->P");
    expect(g.cis.latCtoPUs > 0 && g.cis.latCtoPUs <= (uint32_t)cig.latencyCtoPMs * 1000u,
           "CIS Established: C->P latency within Max_Transport_Latency");

    // 9. ISO paths over HCI
    n = buildSetupIsoPath(b, sizeof(b), cisH, false);
    hciOk(0x206E, b, n, "LE Setup ISO Data Path (input)");
    expect(g.retLen >= 2 && (uint16_t)(g.ret[0] | (g.ret[1] << 8)) == cisH, "Setup ISO Data Path: handle in response");
    if (sc.mic) {
        n = buildSetupIsoPath(b, sizeof(b), cisH, true);
        hciOk(0x206E, b, n, "LE Setup ISO Data Path (output)");
    }

    // Stream: P4 supplies one SDU per event (sequence number from 0), earbuds acknowledge; microphone in events 2..7
    constexpr int kSdus = 8;
    const uint16_t sduLen = sinkQos.maxSdu;
    std::vector<std::vector<uint8_t>> sent, got;
    for (int i = 0; i < kSdus; i++) {
        std::vector<uint8_t> s(sduLen);
        for (uint16_t k = 0; k < sduLen; k++) {
            s[k] = (uint8_t)(i * 31 + k);
        }
        sent.push_back(s);
    }
    std::vector<std::vector<uint8_t>> micSent;
    for (uint32_t ev = 1; ev <= 16; ev++) {
        if (ev <= (uint32_t)kSdus) {
            expect(isoSend(cisH, (uint16_t)(ev - 1), sent[ev - 1].data(), sduLen), "P4 has a credit for the SDU");
        }
        const bool micNow = sc.mic && ev >= 2 && ev <= 7;
        std::vector<uint8_t> m;
        if (micNow) {
            m.resize(srcQos.maxSdu);
            for (size_t k = 0; k < m.size(); k++) {
                m[k] = (uint8_t)(ev * 7 + k);
            }
            micSent.push_back(m);
        }
        // event 9: the earbuds do not respond (reception lost in both directions)
        const bool lostRx = sc.mic && ev == 9;
        std::vector<uint8_t> pl = radioEvent(ev, micNow ? m.data() : nullptr, (uint8_t)m.size(), !lostRx);
        if (!pl.empty()) {
            got.push_back(pl);
        }
    }
    expectEq((long)got.size(), kSdus, "all SDUs from the P4 reached the earbuds");
    bool same = got.size() == sent.size();
    for (size_t i = 0; same && i < got.size(); i++) {
        same = got[i] == sent[i];
    }
    expect(same, "C->P SDU content and order unchanged (byte for byte)");
    expectEq(g.credits.available(), ib.isoNum, "ISO credits returned via Number Of Completed Packets");
    expectEq((long)g.credits.bogus(), 0, "the controller did not return more credits than it received");
    cis_engine_stats_t st;
    expect(cis_engine_get_stats(g.e, cisH, &st), "CIS statistics");
    expectEq((long)st.sdu_dropped, 0, "the engine did not drop any SDU from the P4");
    expectEq((long)st.sdu_late, 0, "the engine did not consider any SDU from the P4 late");
    expectEq(g.isoErrors, 0, "P4 accepted every ISO packet from the C6 (lengths, flags)");
    if (sc.mic) {
        std::vector<P4::Rx> valid;
        int lost = 0;
        for (const auto& r : g.rx) {
            expectEq(r.handle, cisH, "ISO from the C6: CIS handle");
            if (r.status == IsoStatus::Valid && !r.data.empty()) {
                valid.push_back(r);
            } else if (r.status == IsoStatus::Lost) {
                lost++;
            }
        }
        expectEq((long)valid.size(), (long)micSent.size(), "microphone: every SDU from the earbuds reached the P4");
        bool micSame = valid.size() == micSent.size();
        for (size_t i = 0; micSame && i < valid.size(); i++) {
            micSame = valid[i].data == micSent[i];
        }
        expect(micSame, "microphone: content byte for byte");
        bool seqOk = true;
        for (size_t i = 1; i < g.rx.size(); i++) {
            seqOk = seqOk && (uint16_t)(g.rx[i].seq - g.rx[i - 1].seq) == 1;
        }
        expect(seqOk, "microphone: consecutive sequence numbers (the P4 detects gaps)");
        expect(lost >= 1, "microphone: lost SDU reported as Lost");
    }

    // teardown() of the LE Audio HAL: remove the paths, disconnect the CIS, remove the CIG
    uint8_t rm[3];
    expectEq((long)buildRemoveIsoPath(rm, sizeof(rm), cisH, false, false), 0, "Remove ISO Data Path with no direction");
    n = buildRemoveIsoPath(rm, sizeof(rm), cisH, true, sc.mic);
    hciOk(0x206F, rm, n, "LE Remove ISO Data Path (directions set)");
    const uint8_t d[3] = {(uint8_t)(cisH & 0xFF), (uint8_t)(cisH >> 8), 0x13};
    hciOk(0x0406, d, sizeof(d), "Disconnect (CIS)");
    cis_engine_process(g.e);
    deliver();
    expect(!g.cisUp && g.discCis == 1, "P4 received Disconnection Complete for the CIS");
    const uint8_t c[1] = {kCigId};
    hciOk(0x2065, c, sizeof(c), "LE Remove CIG");
    expect(!cis_engine_active(g.e), "engine has no CIG after teardown (the C6 hooks pass everything through)");
}

}  // namespace

int main()
{
    // Music: 48_4, earbud preference "high reliability" (RTN 4, 60 ms)
    Scenario music{"P4<->C6 contract: music (no microphone)", false, {}, {}};
    music.sinkPref.phyMask      = 0x03;
    music.sinkPref.rtn          = 4;
    music.sinkPref.maxLatencyMs = 60;
    music.sinkPref.pdMinUs      = 20000;
    music.sinkPref.pdMaxUs      = 40000;
    run(music);

    // Call: 48_4 downstream + 16_2 microphone, "low latency" (RTN 2, 10 ms)
    Scenario call{"P4<->C6 contract: call (earbud microphone)", true, {}, {}};
    call.sinkPref.phyMask      = 0x02;
    call.sinkPref.rtn          = 2;
    call.sinkPref.maxLatencyMs = 10;
    call.sinkPref.pdMinUs      = 20000;
    call.sinkPref.pdMaxUs      = 40000;
    call.srcPref               = call.sinkPref;
    run(call);

    std::printf("le_audio_contract: %d OK, %d FAIL\n", g_ok, g_fail);
    return g_fail == 0 ? 0 : 1;
}
