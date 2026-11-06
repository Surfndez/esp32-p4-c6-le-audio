#pragma once
// Internal interface between le_host.cpp (HCI/ATT over SDIO, ex hal_bt.cpp), le_stream.cpp (BAP unicast client,
// ex hal_le_audio.cpp) and le_audio_api.cpp (public C API). Not part of the public API.

#include <cstddef>
#include <cstdint>

/* ---------------- hal_bt.cpp services for LE Audio ---------------- */

// Synchronous HCI command: true = Command Complete/Status arrived; *status = its code; ret = return parameters after
// the status (Command Complete), *retLen = their length (truncated to retCap). Serialized with the rest of hal_bt
// (one command at a time).
bool halBtHciCmd(uint16_t opcode, const uint8_t* params, uint8_t plen, int timeoutMs, uint8_t* status, uint8_t* ret,
                 uint8_t retCap, uint8_t* retLen);
// ATT client request (first byte = opcode) and the server response; false = no response / disconnected.
bool halBtAtt(uint16_t connHandle, const uint8_t* req, uint16_t reqLen, uint8_t* rsp, uint16_t rspCap,
              uint16_t* rspLen, int timeoutMs);
// Raw H4 packet (e.g. HCI ISO 0x05) to the controller.
bool halBtSendRaw(const uint8_t* pkt, uint16_t len);

struct HalBtLeAudioLink {
    uint16_t acl            = 0xFFFF;  // ACL connection to the earbud (0xFFFF = none)
    bool encrypted          = false;   // LE Audio needs an encrypted link (BAP 5.x: security mode 1 level 2+)
    bool leAudioOn          = false;   // bt.leaudio setting (experiment on CIS emulation in the C6)
    bool cisCentral         = false;   // controller reported LE feature bit 28
    uint16_t dleTxOctets    = 0;       // from LE Data Length Change (0 = no event yet)
    uint16_t connItvl       = 0;       // ACL interval in 1.25 ms units (0 = unknown)
    uint16_t pacsStart = 0, pacsEnd = 0;
    uint16_t ascsCp         = 0;
    uint16_t sinkAse[2]     = {0, 0};  // value handles of Sink ASE (0x2BC4)
    uint16_t srcAse[2]      = {0, 0};  // value handles of Source ASE (0x2BC5)
    // Second member of the set (CSIP, e.g. left earbud next to a primary right one): connected and encrypted alongside
    // the primary; 0xFFFF = none. LE Audio discovers its PACS/ASCS itself (ATT); link state comes from halBtAcl.
    uint16_t acl2           = 0xFFFF;
    bool setJoining         = false;  // the second earbud of the set is joining right now (RSI scan, encryption) - wait
};
// Snapshot of the connection state needed by the BAP procedure (taken under the hal_bt lock).
HalBtLeAudioLink halBtLeAudioLink();
// State of one ACL connection by handle (primary or second set member).
struct HalBtAcl {
    bool up              = false;
    bool encrypted       = false;
    uint16_t dleTxOctets = 0;  // from LE Data Length Change (0 = no event yet)
    uint16_t connItvl    = 0;  // ACL interval in 1.25 ms units (0 = unknown)
};
HalBtAcl halBtAcl(uint16_t acl);
// LE Set Data Length (0x2022) for a connection - retry when the Data Length Change event did not arrive.
void halBtRequestDataLength(uint16_t acl);

/* ---------------- callbacks hal_bt.cpp -> le_stream.cpp ---------------- */
// All run on the SDIO receive thread (4 kB stack), OUTSIDE the hal_bt state lock: keep them short and non-blocking.

// HCI ISO packet (first byte 0x05).
void halLeAudioOnIsoRx(const uint8_t* pkt, uint32_t len);
// HCI event (without the H4 byte): Number Of Completed Packets, Disconnection Complete, LE CIS Established.
void halLeAudioOnHciEvent(const uint8_t* ev, uint32_t len);
// ATT notification / indication from an ASE or ASE Control Point handle.
void halLeAudioOnAttValue(uint16_t connHandle, uint16_t attrHandle, const uint8_t* val, uint16_t len);
// Whether the ATT handle belongs to LE Audio (ASE / ASE CP) - called UNDER the hal_bt lock: atomics only.
bool halLeAudioOwnsAttr(uint16_t attrHandle);

/* ---------------- credit-based SDU queue (diagnostic switch le_audio_fix) ---------------- */
// txq: an SDU without an ISO credit waits in a queue until txqDeadlineMs after the SDU instead of being dropped.
// Enabled by default (measured 2026-11-02: with credit-return stalls of up to 63 ms, loss fell from 0.21 to 0.10 %);
// disabling is only for A/B comparisons.
struct LeAudioFix {
    bool txq          = true;
    int txqDeadlineMs = 25;
};
LeAudioFix halLeAudioFixGet();
void halLeAudioFixSet(const LeAudioFix& f);
// Longest gap between NOCP events during the stream since the previous call (us); resets it.
uint32_t halLeAudioNocpGapTake();
// Resets the minima and maxima of bt.stream (credits_min, txq_max, max_*_us) - a measurement window without restarting
// the stream.
void halLeAudioResetExtremes();

/* ---------------- audio path (hal_audio.cpp) ---------------- */

// Stream to the earbuds is ready (CIS established, ISO path set up, ASE Streaming).
bool halLeAudioSinkReady();
// Audio stopped reaching the BT earbuds during playback (output task, hal_audio): speaker stays silent until
// btAudioLostAck.
void halAudioBtLost();
// The stream is playing or being set up (Configuring / Streaming phase) - without copying the status.
bool halLeAudioActive();
// btLeAudioRestart is in progress: the gap in the stream is intentional (no halAudioBtLost in the output task).
bool halLeAudioRestarting();
// CCIDs of the local GATT content control services (hal_bt.cpp kLocalGatt) - also in CCID_List on Enable:
// earbuds bind the stream to that service and send their gestures there.
constexpr uint8_t kGmcsCcid = 0x01;
constexpr uint8_t kGtbsCcid = 0x02;
// 48 kHz, interleaved int16 stereo (ready samples from the output task); queue to the LC3 encoder. Non-blocking.
// volumePct: output volume (0-100) - the Tab5 codec applies it in hardware, for BT earbuds it is applied digitally.
void halLeAudioPushPcm(const int16_t* stereo, size_t frames, int volumePct);
// Blocking push for producers without an audio clock: waits up to timeoutMs until at most 40 ms are queued; returns
// frames accepted (0 = stream not ready or timeout). le_audio_write_pcm.
size_t halLeAudioWritePcm(const int16_t* stereo, size_t frames, int volumePct, int timeoutMs);
// Earbud microphone: active 16 kHz mono stream.
bool halLeAudioMicReady();
// Reads up to n 16 kHz mono samples; waits up to timeoutMs. 0 = stream closed or timed out.
size_t halLeAudioMicRead(int16_t* out, size_t n, int timeoutMs);
