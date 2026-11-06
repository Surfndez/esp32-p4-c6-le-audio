/*
 * Cyberdeck LE Audio: pure CIS protocol (stateless, platform-free) - LLCP PDU encoding, CIG/CIS parameters, access
 * address, CSA#2 with ISO subevents, CIS PDU header, unframed segmentation, HCI ISO and HCI events.
 *
 * Everything here is a pure function: the engine, the simulator and the tests compute exactly the same thing, and
 * the tests compare against the Core sample data without mocks. Byte order: little-endian on the air and in HCI
 * (Core). Sources: Core 5.4 Vol 6 Part B (LL), Part C (sample data), Part G (ISOAL), Vol 4 Part E (HCI);
 * cross-checked against: Zephyr subsys/bluetooth/controller/ll_sw (Apache-2.0).
 */
#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "cis_platform.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ---- Implementation limits (not from Core): small, because the C6 memory is shared with Wi-Fi ---- */
#define CIS_MAX_CIS        2    /* CISes in one CIG: left + right earbud */
#define CIS_MIC_LEN        4
/* Max_PDU without MIC: the contract keeps the payload with MIC in 251 B (cis_pdu_t), hence 247. */
#define CIS_MAX_PDU        (CIS_PDU_MAX_PAYLOAD - CIS_MIC_LEN)
/* SDU of up to 2 PDUs (BN <= 2): a 10 ms LC3 frame of one channel is <= 155 B, with room for two channels in one
 * CIS. */
#define CIS_SDU_MAX        (2 * CIS_MAX_PDU)
#define CIS_NSE_MAX        31   /* NSE field in Core: 0x01..0x1F */
#define CIS_JITTER_MIN     2    /* C->P jitter buffer in SDUs: the former fixed engine value */
#define CIS_JITTER_MAX     4    /* 40 ms at 10 ms: music from the SD card (SDIO shared with the C6) without dropouts */
#define CIS_FT_MAX         16   /* more makes no sense with a jitter buffer of a few SDUs */

/* ---- LLCP ---- */
#define CIS_LL_UNKNOWN_RSP        0x07
#define CIS_LL_ENC_REQ            0x03
#define CIS_LL_ENC_RSP            0x04
#define CIS_LL_REJECT_EXT_IND     0x11
#define CIS_LL_CIS_REQ            0x1F
#define CIS_LL_CIS_RSP            0x20
#define CIS_LL_CIS_IND            0x21
#define CIS_LL_CIS_TERMINATE_IND  0x22

/* Lengths by opcode (opcode + CtrData). */
#define CIS_LL_CIS_REQ_LEN        (1 + 35)
#define CIS_LL_CIS_RSP_LEN        (1 + 8)
#define CIS_LL_CIS_IND_LEN        (1 + 15)
#define CIS_LL_CIS_TERM_LEN       (1 + 3)
#define CIS_LL_REJECT_EXT_LEN     (1 + 2)
#define CIS_LL_UNKNOWN_RSP_LEN    (1 + 1)
#define CIS_LL_ENC_REQ_LEN        (1 + 22)
#define CIS_LL_ENC_RSP_LEN        (1 + 12)

/* Smallest CIS_Offset per Core (Vol 6 Part B 2.4.2.29). */
#define CIS_OFFSET_MIN_US         500u

typedef struct {
	uint8_t  cig_id;
	uint8_t  cis_id;
	uint8_t  phy_c_to_p;          /* PHY mask with a single bit (CIS_PHY_*) */
	uint8_t  phy_p_to_c;
	uint16_t max_sdu_c_to_p;      /* 12 bits */
	uint16_t max_sdu_p_to_c;
	bool     framed;
	uint32_t sdu_interval_c_to_p; /* 20 bits, us */
	uint32_t sdu_interval_p_to_c;
	uint16_t max_pdu_c_to_p;      /* without MIC */
	uint16_t max_pdu_p_to_c;
	uint8_t  nse;
	uint32_t sub_interval_us;     /* 24 bits; 0 when NSE = 1 */
	uint8_t  bn_c_to_p;           /* 4 bits */
	uint8_t  bn_p_to_c;
	uint8_t  ft_c_to_p;
	uint8_t  ft_p_to_c;
	uint16_t iso_interval;        /* units of 1.25 ms */
	uint32_t cis_offset_min_us;   /* 24 bits */
	uint32_t cis_offset_max_us;
	uint16_t conn_event_count;
} cis_ll_cis_req_t;

typedef struct {
	uint32_t cis_offset_min_us;
	uint32_t cis_offset_max_us;
	uint16_t conn_event_count;
} cis_ll_cis_rsp_t;

typedef struct {
	uint32_t access_address;
	uint32_t cis_offset_us;
	uint32_t cig_sync_delay_us;
	uint32_t cis_sync_delay_us;
	uint16_t conn_event_count;
} cis_ll_cis_ind_t;

typedef struct {
	uint8_t cig_id;
	uint8_t cis_id;
	uint8_t error_code;
} cis_ll_cis_term_t;

/* Packing: out = opcode + CtrData; returns the length. Parsing: ctr = opcode + CtrData, false for a wrong opcode
 * or a length other than exact (a PDU of a different length is not ours - we pass it to the controller). */
uint8_t cis_ll_cis_req_pack(const cis_ll_cis_req_t *r, uint8_t out[CIS_LL_CIS_REQ_LEN]);
bool cis_ll_cis_req_parse(const uint8_t *ctr, uint8_t len, cis_ll_cis_req_t *r);
uint8_t cis_ll_cis_rsp_pack(const cis_ll_cis_rsp_t *r, uint8_t out[CIS_LL_CIS_RSP_LEN]);
bool cis_ll_cis_rsp_parse(const uint8_t *ctr, uint8_t len, cis_ll_cis_rsp_t *r);
uint8_t cis_ll_cis_ind_pack(const cis_ll_cis_ind_t *r, uint8_t out[CIS_LL_CIS_IND_LEN]);
bool cis_ll_cis_ind_parse(const uint8_t *ctr, uint8_t len, cis_ll_cis_ind_t *r);
uint8_t cis_ll_cis_term_pack(const cis_ll_cis_term_t *r, uint8_t out[CIS_LL_CIS_TERM_LEN]);
bool cis_ll_cis_term_parse(const uint8_t *ctr, uint8_t len, cis_ll_cis_term_t *r);
uint8_t cis_ll_reject_ext_pack(uint8_t reject_opcode, uint8_t error_code, uint8_t out[CIS_LL_REJECT_EXT_LEN]);
bool cis_ll_reject_ext_parse(const uint8_t *ctr, uint8_t len, uint8_t *reject_opcode, uint8_t *error_code);
bool cis_ll_unknown_rsp_parse(const uint8_t *ctr, uint8_t len, uint8_t *unknown_type);
/* LL_ENC_REQ: SKDm and IVm in over-the-air order (LSO first). */
bool cis_ll_enc_req_parse(const uint8_t *ctr, uint8_t len, uint8_t skdm[8], uint8_t ivm[4]);
bool cis_ll_enc_rsp_parse(const uint8_t *ctr, uint8_t len, uint8_t skds[8], uint8_t ivs[4]);

/* ---- HCI LE Set CIG Parameters and derivation of the CIG/CIS parameters ---- */
typedef struct {
	uint8_t  cis_id;
	uint16_t max_sdu_c_to_p;
	uint16_t max_sdu_p_to_c;
	uint8_t  phy_c_to_p;          /* HCI mask: bit0 1M, bit1 2M, bit2 Coded */
	uint8_t  phy_p_to_c;
	uint8_t  rtn_c_to_p;
	uint8_t  rtn_p_to_c;
} cis_hci_cis_cfg_t;

typedef struct {
	uint8_t  cig_id;
	uint32_t sdu_interval_c_to_p;
	uint32_t sdu_interval_p_to_c;
	uint8_t  sca;
	uint8_t  packing;
	uint8_t  framing;
	uint16_t latency_c_to_p_ms;
	uint16_t latency_p_to_c_ms;
	uint8_t  cis_count;
	cis_hci_cis_cfg_t cis[CIS_MAX_CIS];
} cis_hci_cig_params_t;

/* Parameters of one CIS after derivation (they go into LL_CIS_REQ, link_open and CIS Established). */
typedef struct {
	uint8_t  phy_c_to_p;          /* CIS_PHY_* (one bit) */
	uint8_t  phy_p_to_c;
	uint16_t max_sdu_c_to_p;
	uint16_t max_sdu_p_to_c;
	uint8_t  max_pdu_c_to_p;      /* without MIC */
	uint8_t  max_pdu_p_to_c;
	uint8_t  bn_c_to_p;
	uint8_t  bn_p_to_c;
	uint8_t  ft_c_to_p;
	uint8_t  ft_p_to_c;
	uint8_t  nse;
	uint32_t sub_interval_us;     /* for LL_CIS_REQ: 0 when NSE = 1 */
	uint32_t se_length_us;        /* duration of one subevent (including T_IFS and T_MSS) */
	uint32_t cis_sync_delay_us;
	uint32_t tl_c_to_p_us;
	uint32_t tl_p_to_c_us;
	/* C->P SDUs buffered before the first transmission (tolerance of gaps in delivery from the host): margin
	 * (Max_Transport_Latency - TL) / SDU interval, within CIS_JITTER_MIN..CIS_JITTER_MAX. The engine further clamps it
	 * to the queue capacity at the given BN. */
	uint8_t  jitter_c_to_p;
} cis_cis_derived_t;

typedef struct {
	uint16_t iso_interval;        /* units of 1.25 ms */
	uint32_t iso_interval_us;
	uint32_t cig_sync_delay_us;
	cis_cis_derived_t cis[CIS_MAX_CIS];
} cis_cig_derived_t;

/* Parsing the parameters of command 0x2062 (without the command header). Returns an HCI code (0 = OK). A CIS_Count
 * larger than CIS_MAX_CIS gives 0x07 (Memory Capacity Exceeded), because it is a memory limit, not a host error. */
uint8_t cis_hci_cig_params_parse(const uint8_t *p, uint16_t len, cis_hci_cig_params_t *out);
/* CIG derivation: ISO interval = SDU interval (unframed only), BN = ceil(Max_SDU/Max_PDU), FT from the transport
 * latency, NSE from RTN limited by the radio max_nse, sequential packing with a gap of gap_us between consecutive
 * CISes (cis_radio_caps_t.cis_gap_us); at NSE > 1 Sub_Interval = se_length + sub_gap_us
 * (cis_radio_caps_t.subevent_gap_us), CIS time in the CIG = cis_slot_us. Returns an HCI code (0 = OK, 0x11 =
 * parameters beyond what we support). */
uint8_t cis_cig_derive(const cis_hci_cig_params_t *in, uint8_t max_nse, uint32_t gap_us, uint32_t sub_gap_us,
		cis_cig_derived_t *out);
/* CIS time in the CIG without the CIS gap: NSE x Sub_Interval, and at NSE 1 se_length + sub_gap_us - a radio that
 * serves each subevent with a separate event (C6) needs a gap after the only one too. Shared by cis_cig_derive and
 * by the CIG placement relative to the ACL (cis_engine.c place_cig). */
static inline uint32_t cis_slot_us(uint8_t nse, uint32_t se_length_us, uint32_t sub_gap_us)
{
	return (nse > 1 ? nse : 1u) * (se_length_us + sub_gap_us);
}
/* PDU air time (preamble, AA, header, payload, CRC) for 1M/2M. */
uint32_t cis_pdu_air_us(uint16_t payload_with_mic, uint8_t phy);

/* ---- CIS access address (Core Vol 6 Part B 2.1.2) ---- */
#define CIS_ADV_ACCESS_ADDRESS 0x8E89BED6u
bool cis_aa_valid(uint32_t aa);
/* Draws an AA that satisfies the rules and differs from avoid[]. Returns 0 when it did not succeed after many tries
 * (0 is never a valid AA: all octets equal). */
uint32_t cis_aa_generate(void (*rnd)(void *ctx, uint8_t *buf, uint16_t len), void *ctx,
		const uint32_t *avoid, uint8_t n_avoid);

/* ---- CSA#2 (Core Vol 6 Part B 4.5.8.3) ---- */
uint16_t cis_csa2_chan_id(uint32_t aa);
uint8_t cis_csa2_num_used(const uint8_t chan_map[5]);
/* ACL event (non-ISO). */
uint8_t cis_csa2_chan(uint16_t counter, uint16_t chan_id, const uint8_t chan_map[5]);
typedef struct {
	uint16_t prn_lu;      /* prnSubEvent_lu */
	uint16_t remap_idx;   /* index of the most recently used channel in the used-channels list */
} cis_csa2_iso_t;
/* First subevent of an ISO event (counter = the 16 low bits of cisEventCounter). */
uint8_t cis_csa2_iso_event(uint16_t counter, uint16_t chan_id, const uint8_t chan_map[5], cis_csa2_iso_t *st);
/* Subsequent subevents of the same event. */
uint8_t cis_csa2_iso_subevent(uint16_t chan_id, const uint8_t chan_map[5], cis_csa2_iso_t *st);

/* ---- CIS PDU header ---- */
#define CIS_LLID_COMPLETE_END    0x00   /* unframed: whole SDU or its end */
#define CIS_LLID_START_CONT      0x01   /* unframed: start or continuation; null PDU and padding */
#define CIS_LLID_FRAMED          0x02
#define CIS_HDR_NESN             0x04
#define CIS_HDR_SN               0x08
#define CIS_HDR_CIE              0x10
#define CIS_HDR_NPI              0x40
/* AAD mask for CCM in a CIS: NESN, SN, CIE, NPI are zeroed (Zephyr radio.c: ADATAMASK 0xA3). */
#define CIS_CCM_AAD_MASK         0xA3

typedef struct {
	uint8_t llid;
	bool    nesn;
	bool    sn;
	bool    cie;
	bool    npi;
} cis_pdu_hdr_t;

uint8_t cis_pdu_hdr_pack(const cis_pdu_hdr_t *h);
void cis_pdu_hdr_unpack(uint8_t hdr0, cis_pdu_hdr_t *h);

/* ---- Payload numbers and the flush point (Core Vol 6 Part B 4.5.13.5) ---- */
/* Last event and last subevent (from 0) in which payload pn may go on the air. */
void cis_flush_point(uint64_t pn, uint8_t bn, uint8_t ft, uint8_t nse, uint64_t *last_event, uint8_t *last_subevent);

/* ---- Unframed SDU <-> PDU (Core Vol 6 Part G 2.1) ---- */
/* SDU fragment for payload idx (0..bn-1): offset, length and LLID. Payloads past the end of the SDU are padding
 * (length 0, LLID 0b01). false = the SDU does not fit in bn * max_pdu. */
bool cis_unframed_fragment(uint16_t sdu_len, uint8_t max_pdu, uint8_t bn, uint8_t idx,
		uint16_t *offset, uint8_t *len, uint8_t *llid);

/* Reassembling an SDU from the PDUs of one SDU interval (bn payloads in order). */
typedef struct {
	uint8_t *buf;
	uint16_t cap;
	uint16_t len;
	bool     started;
	bool     ended;
	bool     error;       /* missing fragment, bad LLID or overflow */
	bool     any_data;
} cis_unframed_rx_t;

#define CIS_ISO_STATUS_VALID     0x0
#define CIS_ISO_STATUS_INVALID   0x1   /* data may be incomplete */
#define CIS_ISO_STATUS_LOST      0x2

void cis_unframed_rx_begin(cis_unframed_rx_t *s, uint8_t *buf, uint16_t cap);
/* pdu = NULL: payload lost (flushed without reception). */
void cis_unframed_rx_pdu(cis_unframed_rx_t *s, uint8_t llid, const uint8_t *data, uint8_t len, bool lost);
/* Finishes the SDU: returns the HCI ISO packet status (CIS_ISO_STATUS_*); length in s->len (0 for LOST). */
uint8_t cis_unframed_rx_end(cis_unframed_rx_t *s);

/* ---- HCI ISO (H4 type 0x05) ---- */
#define CIS_H4_CMD   0x01
#define CIS_H4_ACL   0x02
#define CIS_H4_EVT   0x04
#define CIS_H4_ISO   0x05

#define CIS_ISO_PB_FIRST     0x0
#define CIS_ISO_PB_CONT      0x1
#define CIS_ISO_PB_COMPLETE  0x2
#define CIS_ISO_PB_LAST      0x3

typedef struct {
	uint16_t handle;
	uint8_t  pb;
	bool     ts_flag;
	uint32_t timestamp;
	uint16_t psn;          /* PB first/complete only */
	uint16_t sdu_len;      /* PB first/complete only */
	uint8_t  status;       /* Packet_Status_Flag (controller -> host only) */
	const uint8_t *data;
	uint16_t data_len;
} cis_hci_iso_t;

/* h4 with the type byte 0x05. false = inconsistent packet (lengths, type, RFU). */
bool cis_hci_iso_parse(const uint8_t *h4, uint16_t len, cis_hci_iso_t *out);
/* Builds an H4 packet (with the type byte). Returns the length, or 0 when it does not fit in cap. */
uint16_t cis_hci_iso_build(const cis_hci_iso_t *in, uint8_t *out, uint16_t cap);

/* ---- HCI: opcodes, events, errors ---- */
#define CIS_OP_DISCONNECT           0x0406
#define CIS_OP_RESET                0x0C03
#define CIS_OP_LE_READ_BUF_SIZE     0x2002
#define CIS_OP_LE_READ_LOCAL_FEAT   0x2003
#define CIS_OP_LE_ENABLE_ENC        0x2019
#define CIS_OP_LE_READ_BUF_SIZE_V2  0x2060
#define CIS_OP_LE_READ_ISO_TX_SYNC  0x2061
#define CIS_OP_LE_SET_CIG_PARAMS    0x2062
#define CIS_OP_LE_CREATE_CIS        0x2064
#define CIS_OP_LE_REMOVE_CIG        0x2065
#define CIS_OP_LE_SETUP_ISO_PATH    0x206E
#define CIS_OP_LE_REMOVE_ISO_PATH   0x206F
#define CIS_OP_LE_SET_HOST_FEATURE  0x2074

#define CIS_EVT_DISCONN_COMPLETE    0x05
#define CIS_EVT_ENC_CHANGE          0x08
#define CIS_EVT_CMD_COMPLETE        0x0E
#define CIS_EVT_CMD_STATUS          0x0F
#define CIS_EVT_NUM_COMPLETED_PKTS  0x13
#define CIS_EVT_ENC_KEY_REFRESH     0x30
#define CIS_EVT_LE_META             0x3E
#define CIS_LE_SUBEVT_CIS_ESTABLISHED 0x19

#define CIS_FEAT_BIT_CIS_CENTRAL    28
#define CIS_FEAT_BIT_CIS_PERIPHERAL 29
#define CIS_FEAT_BIT_CIS_HOST       32

#define CIS_ERR_SUCCESS             0x00
#define CIS_ERR_UNKNOWN_CMD         0x01
#define CIS_ERR_UNKNOWN_CONN_ID     0x02
#define CIS_ERR_PIN_KEY_MISSING     0x06
#define CIS_ERR_MEM_CAPACITY        0x07
#define CIS_ERR_CONN_TIMEOUT        0x08
#define CIS_ERR_CMD_DISALLOWED      0x0C
#define CIS_ERR_LIMITED_RESOURCES   0x0D
#define CIS_ERR_UNSUPP_FEATURE      0x11
#define CIS_ERR_INVALID_PARAMS      0x12
#define CIS_ERR_REMOTE_USER_TERM    0x13
#define CIS_ERR_LOCAL_HOST_TERM     0x16
#define CIS_ERR_UNSUPP_REMOTE_FEAT  0x1A
#define CIS_ERR_UNSPECIFIED         0x1F
#define CIS_ERR_UNSUPP_LL_PARAM     0x20
#define CIS_ERR_LL_RESP_TIMEOUT     0x22
#define CIS_ERR_MIC_FAILURE         0x3D
#define CIS_ERR_CONN_FAIL_ESTAB     0x3E
#define CIS_ERR_UNKNOWN_ADV_ID      0x42
#define CIS_ERR_OP_CANCELLED_BY_HOST 0x44

/* H4 events (with the type byte 0x04). They return the packet length. The out buffer must hold at least
 * 3 + the parameter size; for Command Complete 6 + plen. */
uint16_t cis_hci_evt_cmd_complete(uint8_t *out, uint16_t opcode, const uint8_t *ret, uint8_t ret_len);
uint16_t cis_hci_evt_cmd_status(uint8_t *out, uint8_t status, uint16_t opcode);
uint16_t cis_hci_evt_disconn_complete(uint8_t *out, uint8_t status, uint16_t handle, uint8_t reason);
uint16_t cis_hci_evt_nocp(uint8_t *out, const uint16_t *handles, const uint16_t *counts, uint8_t n);

typedef struct {
	uint8_t  status;
	uint16_t handle;
	uint32_t cig_sync_delay_us;
	uint32_t cis_sync_delay_us;
	uint32_t tl_c_to_p_us;
	uint32_t tl_p_to_c_us;
	uint8_t  phy_c_to_p;
	uint8_t  phy_p_to_c;
	uint8_t  nse;
	uint8_t  bn_c_to_p;
	uint8_t  bn_p_to_c;
	uint8_t  ft_c_to_p;
	uint8_t  ft_p_to_c;
	uint16_t max_pdu_c_to_p;
	uint16_t max_pdu_p_to_c;
	uint16_t iso_interval;
} cis_hci_cis_established_t;

#define CIS_HCI_CIS_ESTABLISHED_LEN (3 + 29)
uint16_t cis_hci_evt_cis_established(uint8_t *out, const cis_hci_cis_established_t *ev);

/* Reads an H4 command header: false when the packet is shorter than the header or plen does not match the length. */
bool cis_hci_cmd_header(const uint8_t *h4, uint16_t len, uint16_t *opcode, const uint8_t **params, uint8_t *plen);

/* Little-endian helpers (shared by the engine and the tests). */
static inline uint16_t cis_get_le16(const uint8_t *p) { return (uint16_t)(p[0] | (p[1] << 8)); }
static inline uint32_t cis_get_le24(const uint8_t *p) { return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16); }
static inline uint32_t cis_get_le32(const uint8_t *p) { return cis_get_le24(p) | ((uint32_t)p[3] << 24); }
static inline void cis_put_le16(uint8_t *p, uint16_t v) { p[0] = (uint8_t)v; p[1] = (uint8_t)(v >> 8); }
static inline void cis_put_le24(uint8_t *p, uint32_t v) { p[0] = (uint8_t)v; p[1] = (uint8_t)(v >> 8); p[2] = (uint8_t)(v >> 16); }
static inline void cis_put_le32(uint8_t *p, uint32_t v) { cis_put_le24(p, v); p[3] = (uint8_t)(v >> 24); }

#ifdef __cplusplus
}
#endif
