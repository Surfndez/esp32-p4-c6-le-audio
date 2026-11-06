/*
 * Cyberdeck LE Audio: CIS engine on the Central side (link layer) - public API (contract).
 *
 * Inputs: HCI from the host (P4), LL control PDUs received on the ACL, ACL events (encryption, disconnection),
 * timer. Outputs: through cis_platform_ops_t. Details: docs/c6-link-layer.md.
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "cis_platform.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct cis_engine cis_engine_t;

/* Engine memory in one block provided by the platform (no malloc in the engine). */
uint32_t cis_engine_mem_size(void);
cis_engine_t *cis_engine_init(void *mem, uint32_t mem_size, const cis_platform_ops_t *ops);

/* H4 packet from the host, type 0x01 (command). Returns true when the engine handled the command (reply sent via
 * hci_to_host); false = pass to the controller unchanged. Command 0x2019 (LE Enable Encryption) is only
 * snooped (LTK) and always returns false. */
bool cis_engine_hci_cmd(cis_engine_t *e, const uint8_t *h4, uint16_t len);

/* H4 packet from the host, type 0x05 (ISO data). Always consumed. */
void cis_engine_hci_iso(cis_engine_t *e, const uint8_t *h4, uint16_t len);

/* HCI event from the controller to the host (snooped: Disconnection Complete, Encryption Change, LE Connection
 * Complete). Returns true when the engine consumes it (it does not reach the host). */
bool cis_engine_hci_evt_from_controller(cis_engine_t *e, const uint8_t *h4, uint16_t len);

/* LL control PDU received on the ACL: ctr = opcode + CtrData. Returns true when consumed (CIS opcodes
 * with a valid length, or LL_REJECT_EXT_IND / LL_UNKNOWN_RSP concerning our procedure). */
bool cis_engine_ll_ctrl_rx(cis_engine_t *e, uint16_t acl_handle, const uint8_t *ctr, uint8_t len);

/* LL control PDU sent by the controller (snoops LL_ENC_REQ: SKDm, IVm). */
void cis_engine_ll_ctrl_tx_seen(cis_engine_t *e, uint16_t acl_handle, const uint8_t *ctr, uint8_t len);

void cis_engine_on_timer(cis_engine_t *e);

/* Work deferred from the radio context (encryption, decryption, HCI events) - call from a task. */
void cis_engine_process(cis_engine_t *e);

/* true = the engine has an active CIG (hooks pass everything when false). Snooping (0x2019, LL_ENC_REQ/RSP,
 * Encryption Change, 0x2002/0x2003, Data Length Change) is always forwarded by the platform, because ACL encryption
 * usually starts before the first CIG command. */
bool cis_engine_active(const cis_engine_t *e);

/* Counters of a single CIS. Reset on LE Create CIS. */
typedef struct {
	uint32_t events;          /* completed CIS events (including skipped ones) */
	uint32_t events_skipped;  /* event_skipped from the radio */
	uint32_t tx_pdus;         /* transmitted data PDUs (including retransmissions) */
	uint32_t tx_acked;        /* acknowledged payloads */
	uint32_t tx_flushed;      /* data payloads flushed after FT without an acknowledgement */
	uint32_t tx_underrun;     /* payloads without host data after the stream started (a null PDU was sent) */
	uint32_t sdu_in;          /* SDUs from the host accepted into the buffer */
	uint32_t sdu_dropped;     /* SDUs dropped (full buffer, bad length, no data path) */
	uint32_t sdu_late;        /* late SDUs - the buffer was reset */
	uint32_t rx_ok;           /* data PDUs received (MIC OK) */
	uint32_t rx_crc;          /* PDUs with a CRC error */
	uint32_t rx_mic_fail;
	uint32_t rx_lost;         /* payloads from the Peripheral flushed without reception */
	uint32_t rx_overflow;     /* no room in the receive queue (PDU not acknowledged, the Peripheral will retransmit) */
	uint32_t key_src;         /* CIS_KEY_*: where the CIS encryption key came from */
} cis_engine_stats_t;

/* cis_engine_stats_t.key_src - set when the CIS is set up from the ACL state. */
#define CIS_KEY_NONE     0 /* ACL without encryption - so is the CIS */
#define CIS_KEY_SNOOP    1 /* snooped LTK/SKD/IV only (the platform did not supply the controller keys) */
#define CIS_KEY_MATCH    2 /* snooped == controller keys */
#define CIS_KEY_CTRL     3 /* controller keys only (snooping incomplete) */
#define CIS_KEY_MISMATCH 4 /* snooped != controller keys - the controller keys were used */

/* false = unknown CIS handle. Call from a task. */
bool cis_engine_get_stats(const cis_engine_t *e, uint16_t cis_handle, cis_engine_stats_t *out);

#ifdef __cplusplus
}
#endif
