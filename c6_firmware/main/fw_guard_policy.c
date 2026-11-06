/*
 * Cyberdeck: C6 image guard decisions - see fw_guard_policy.h.
 */
#include "fw_guard_policy.h"

#include <stddef.h>

static uint8_t inc_u8(uint8_t v)
{
	return v == UINT8_MAX ? v : (uint8_t)(v + 1);
}

fw_guard_decision_t fw_guard_decide_boot(bool confirmed, bool have_previous,
		const fw_guard_rec_t *stored, fw_guard_rst_t rst)
{
	fw_guard_decision_t d = { .action = FW_GUARD_FREE, .rec = { 0 }, .why = NULL };

	if (stored) {
		d.rec = *stored;
	}
	if (confirmed) {
		d.why = "image confirmed";
		return d;
	}
	if (d.rec.gave_up) {
		d.why = "rollback was impossible - no trial";
		return d;
	}
	if (!have_previous) {
		/* A trial without a rollback target would end in a restart loop - better to keep running. */
		d.why = "no valid previous image - no trial";
		return d;
	}

	d.rec.boots = inc_u8(d.rec.boots);
	if (rst == FW_GUARD_RST_CRASH) {
		d.rec.crashes = inc_u8(d.rec.crashes);
	}

	d.action = FW_GUARD_ROLLBACK;
	if (rst == FW_GUARD_RST_HW_DOG) {
		d.why = "RTC hardware watchdog during the trial (hang)";
	} else if (d.rec.crashes >= FW_GUARD_MAX_CRASHES) {
		d.why = "too many crashes during the trial";
	} else if (d.rec.boots > FW_GUARD_MAX_BOOTS) {
		d.why = "too many boots during the trial";
	} else if (d.rec.secs >= FW_GUARD_TRIAL_SECS) {
		d.why = "total trial time elapsed without confirmation";
	} else {
		d.action = FW_GUARD_TRIAL;
		d.why = "image unconfirmed - trial period";
	}
	return d;
}

/* A host that enabled Wi-Fi (RPC WifiStart) gets the Wi-Fi rule: for it Wi-Fi may be the only path to the next
 * C6 update, so an image with broken Wi-Fi must not confirm itself on BLE alone. A BLE-only host
 * (e.g. an LE Audio player without a network) never enables Wi-Fi - for it the evidence is HCI both ways. */
static bool wifi_ok(const fw_guard_health_t *h)
{
	return h->sta_connected && h->frames_to_air >= FW_GUARD_MIN_FRAMES && h->frames_to_host >= FW_GUARD_MIN_FRAMES;
}

static bool hci_ok(const fw_guard_health_t *h)
{
	return h->hci_from_host >= FW_GUARD_MIN_HCI && h->hci_to_host >= FW_GUARD_MIN_HCI;
}

bool fw_guard_health_ok(const fw_guard_health_t *h)
{
	if (!h->datapath || !h->bt_enabled) {
		return false;
	}
	return h->wifi_requested ? wifi_ok(h) : hci_ok(h);
}

bool fw_guard_should_confirm(uint32_t healthy_secs, const fw_guard_health_t *start,
		const fw_guard_health_t *now)
{
	if (healthy_secs < FW_GUARD_HEALTHY_SECS || !fw_guard_health_ok(now)) {
		return false;
	}
	/* Traffic in both directions within the window itself: datapath stays 1 after the host dies (only an explicit
	 * close clears it), and a counter since boot alone does not prove the path works now */
	if (!now->wifi_requested) {
		return now->hci_from_host != start->hci_from_host && now->hci_to_host != start->hci_to_host;
	}
	return now->frames_to_air != start->frames_to_air && now->frames_to_host != start->frames_to_host;
}

uint32_t fw_guard_trial_left(uint16_t secs_before_boot, uint32_t secs_this_boot)
{
	const uint64_t used = (uint64_t)secs_before_boot + secs_this_boot;
	return used >= FW_GUARD_TRIAL_SECS ? 0 : (uint32_t)(FW_GUARD_TRIAL_SECS - used);
}

bool fw_guard_early_step(fw_guard_early_t *st, bool crash)
{
	if (st->magic != FW_GUARD_EARLY_MAGIC) {
		/* RTC memory after power-on (or an EN pin reset from the P4) holds garbage. */
		st->magic = FW_GUARD_EARLY_MAGIC;
		st->crashes = 0;
	}
	if (!crash) {
		st->crashes = 0;
		return false;
	}
	if (++st->crashes < FW_GUARD_EARLY_CRASHES) {
		return false;
	}
	st->crashes = 0;
	return true;
}

void fw_guard_early_passed(fw_guard_early_t *st)
{
	st->magic = FW_GUARD_EARLY_MAGIC;
	st->crashes = 0;
}
