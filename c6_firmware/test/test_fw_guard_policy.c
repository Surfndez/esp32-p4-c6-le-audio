/*
 * Host test of the C6 image guard's decisions (fw_guard_policy).
 * Run: sh test/run_host_tests.sh  (gcc, no ESP-IDF)
 */
#include <stdio.h>
#include <stdlib.h>

#include "fw_guard_policy.h"

static int failures;

#define CHECK(cond) do { \
	if (!(cond)) { \
		printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond); \
		failures++; \
	} \
} while (0)

static fw_guard_rec_t rec(uint8_t boots, uint8_t crashes, uint16_t secs)
{
	fw_guard_rec_t r = { .boots = boots, .crashes = crashes, .gave_up = 0, .reserved = 0, .secs = secs };
	return r;
}

static void confirmed_image_runs_free_without_counting(void)
{
	fw_guard_rec_t r = rec(9, 2, 590);
	fw_guard_decision_t d = fw_guard_decide_boot(true, true, &r, FW_GUARD_RST_CRASH);
	CHECK(d.action == FW_GUARD_FREE);
	CHECK(d.rec.boots == 9 && d.rec.crashes == 2);
}

static void image_whose_rollback_failed_runs_free(void)
{
	/* Without a slot to go back to, further trials would only block OTA for 10 minutes at every boot. */
	fw_guard_rec_t r = rec(11, 3, 600);
	r.gave_up = 1;
	fw_guard_decision_t d = fw_guard_decide_boot(false, true, &r, FW_GUARD_RST_HW_DOG);
	CHECK(d.action == FW_GUARD_FREE);
}

static void no_previous_image_means_no_trial(void)
{
	fw_guard_decision_t d = fw_guard_decide_boot(false, false, NULL, FW_GUARD_RST_CRASH);
	CHECK(d.action == FW_GUARD_FREE);
}

static void fresh_install_starts_trial(void)
{
	fw_guard_decision_t d = fw_guard_decide_boot(false, true, NULL, FW_GUARD_RST_NORMAL);
	CHECK(d.action == FW_GUARD_TRIAL);
	CHECK(d.rec.boots == 1 && d.rec.crashes == 0 && d.rec.secs == 0);
}

static void third_crash_rolls_back(void)
{
	fw_guard_rec_t r = rec(2, 1, 20);
	fw_guard_decision_t d = fw_guard_decide_boot(false, true, &r, FW_GUARD_RST_CRASH);
	CHECK(d.action == FW_GUARD_TRIAL && d.rec.crashes == 2);
	d = fw_guard_decide_boot(false, true, &d.rec, FW_GUARD_RST_CRASH);
	CHECK(d.action == FW_GUARD_ROLLBACK && d.rec.crashes == 3);
}

static void normal_resets_do_not_count_as_crashes(void)
{
	fw_guard_rec_t r = rec(1, 2, 100);
	fw_guard_decision_t d = fw_guard_decide_boot(false, true, &r, FW_GUARD_RST_NORMAL);
	CHECK(d.action == FW_GUARD_TRIAL && d.rec.crashes == 2 && d.rec.boots == 2);
}

static void hw_watchdog_in_trial_rolls_back_at_once(void)
{
	fw_guard_decision_t d = fw_guard_decide_boot(false, true, NULL, FW_GUARD_RST_HW_DOG);
	CHECK(d.action == FW_GUARD_ROLLBACK);
}

static void eleventh_boot_rolls_back(void)
{
	fw_guard_rec_t r = rec(FW_GUARD_MAX_BOOTS - 1, 0, 10);
	fw_guard_decision_t d = fw_guard_decide_boot(false, true, &r, FW_GUARD_RST_NORMAL);
	CHECK(d.action == FW_GUARD_TRIAL && d.rec.boots == FW_GUARD_MAX_BOOTS);
	d = fw_guard_decide_boot(false, true, &d.rec, FW_GUARD_RST_NORMAL);
	CHECK(d.action == FW_GUARD_ROLLBACK);
}

static void trial_time_is_summed_across_resets(void)
{
	fw_guard_rec_t r = rec(3, 0, FW_GUARD_TRIAL_SECS - 1);
	fw_guard_decision_t d = fw_guard_decide_boot(false, true, &r, FW_GUARD_RST_NORMAL);
	CHECK(d.action == FW_GUARD_TRIAL);
	r.secs = FW_GUARD_TRIAL_SECS;
	d = fw_guard_decide_boot(false, true, &r, FW_GUARD_RST_NORMAL);
	CHECK(d.action == FW_GUARD_ROLLBACK);
}

static void counters_saturate(void)
{
	fw_guard_rec_t r = rec(UINT8_MAX, UINT8_MAX, 0);
	fw_guard_decision_t d = fw_guard_decide_boot(false, true, &r, FW_GUARD_RST_CRASH);
	CHECK(d.rec.boots == UINT8_MAX && d.rec.crashes == UINT8_MAX);
	CHECK(d.action == FW_GUARD_ROLLBACK);
}

static void trial_left_counts_down_and_stops_at_zero(void)
{
	CHECK(fw_guard_trial_left(0, 0) == FW_GUARD_TRIAL_SECS);
	CHECK(fw_guard_trial_left(500, 99) == 1);
	CHECK(fw_guard_trial_left(500, 100) == 0);
	CHECK(fw_guard_trial_left(UINT16_MAX, UINT32_MAX) == 0);
}

static fw_guard_health_t wifi_host(bool sta, uint32_t to_air, uint32_t to_host)
{
	fw_guard_health_t h = { .datapath = true, .sta_connected = sta, .bt_enabled = true, .frames_to_air = to_air,
		.frames_to_host = to_host, .wifi_requested = true };
	return h;
}

static fw_guard_health_t ble_host(uint32_t hci_from, uint32_t hci_to)
{
	fw_guard_health_t h = { .datapath = true, .bt_enabled = true, .hci_from_host = hci_from, .hci_to_host = hci_to };
	return h;
}

static void health_needs_link_radio_bt_and_traffic_both_ways(void)
{
	fw_guard_health_t h = wifi_host(true, FW_GUARD_MIN_FRAMES, FW_GUARD_MIN_FRAMES);
	CHECK(fw_guard_health_ok(&h));
	h.frames_to_host = FW_GUARD_MIN_FRAMES - 1;
	CHECK(!fw_guard_health_ok(&h));
	h.frames_to_host = 1000;
	h.frames_to_air = 0;
	CHECK(!fw_guard_health_ok(&h));
	h.frames_to_air = 1000;
	h.bt_enabled = false;
	CHECK(!fw_guard_health_ok(&h));
	h.bt_enabled = true;
	h.sta_connected = false;
	CHECK(!fw_guard_health_ok(&h));
	h.sta_connected = true;
	h.datapath = false;
	CHECK(!fw_guard_health_ok(&h));
}

static void confirm_needs_full_window_and_traffic_inside_it(void)
{
	const fw_guard_health_t start = wifi_host(true, 20, 30);
	fw_guard_health_t now = wifi_host(true, 21, 31);
	CHECK(fw_guard_should_confirm(FW_GUARD_HEALTHY_SECS, &start, &now));
	CHECK(!fw_guard_should_confirm(FW_GUARD_HEALTHY_SECS - 1, &start, &now));
	/* The data path died after the first frames: the counters stand still in the window -> no confirmation. */
	now.frames_to_air = start.frames_to_air;
	CHECK(!fw_guard_should_confirm(FW_GUARD_HEALTHY_SECS * 5, &start, &now));
	now.frames_to_air = 21;
	now.frames_to_host = start.frames_to_host;
	CHECK(!fw_guard_should_confirm(FW_GUARD_HEALTHY_SECS * 5, &start, &now));
	now.frames_to_host = 31;
	now.sta_connected = false;
	CHECK(!fw_guard_should_confirm(FW_GUARD_HEALTHY_SECS, &start, &now));
}

/* Crash counter before normal operation (RTC, no NVS): after power-on the RTC memory holds garbage. */
static void early_garbage_after_power_on_starts_from_zero(void)
{
	fw_guard_early_t st = { .magic = 0xDEADBEEFu, .crashes = 77 };
	CHECK(!fw_guard_early_step(&st, false));
	CHECK(st.magic == FW_GUARD_EARLY_MAGIC && st.crashes == 0);
	fw_guard_early_t st2 = { .magic = 0x12345678u, .crashes = 2 };
	CHECK(!fw_guard_early_step(&st2, true));
	CHECK(st2.crashes == 1);
}

static void early_third_crash_in_a_row_rolls_back_and_resets(void)
{
	fw_guard_early_t st = { 0 };
	CHECK(!fw_guard_early_step(&st, false));
	CHECK(!fw_guard_early_step(&st, true));
	CHECK(!fw_guard_early_step(&st, true));
	CHECK(fw_guard_early_step(&st, true));
	CHECK(st.crashes == 0);
}

static void early_normal_reset_or_reaching_host_link_clears_count(void)
{
	fw_guard_early_t st = { 0 };
	fw_guard_early_step(&st, false);
	fw_guard_early_step(&st, true);
	fw_guard_early_step(&st, true);
	CHECK(!fw_guard_early_step(&st, false));
	CHECK(st.crashes == 0);
	fw_guard_early_step(&st, true);
	fw_guard_early_step(&st, true);
	fw_guard_early_passed(&st);
	CHECK(!fw_guard_early_step(&st, true));
	CHECK(st.crashes == 1);
}

/* A host that enabled Wi-Fi will not confirm the image on BLE alone: Wi-Fi may be its only path to a C6 update. */
static void wifi_host_without_wifi_is_not_healthy_even_with_hci(void)
{
	fw_guard_health_t h = wifi_host(false, 0, 0);
	h.hci_from_host = 1000;
	h.hci_to_host = 1000;
	CHECK(!fw_guard_health_ok(&h));
	CHECK(!fw_guard_should_confirm(FW_GUARD_HEALTHY_SECS * 2, &h, &h));
}

/* BLE-only host (an LE Audio player without a network): SDIO, BLE and HCI both ways suffice, Wi-Fi is not needed. */
static void ble_only_host_is_healthy_with_hci_both_ways(void)
{
	fw_guard_health_t h = ble_host(FW_GUARD_MIN_HCI, FW_GUARD_MIN_HCI);
	CHECK(fw_guard_health_ok(&h));
	h.hci_to_host = FW_GUARD_MIN_HCI - 1;
	CHECK(!fw_guard_health_ok(&h));
	h = ble_host(FW_GUARD_MIN_HCI - 1, FW_GUARD_MIN_HCI);
	CHECK(!fw_guard_health_ok(&h));
	h = ble_host(100, 100);
	h.bt_enabled = false;
	CHECK(!fw_guard_health_ok(&h));
	h = ble_host(100, 100);
	h.datapath = false;
	CHECK(!fw_guard_health_ok(&h));
}

static void ble_only_host_confirms_after_full_window_with_traffic_inside_it(void)
{
	const fw_guard_health_t start = ble_host(FW_GUARD_MIN_HCI, FW_GUARD_MIN_HCI);
	fw_guard_health_t now = ble_host(FW_GUARD_MIN_HCI + 3, FW_GUARD_MIN_HCI + 5);
	CHECK(fw_guard_should_confirm(FW_GUARD_HEALTHY_SECS, &start, &now));
	CHECK(!fw_guard_should_confirm(FW_GUARD_HEALTHY_SECS - 1, &start, &now));
	/* datapath stays 1 after the host dies (only an explicit close clears it): no HCI traffic in the window - no
	 * confirmation, as on the Wi-Fi path */
	now = start;
	CHECK(!fw_guard_should_confirm(FW_GUARD_HEALTHY_SECS * 5, &start, &now));
	/* Traffic from the host flows but nothing reaches the host: the C6 -> P4 path is dead */
	now = ble_host(FW_GUARD_MIN_HCI + 9, FW_GUARD_MIN_HCI);
	CHECK(!fw_guard_should_confirm(FW_GUARD_HEALTHY_SECS * 5, &start, &now));
}

int main(void)
{
	early_garbage_after_power_on_starts_from_zero();
	early_third_crash_in_a_row_rolls_back_and_resets();
	early_normal_reset_or_reaching_host_link_clears_count();
	confirmed_image_runs_free_without_counting();
	image_whose_rollback_failed_runs_free();
	confirm_needs_full_window_and_traffic_inside_it();
	no_previous_image_means_no_trial();
	fresh_install_starts_trial();
	third_crash_rolls_back();
	normal_resets_do_not_count_as_crashes();
	hw_watchdog_in_trial_rolls_back_at_once();
	eleventh_boot_rolls_back();
	trial_time_is_summed_across_resets();
	counters_saturate();
	trial_left_counts_down_and_stops_at_zero();
	health_needs_link_radio_bt_and_traffic_both_ways();
	wifi_host_without_wifi_is_not_healthy_even_with_hci();
	ble_only_host_is_healthy_with_hci_both_ways();
	ble_only_host_confirms_after_full_window_with_traffic_inside_it();
	if (failures) {
		printf("%d FAILED\n", failures);
		return EXIT_FAILURE;
	}
	printf("fw_guard_policy: OK\n");
	return EXIT_SUCCESS;
}
