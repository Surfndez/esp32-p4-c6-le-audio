/*
 * Cyberdeck: host test of the CIS adapter's self-disable after crashes (main/cis_c6_crash.c).
 * Run: sh test/run_host_tests.sh  (gcc, no ESP-IDF)
 */
#include <stdio.h>
#include <string.h>

#include "cis_c6_crash.h"

static int failures;

#define CHECK(cond) do { \
	if (!(cond)) { \
		printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond); \
		failures++; \
	} \
} while (0)

static const uint8_t IMG_A[CIS_CRASH_IMG_LEN] = { 1, 2, 3, 4, 5, 6, 7, 8 };
static const uint8_t IMG_B[CIS_CRASH_IMG_LEN] = { 8, 7, 6, 5, 4, 3, 2, 1 };

static cis_crash_rec_t rec(const uint8_t *img, uint8_t crashes)
{
	cis_crash_rec_t r = { .ver = CIS_CRASH_REC_VER, .crashes = crashes };
	memcpy(r.img, img, CIS_CRASH_IMG_LEN);
	return r;
}

static void first_boot_enables_without_write(void)
{
	cis_crash_boot_t b = cis_crash_boot(NULL, IMG_A, false, false);
	CHECK(!b.disabled && !b.write && b.rec.crashes == 0);
	CHECK(memcmp(b.rec.img, IMG_A, CIS_CRASH_IMG_LEN) == 0);
}

static void crash_outside_cis_is_not_counted(void)
{
	/* A crash without a live CIS is not the adapter's concern (fw_guard counts it, or nobody does). */
	cis_crash_rec_t r = rec(IMG_A, 1);
	cis_crash_boot_t b = cis_crash_boot(&r, IMG_A, false, true);
	CHECK(!b.disabled && !b.write && b.rec.crashes == 1);
}

static void normal_reset_during_cis_is_not_counted(void)
{
	/* The P4 resets the C6 on each of its own starts - not a crash, even in the middle of a stream. */
	cis_crash_rec_t r = rec(IMG_A, 1);
	cis_crash_boot_t b = cis_crash_boot(&r, IMG_A, true, false);
	CHECK(!b.disabled && !b.write && b.rec.crashes == 1);
}

static void second_crash_during_cis_disables(void)
{
	cis_crash_boot_t b1 = cis_crash_boot(NULL, IMG_A, true, true);
	CHECK(!b1.disabled && b1.write && b1.rec.crashes == 1);
	cis_crash_boot_t b2 = cis_crash_boot(&b1.rec, IMG_A, true, true);
	CHECK(b2.disabled && b2.write && b2.rec.crashes == 2);
	/* It stays disabled after further normal starts of the same image. */
	cis_crash_boot_t b3 = cis_crash_boot(&b2.rec, IMG_A, false, false);
	CHECK(b3.disabled && !b3.write);
}

static void new_image_resets_counter(void)
{
	cis_crash_rec_t r = rec(IMG_A, 2);
	cis_crash_boot_t b = cis_crash_boot(&r, IMG_B, false, false);
	CHECK(!b.disabled && b.write && b.rec.crashes == 0);
	CHECK(memcmp(b.rec.img, IMG_B, CIS_CRASH_IMG_LEN) == 0);
}

static void unknown_record_version_starts_fresh(void)
{
	cis_crash_rec_t r = rec(IMG_A, 5);
	r.ver = CIS_CRASH_REC_VER + 1;
	cis_crash_boot_t b = cis_crash_boot(&r, IMG_A, false, false);
	CHECK(!b.disabled && b.write && b.rec.crashes == 0);
}

static void counter_saturates(void)
{
	cis_crash_rec_t r = rec(IMG_A, 255);
	cis_crash_boot_t b = cis_crash_boot(&r, IMG_A, true, true);
	CHECK(b.disabled && b.rec.crashes == 255);
}

int main(void)
{
	first_boot_enables_without_write();
	crash_outside_cis_is_not_counted();
	normal_reset_during_cis_is_not_counted();
	second_crash_during_cis_disables();
	new_image_resets_counter();
	unknown_record_version_starts_fresh();
	counter_saturates();
	if (failures) {
		printf("cis_c6_crash: %d FAILURES\n", failures);
		return 1;
	}
	printf("cis_c6_crash: OK\n");
	return 0;
}
