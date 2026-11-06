/*
 * SPDX-FileCopyrightText: 2015-2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "diskio_impl.h"
#include "ffconf.h"
#include "ff.h"
#include "sdmmc_cmd.h"
#include "diskio_sdmmc.h"
#include "diskio_sdmmc_split.h"
#include "esp_log.h"
#include "esp_compiler.h"
#include "esp_timer.h"
#include <stdatomic.h>

static sdmmc_card_t* s_cards[FF_VOLUMES] = { NULL };
static bool s_disk_status_check_en[FF_VOLUMES] = { };

static const char* TAG = "diskio_sdmmc";

/* Cyberdeck: how long the SD card holds the SDMMC controller. The controller is shared with the SDIO link to the C6
 * coprocessor (Wi-Fi, BT, LE Audio), and the IDF transaction lock (esp_driver_sdmmc sdmmc_transaction.c,
 * s_request_mutex) covers the whole data transfer of one command - that is how long SDIO waits. The duration
 * of a sdmmc_read/write_sectors call (one piece after the split, diskio_sdmmc_split.h) is the upper bound of
 * one transaction (unaligned buffer: several transactions through a bounce buffer; write: also CMD13 polling
 * until the card is ready, with SDIO getting in between). There is usually a single writer (the FATFS volume
 * lock); a race loses at most one maximum. Read by the firmware's status API (storage.sd_xfer_*).
 * TRIM separately: FF_USE_TRIM 1 (ffconf.h), so deleting and truncating files sends CMD38, and IDF waits for
 * the card's erase to finish IN THE SAME transaction (sdmmc_erase_sectors: SCF_WAIT_BUSY ->
 * wait_for_busy_cleared under s_request_mutex) - deleting one large file can hold the controller longer than
 * any read. */
static atomic_uint s_xfer_max_us;
static atomic_uint s_xfer_slow;
static atomic_uint s_req_max_sectors;
#if FF_USE_TRIM
static atomic_uint s_trim_max_us;
#endif

/* 10 ms = one LE Audio SDU interval; the C6 reserve is 4 SDUs (40 ms). */
#define CYBERDECK_SD_SLOW_US 10000

static void sd_note_request(UINT count)
{
    if (count > atomic_load_explicit(&s_req_max_sectors, memory_order_relaxed)) {
        atomic_store_explicit(&s_req_max_sectors, count, memory_order_relaxed);
    }
}

static void sd_note_xfer(int64_t us)
{
    const unsigned u = us > 0 ? (unsigned)us : 0;
    if (u > atomic_load_explicit(&s_xfer_max_us, memory_order_relaxed)) {
        atomic_store_explicit(&s_xfer_max_us, u, memory_order_relaxed);
    }
    if (u > CYBERDECK_SD_SLOW_US) {
        atomic_fetch_add_explicit(&s_xfer_slow, 1, memory_order_relaxed);
    }
}

#if FF_USE_TRIM
static void sd_note_trim(int64_t us)
{
    const unsigned u = us > 0 ? (unsigned)us : 0;
    if (u > atomic_load_explicit(&s_trim_max_us, memory_order_relaxed)) {
        atomic_store_explicit(&s_trim_max_us, u, memory_order_relaxed);
    }
}
#endif

void ff_sdmmc_cyberdeck_stats(ff_sdmmc_cyberdeck_stats_t* out)
{
    out->xfer_max_us     = atomic_load_explicit(&s_xfer_max_us, memory_order_relaxed);
    out->xfer_slow       = atomic_load_explicit(&s_xfer_slow, memory_order_relaxed);
    out->req_max_sectors = atomic_load_explicit(&s_req_max_sectors, memory_order_relaxed);
#if FF_USE_TRIM
    out->trim_max_us = atomic_load_explicit(&s_trim_max_us, memory_order_relaxed);
#else
    out->trim_max_us = 0;
#endif
}

//Check if SD/MMC card is present
static DSTATUS ff_sdmmc_card_available(BYTE pdrv)
{
    sdmmc_card_t* card = s_cards[pdrv];
    assert(card);
    esp_err_t err = sdmmc_get_status(card);
    if (unlikely(err != ESP_OK)) {
        ESP_LOGE(TAG, "Check status failed (0x%x)", err);
        return STA_NOINIT;
    }
    return 0;
}

/**
*   ff_sdmmc_status() and ff_sdmmc_initialize() return STA_NOINIT when sdmmc_get_status()
*   fails. This error value is checked throughout the FATFS code.
*   Both functions return 0 on success.
*/
static DSTATUS ff_sdmmc_initialize (BYTE pdrv)
{
    return ff_sdmmc_card_available(pdrv);
}

static DSTATUS ff_sdmmc_status(BYTE pdrv)
{
    if (s_disk_status_check_en[pdrv]) {
        return ff_sdmmc_card_available(pdrv);
    }
    return 0;
}

/* Cyberdeck: one piece of a FATFS request (diskio_sdmmc_split.h) = one measured sdmmc_*_sectors call. */
typedef struct {
    sdmmc_card_t* card;
    BYTE* rbuf;
    const BYTE* wbuf;
} sd_chunk_ctx_t;

static int sd_read_chunk(void* ctx, uint32_t sector, uint32_t count, size_t offset)
{
    const sd_chunk_ctx_t* c = (const sd_chunk_ctx_t*)ctx;
    const int64_t t0 = esp_timer_get_time();
    const esp_err_t err = sdmmc_read_sectors(c->card, c->rbuf + offset, sector, count);
    sd_note_xfer(esp_timer_get_time() - t0);
    return err;
}

static int sd_write_chunk(void* ctx, uint32_t sector, uint32_t count, size_t offset)
{
    const sd_chunk_ctx_t* c = (const sd_chunk_ctx_t*)ctx;
    const int64_t t0 = esp_timer_get_time();
    const esp_err_t err = sdmmc_write_sectors(c->card, c->wbuf + offset, sector, count);
    sd_note_xfer(esp_timer_get_time() - t0);
    return err;
}

static DRESULT ff_sdmmc_read (BYTE pdrv, BYTE* buff, DWORD sector, UINT count)
{
    sdmmc_card_t* card = s_cards[pdrv];
    assert(card);
    sd_note_request(count);
    /* Cyberdeck: FF_SDMMC_MAX_CHUNK_SECTORS per transaction - between them the C6 SDIO gets the controller */
    sd_chunk_ctx_t ctx = { .card = card, .rbuf = buff };
    esp_err_t err = ff_sdmmc_split_io(sector, count, (uint32_t)card->csd.sector_size, FF_SDMMC_MAX_CHUNK_SECTORS,
                                      sd_read_chunk, &ctx);
    if (unlikely(err != ESP_OK)) {
        ESP_LOGE(TAG, "sdmmc_read_blocks failed (0x%x)", err);
        return RES_ERROR;
    }
    return RES_OK;
}

static DRESULT ff_sdmmc_write (BYTE pdrv, const BYTE* buff, DWORD sector, UINT count)
{
    sdmmc_card_t* card = s_cards[pdrv];
    assert(card);
    sd_note_request(count);
    /* Cyberdeck: like read - FF_SDMMC_MAX_CHUNK_SECTORS each. At the end of programming a piece IDF polls with a
     * series of CMD13 (sdmmc_wait_for_idle), each a separate transaction - SDIO gets in then as well. */
    sd_chunk_ctx_t ctx = { .card = card, .wbuf = buff };
    esp_err_t err = ff_sdmmc_split_io(sector, count, (uint32_t)card->csd.sector_size, FF_SDMMC_MAX_CHUNK_SECTORS,
                                      sd_write_chunk, &ctx);
    if (unlikely(err != ESP_OK)) {
        ESP_LOGE(TAG, "sdmmc_write_blocks failed (0x%x)", err);
        return RES_ERROR;
    }
    return RES_OK;
}

#if FF_USE_TRIM
static DRESULT ff_sdmmc_trim (BYTE pdrv, DWORD start_sector, DWORD sector_count)
{
    sdmmc_card_t* card = s_cards[pdrv];
    assert(card);
    sdmmc_erase_arg_t arg;

    arg = sdmmc_can_discard(card) == ESP_OK ? SDMMC_DISCARD_ARG : SDMMC_ERASE_ARG;
    const int64_t t0 = esp_timer_get_time();
    esp_err_t err = sdmmc_erase_sectors(card, start_sector, sector_count, arg);
    sd_note_trim(esp_timer_get_time() - t0);
    if (unlikely(err != ESP_OK)) {
        ESP_LOGE(TAG, "sdmmc_erase_sectors failed (%d)", err);
        return RES_ERROR;
    }
    return RES_OK;
}
#endif //FF_USE_TRIM

static DRESULT ff_sdmmc_ioctl (BYTE pdrv, BYTE cmd, void* buff)
{
    sdmmc_card_t* card = s_cards[pdrv];
    assert(card);
    switch(cmd) {
        case CTRL_SYNC:
            return RES_OK;
        case GET_SECTOR_COUNT:
            *((DWORD*) buff) = card->csd.capacity;
            return RES_OK;
        case GET_SECTOR_SIZE:
            *((WORD*) buff) = card->csd.sector_size;
            return RES_OK;
        case GET_BLOCK_SIZE:
            return RES_ERROR;
#if FF_USE_TRIM
        case CTRL_TRIM:
            if (sdmmc_can_trim(card) != ESP_OK) {
                return RES_PARERR;
            }
            return ff_sdmmc_trim (pdrv, *((DWORD*)buff), //start_sector
                    (*((DWORD*)buff + 1) - *((DWORD*)buff) + 1)); //sector_count
#endif //FF_USE_TRIM
    }
    return RES_ERROR;
}

void ff_sdmmc_set_disk_status_check(BYTE pdrv, bool enable)
{
    s_disk_status_check_en[pdrv] = enable;
}

void ff_diskio_register_sdmmc(BYTE pdrv, sdmmc_card_t* card)
{
    static const ff_diskio_impl_t sdmmc_impl = {
        .init = &ff_sdmmc_initialize,
        .status = &ff_sdmmc_status,
        .read = &ff_sdmmc_read,
        .write = &ff_sdmmc_write,
        .ioctl = &ff_sdmmc_ioctl
    };
    s_cards[pdrv] = card;
    s_disk_status_check_en[pdrv] = false;
    ff_diskio_register(pdrv, &sdmmc_impl);
}

BYTE ff_diskio_get_pdrv_card(const sdmmc_card_t* card)
{
    for (int i = 0; i < FF_VOLUMES; i++) {
        if (card == s_cards[i]) {
            return i;
        }
    }
    return 0xff;
}
