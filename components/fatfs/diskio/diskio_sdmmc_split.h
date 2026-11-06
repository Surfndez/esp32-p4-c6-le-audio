/*
 * Cyberdeck: splitting a FATFS request into short SD card transactions (diskio_sdmmc.c).
 * Pure arithmetic without ESP-IDF - host test: components/fatfs/test_host/test_sd_split.cpp.
 *
 * Why: the SDMMC controller is shared with the SDIO link to the C6 coprocessor (Wi-Fi, BT, LE Audio), and the IDF
 * transaction lock (esp_driver_sdmmc sdmmc_transaction.c, s_request_mutex) covers the whole data transfer of one
 * command. FATFS asks for a whole cluster at once (ff.c f_read/f_write: "Clip at cluster boundary"), and IDF with
 * a buffer aligned to 128 B sends it as ONE CMD18/CMD25 (sdmmc_cmd.c sdmmc_read_sectors/sdmmc_write_sectors, the
 * is_aligned branch). An exFAT cluster on an SDXC card is usually 128 kB - at the measured ~6.5 MB/s (32 kB in
 * about 5 ms in the firmware's file server) that is about 20 ms without SDIO, longer on a slower card or with a
 * larger cluster. The C6 reserve is 4 SDU = 40 ms.
 */
#pragma once

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Max sectors (512 B) in one sdmmc_read/write_sectors call: 32 = 16 kB.
 * - about 2.5 ms of controller occupancy at 6.5 MB/s: below a quarter of the SDU interval (10 ms) and 1/16 of the
 *   C6 reserve; SDIO gets in between pieces (the esp_hosted tasks have priority 23, higher than the readers -
 *   releasing the transaction lock hands the controller over to them immediately, FreeRTOS priority inheritance).
 * - 16 kB = the whole IDMAC descriptor ring (sdmmc_host.c: 4 x 4 kB), so a piece goes without adding descriptors.
 * - A multiple of the 16 sectors of the bounce buffer (board support code,
 *   unaligned_multi_block_rw_max_chunk_size): an unaligned buffer went in 8 kB pieces anyway - the split adds no
 *   transaction there.
 * - The piece offset is a multiple of 512 B, so an aligned buffer stays aligned (128 B cache line) and every
 *   piece still goes by DMA straight into the caller's buffer.
 * Cost: for aligned buffers longer than 16 kB (video 64 kB, file transfer 32 kB) one more command (CMD18/CMD25 +
 * stop + CMD13) per 16 kB.
 */
#define FF_SDMMC_MAX_CHUNK_SECTORS 32u

/* One piece: sectors [sector, sector + count) to/from the buffer starting at byte offset. 0 = success, otherwise
 * an error code. */
typedef int (*ff_sdmmc_chunk_io_t)(void* ctx, uint32_t sector, uint32_t count, size_t offset);

/*
 * A request (sector, count) as consecutive pieces of at most max_chunk sectors (0 = one at a time), ascending; a
 * piece starts in the buffer at byte (sector - first sector) * sector_size. The first error ends the split (no
 * further pieces) and is returned to the caller; count = 0 - no call, result 0.
 */
static inline int ff_sdmmc_split_io(uint32_t sector, uint32_t count, uint32_t sector_size, uint32_t max_chunk,
                                    ff_sdmmc_chunk_io_t io, void* ctx)
{
    const uint32_t step = max_chunk != 0 ? max_chunk : 1;
    uint32_t done       = 0;
    while (done < count) {
        const uint32_t left = count - done;
        const uint32_t n    = left < step ? left : step;
        const int err       = io(ctx, sector + done, n, (size_t)done * sector_size);
        if (err != 0) {
            return err;
        }
        done += n;
    }
    return 0;
}

#ifdef __cplusplus
}
#endif
