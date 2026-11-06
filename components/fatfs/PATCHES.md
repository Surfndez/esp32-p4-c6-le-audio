# fatfs (ESP-IDF v5.5.5) - patched copy

Copy of `$IDF_PATH/components/fatfs` from the `espressif/idf:v5.5.5` image without its test directories. A
component in the project's component directories takes precedence over the ESP-IDF one of the same name. License:
Apache-2.0 (Espressif) and the FatFs license (ChaN) as in ESP-IDF.

Changes (search for "Cyberdeck" in the code - the firmware they were made for):

1. `src/ffconf.h` - `FF_FS_EXFAT 1`. ESP-IDF 5.5 has no Kconfig option for it (hard-wired 0), and SDXC cards
   (64 GB and more) come formatted as exFAT - without it the mount fails with FatFs error 13 (FR_NO_FILESYSTEM).
   Needs long file names (`CONFIG_FATFS_LFN_HEAP=y`); exFAT buffers go through `ff_memalloc`, i.e. to PSRAM with
   `CONFIG_FATFS_ALLOC_PREFER_EXTRAM=y`.
2. `diskio/diskio_sdmmc.c` (+ `diskio_sdmmc.h`, `esp_timer` in `CMakeLists.txt`) - SD transfer statistics:
   `ff_sdmmc_cyberdeck_stats()` = longest `sdmmc_read/write_sectors` call since boot, number of calls over 10 ms,
   largest FATFS request in sectors and the longest TRIM. The SDMMC controller is shared with the C6 SDIO link, and
   the IDF transaction lock covers the whole data transfer of one command - that long Wi-Fi, BT and LE Audio wait.
3. `diskio/diskio_sdmmc.c` + `diskio/diskio_sdmmc_split.h` - SD reads and writes in pieces of at most 32 sectors
   (**16 kB**) per `sdmmc_read/write_sectors` call. FATFS asks for a whole cluster at once and IDF sends an aligned
   buffer as one CMD18/CMD25: a 128 kB exFAT cluster is about 20 ms without SDIO at the measured ~6.5 MB/s, while the
   C6 holds 40 ms of LE Audio. The esp_hosted SDIO tasks get the controller between the pieces. The split
   arithmetic is tested on the host: `test_host/test_sd_split.cpp` (run by `p4_le_audio/test/host/run.sh`).

Limitation: `off_t` in ESP-IDF newlib is 32 bits - `stat()` of a file over 2 GiB reports a wrong size. Streaming
reads and writes of large files work.
