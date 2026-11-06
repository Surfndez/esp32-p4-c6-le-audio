// Cyberdeck - host tests: splitting FATFS requests into short SD card transactions
// (components/fatfs/diskio/diskio_sdmmc_split.h) - the whole range in order, every piece <= the limit, buffer
// offsets, piece alignment, the first error ends the split, no extra transactions for short requests.

#include <diskio_sdmmc_split.h>

#include <cstdint>
#include <cstdio>
#include <vector>

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

struct Chunk {
    uint32_t sector, count;
    size_t offset;
};

// Mock card: records every call; failAt = number of the call (from 0) that returns an error, -1 = none
struct FakeCard {
    std::vector<Chunk> calls;
    int failAt = -1;
    int err    = 0x107;  // like ESP_ERR_TIMEOUT
};

int fakeIo(void* ctx, uint32_t sector, uint32_t count, size_t offset)
{
    auto* c = static_cast<FakeCard*>(ctx);
    c->calls.push_back({sector, count, offset});
    return (int)c->calls.size() - 1 == c->failAt ? c->err : 0;
}

// Pieces cover [sector, sector + count) exactly once, ascending, each <= max, offset consistent with the sector
bool coversExactly(const FakeCard& c, uint32_t sector, uint32_t count, uint32_t max, uint32_t ss)
{
    uint32_t next = sector;
    for (const Chunk& k : c.calls) {
        if (k.sector != next || k.count == 0 || k.count > max || k.offset != (size_t)(k.sector - sector) * ss) {
            return false;
        }
        next += k.count;
    }
    return next == sector + count;
}

void testWholeCluster()
{
    std::printf("exFAT cluster 128 kB (256 sectors) in pieces of 32\n");
    FakeCard c;
    const int r = ff_sdmmc_split_io(1000, 256, 512, FF_SDMMC_MAX_CHUNK_SECTORS, fakeIo, &c);
    expect(r == 0, "result 0");
    expect(c.calls.size() == 8, "8 transactions of 16 kB");
    expect(coversExactly(c, 1000, 256, FF_SDMMC_MAX_CHUNK_SECTORS, 512), "whole range once, in order, <= 32");
}

void testRemainder()
{
    std::printf("remainder at the end: 70 = 32 + 32 + 6\n");
    FakeCard c;
    expect(ff_sdmmc_split_io(7, 70, 512, 32, fakeIo, &c) == 0, "result 0");
    expect(c.calls.size() == 3, "3 transactions");
    expect(c.calls.size() == 3 && c.calls[2].count == 6 && c.calls[2].sector == 71, "last piece 6 sectors from 71");
    expect(coversExactly(c, 7, 70, 32, 512), "whole range");
}

void testShortRequestUnchanged()
{
    std::printf("short requests (FAT sector, directory, 16 kB cluster) - one call as before the split\n");
    const uint32_t sizes[] = {1, 2, 31, 32};
    for (uint32_t n : sizes) {
        FakeCard c;
        expect(ff_sdmmc_split_io(123456, n, 512, 32, fakeIo, &c) == 0, "result 0");
        expect(c.calls.size() == 1 && c.calls[0].sector == 123456 && c.calls[0].count == n && c.calls[0].offset == 0,
               "one call with the whole request");
    }
}

void testEmpty()
{
    std::printf("count = 0 - nothing\n");
    FakeCard c;
    expect(ff_sdmmc_split_io(5, 0, 512, 32, fakeIo, &c) == 0, "result 0");
    expect(c.calls.empty(), "no calls");
}

void testZeroLimitIsOneSector()
{
    std::printf("limit 0 - one sector at a time (no endless loop)\n");
    FakeCard c;
    expect(ff_sdmmc_split_io(10, 4, 512, 0, fakeIo, &c) == 0, "result 0");
    expect(c.calls.size() == 4, "4 calls");
    expect(coversExactly(c, 10, 4, 1, 512), "one at a time, in order");
}

void testFirstErrorStops()
{
    std::printf("error in the second piece ends the split\n");
    FakeCard c;
    c.failAt  = 1;
    const int r = ff_sdmmc_split_io(0, 256, 512, 32, fakeIo, &c);
    expect(r == c.err, "the error code from the piece is returned to the caller");
    expect(c.calls.size() == 2, "no more transactions after the error");
}

void testAlignmentKept()
{
    std::printf("pieces of an aligned buffer stay aligned to the 128 B cache line\n");
    FakeCard c;
    expect(ff_sdmmc_split_io(3, 200, 512, 32, fakeIo, &c) == 0, "result 0");
    bool aligned = !c.calls.empty();
    for (const Chunk& k : c.calls) {
        aligned = aligned && k.offset % 128 == 0;
    }
    expect(aligned, "every offset a multiple of 128 B");
}

void testLargeSectorSize()
{
    std::printf("4096 B sector (offsets in bytes)\n");
    FakeCard c;
    expect(ff_sdmmc_split_io(0, 10, 4096, 4, fakeIo, &c) == 0, "result 0");
    expect(coversExactly(c, 0, 10, 4, 4096), "4 + 4 + 2, offsets every 16 kB");
}

}  // namespace

int main()
{
    testWholeCluster();
    testRemainder();
    testShortRequestUnchanged();
    testEmpty();
    testZeroLimitIsOneSector();
    testFirstErrorStops();
    testAlignmentKept();
    testLargeSectorSize();
    std::printf("sd_split: %d OK, %d FAIL\n", g_ok, g_fail);
    return g_fail == 0 ? 0 : 1;
}
