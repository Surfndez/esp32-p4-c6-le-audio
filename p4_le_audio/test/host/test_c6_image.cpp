// Host test: header check of a C6 firmware image before OTA (models/bt/c6_image_model.cpp).

#include <bt/c6_image_model.h>

#include <cstdio>
#include <cstring>
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

// Header layout of an esp_image built by ESP-IDF for the ESP32-C6 with an app descriptor
std::vector<uint8_t> image(const char* version = "1.4.85")
{
    std::vector<uint8_t> b(4096, 0);
    b[0]  = 0xE9;
    b[12] = 0x0D;  // chip_id ESP32-C6
    b[13] = 0x00;
    b[23] = 1;     // hash_appended
    const uint8_t magic[4] = {0x32, 0x54, 0xCD, 0xAB};
    std::memcpy(&b[32], magic, 4);
    std::strncpy(reinterpret_cast<char*>(&b[48]), version, 32);
    std::strncpy(reinterpret_cast<char*>(&b[80]), "network_adapter", 32);
    return b;
}

bool ok(const std::vector<uint8_t>& b, size_t len, std::string* versionOut = nullptr)
{
    std::string v;
    const char* err = nullptr;
    const bool r    = app::c6image::parse(b.data(), len, v, err);
    if (versionOut != nullptr) {
        *versionOut = v;
    }
    if (!r && err == nullptr) {
        std::printf("  FAIL rejected without a reason\n");
        g_fail++;
    }
    return r;
}

}  // namespace

int main()
{
    std::string v;
    expect(ok(image(), 4096, &v) && v == "1.4.85", "valid image gives its version");

    auto b = image();
    b[0] = 0xE8;
    expect(!ok(b, b.size()), "wrong magic rejected");

    b = image();
    b[12] = 0x05;  // ESP32-C3
    expect(!ok(b, b.size()), "image for another chip rejected");

    b = image();
    b[23] = 0;
    expect(!ok(b, b.size()), "image without appended SHA-256 rejected");

    b = image();
    b[32] = 0;
    expect(!ok(b, b.size()), "image without app descriptor rejected");

    b = image();
    std::strncpy(reinterpret_cast<char*>(&b[80]), "other_project", 32);
    expect(!ok(b, b.size()), "image of another project rejected");

    expect(!ok(image(), app::c6image::kHeaderBytes - 1), "truncated header rejected");
    expect(!ok(image(), app::c6image::kMaxImage + 1), "image larger than the OTA slot rejected");
    expect(ok(image(), app::c6image::kMaxImage), "image exactly the slot size accepted");

    b = image("1.4.85-with-a-version-string-of-32");  // 32 bytes, no terminating NUL in the descriptor
    expect(ok(b, b.size(), &v) && v.size() == 32, "32-byte version without NUL stays within the field");

    std::printf("c6_image: %d OK, %d FAIL\n", g_ok, g_fail);
    return g_fail == 0 ? 0 : 1;
}
