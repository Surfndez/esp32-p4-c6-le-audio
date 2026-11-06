// Host tests: automatic LE Audio start after the earbuds connect (bt/bt_audio_model.cpp).
// Separate program.

#include <bt/bt_audio_model.h>

#include <cstdio>

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

using app::btaudio::AutoStart;
using app::btaudio::Inputs;
using app::btaudio::kRetryMs;
using app::btaudio::kSettleMs;

Inputs conn(uint32_t now, uint16_t handle = 1)
{
    Inputs in;
    in.nowMs      = now;
    in.enabled    = true;
    in.connected  = true;
    in.ready      = true;
    in.connHandle = handle;
    return in;
}

}  // namespace

int main()
{
    std::printf("LE Audio: automatic start\n");

    {
        AutoStart a;
        expect(!a.tick(conn(1000)), "not right after connecting (GATT, subscriptions)");
        expect(!a.tick(conn(1000 + kSettleMs - 1)), "not before the settle time elapses");
        expect(a.tick(conn(1000 + kSettleMs)), "start after the settle time");
        Inputs s = conn(1000 + kSettleMs + 1000);
        s.streaming = true;
        expect(!a.tick(s), "streaming - does nothing");
        s.nowMs += 60000;
        expect(!a.tick(s), "still streaming - nothing");
    }
    {
        AutoStart a;
        a.tick(conn(0));
        expect(a.tick(conn(kSettleMs)), "first attempt");
        expect(!a.tick(conn(kSettleMs + 1000)), "not immediately after a failure");
        expect(a.tick(conn(kSettleMs + kRetryMs)), "one retry");
        expect(!a.tick(conn(kSettleMs + 3 * kRetryMs)), "no loop - stops after two attempts");
        Inputs off = conn(kSettleMs + 4 * kRetryMs);
        off.connected = false;
        a.tick(off);
        a.tick(conn(off.nowMs + 1000, 2));
        expect(a.tick(conn(off.nowMs + 1000 + kSettleMs, 2)), "new connection - tries again");
    }
    {
        AutoStart a;
        a.tick(conn(0));
        expect(a.tick(conn(kSettleMs)), "start");
        a.userStopped();
        expect(!a.tick(conn(kSettleMs + 10 * kRetryMs)), "does not restart by itself after a manual stop");
        Inputs off = conn(kSettleMs + 11 * kRetryMs);
        off.connected = false;
        a.tick(off);
        a.tick(conn(off.nowMs + 10));
        expect(a.tick(conn(off.nowMs + 10 + kSettleMs)), "starts by itself again after reconnecting");
    }
    {
        AutoStart a;
        Inputs in = conn(0);
        in.enabled = false;
        a.tick(in);
        in.nowMs = 10 * kSettleMs;
        expect(!a.tick(in), "disabled in settings - never");
        AutoStart b;
        Inputs u = conn(0);
        u.ready = false;
        b.tick(u);
        u.nowMs = 10 * kSettleMs;
        expect(!b.tick(u), "not without pairing (no encryption)");
        u.ready = true;
        expect(b.tick(u), "start once paired on the same connection");
    }
    {
        AutoStart a;
        a.tick(conn(0));
        Inputs busy = conn(kSettleMs);
        busy.busy = true;
        expect(!a.tick(busy), "start or stop in progress - waits");
        expect(a.tick(conn(kSettleMs + 10)), "start after it finishes");
        a.tick(conn(kSettleMs + 20));
        expect(!a.tick(conn(kSettleMs + 30, 7)) , "different handle - new connection, settle time again");
        expect(a.tick(conn(kSettleMs + 30 + kSettleMs, 7)), "start on the new connection after the settle time");
    }

    std::printf("LE Audio: automatic start: %d OK, %d FAIL\n", g_ok, g_fail);
    return g_fail == 0 ? 0 : 1;
}
