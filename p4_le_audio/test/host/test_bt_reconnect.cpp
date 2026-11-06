// Host tests: automatic reconnection to saved earbuds (bt/bt_reconnect_model.cpp).
// Separate program.

#include <bt/bt_reconnect_model.h>

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

using app::btreconnect::Action;
using app::btreconnect::Inputs;
using app::btreconnect::kBootMs;
using app::btreconnect::kFirstMs;
using app::btreconnect::kJoinMaxMs;
using app::btreconnect::kJoinMinMs;
using app::btreconnect::kJoinTries;
using app::btreconnect::kJoinSlowMs;
using app::btreconnect::kScanMaxMs;
using app::btreconnect::kScanMinMs;
using app::btreconnect::Reconnect;

// Bluetooth enabled, saved earbuds exist, radio ready, nothing in progress, not connected.
Inputs idle(uint32_t now)
{
    Inputs in;
    in.nowMs    = now;
    in.enabled  = true;
    in.radioOn  = true;
    in.hasBonds = true;
    return in;
}

Inputs connectedAt(uint32_t now, bool setMissing = false)
{
    Inputs in      = idle(now);
    in.connected   = true;
    in.setMissing  = setMissing;
    return in;
}

// First time >= from at which tick returns an action (every 1 s, like the background service); UINT32_MAX = none.
uint32_t firstAction(Reconnect& r, uint32_t from, uint32_t limit, Action* got, Inputs (*mk)(uint32_t))
{
    for (uint32_t t = from; t <= limit; t += 1000) {
        const Action a = r.tick(mk(t));
        if (a != Action::None) {
            *got = a;
            return t;
        }
    }
    *got = Action::None;
    return UINT32_MAX;
}

Inputs joinNeeded(uint32_t now)
{
    return connectedAt(now, true);
}

Inputs connectedFull(uint32_t now)
{
    return connectedAt(now, false);
}

}  // namespace

int main()
{
    std::printf("bt_reconnect\n");
    {
        Reconnect r;
        Inputs in = idle(0);
        in.enabled = false;
        bool none = true;
        for (uint32_t t = 0; t < 120000; t += 1000) {
            in.nowMs = t;
            none     = none && r.tick(in) == Action::None;
        }
        expect(none, "disabled (bt.autocon / bt.leaudio / bt.on) - never anything");
    }
    {
        Reconnect r;
        bool none = true;
        for (uint32_t t = 0; t < 120000; t += 1000) {
            Inputs in   = idle(t);
            in.hasBonds = false;
            none        = none && r.tick(in) == Action::None;
        }
        expect(none, "no saved earbuds - no background scans");
    }
    {
        // Device starts with the radio off: radio first, then the connection
        Reconnect r;
        Inputs in  = idle(0);
        in.radioOn = false;
        Action a   = Action::None;
        uint32_t t = 0;
        for (; t <= kBootMs + 5000 && a == Action::None; t += 1000) {
            in.nowMs = t;
            a        = r.tick(in);
        }
        expect(a == Action::RadioOn, "radio off - radio is switched on first");
        expect(t - 1000 >= kBootMs, "first action not right after boot (network and OTA confirmation come first)");
        Action b = Action::None;
        const uint32_t tb = firstAction(r, t, t + 10000, &b, idle);
        expect(b == Action::Connect && tb != UINT32_MAX, "after the radio is on - connection attempt");
    }
    {
        // Failed attempts: the gap doubles up to kScanMaxMs, a successful one returns to kScanMinMs
        Reconnect r;
        Action a        = Action::None;
        uint32_t t      = firstAction(r, 0, kBootMs + 5000, &a, idle);
        expect(a == Action::Connect, "radio ready - connection attempt");
        uint32_t gap    = kScanMinMs;
        bool grows      = true;
        for (int i = 0; i < 6; i++) {
            const uint32_t end = t + 3000;  // the scan and attempt take time
            r.done(Action::Connect, false, end);
            const uint32_t next = firstAction(r, end + 1000, end + kScanMaxMs + 5000, &a, idle);
            const uint32_t want = gap * 2 > kScanMaxMs ? kScanMaxMs : gap * 2;
            grows               = grows && a == Action::Connect && next >= end + want && next < end + want + 1000;
            gap                 = want;
            t                   = next;
        }
        expect(grows, "failed attempts - gap x2 up to the limit");
        r.done(Action::Connect, true, t + 3000);
        firstAction(r, t + 4000, t + 4000 + kScanMaxMs, &a, connectedFull);
        expect(a == Action::None, "connected, set complete - nothing");
    }
    {
        // Radio did not come up (coprocessor not responding): retry with a gap, not every second
        Reconnect r;
        Inputs in  = idle(0);
        in.radioOn = false;
        int radioOn = 0;
        for (uint32_t t = 0; t < kBootMs + 30000; t += 1000) {
            in.nowMs = t;
            if (r.tick(in) == Action::RadioOn) {
                radioOn++;
                r.done(Action::RadioOn, false, t + 500);
            }
        }
        expect(radioOn >= 2 && radioOn <= 4, "radio does not come up - attempts with a gap");
    }
    {
        // Busy (HAL is scanning / connecting, our attempt in progress): wait, the plan stays
        Reconnect r;
        Inputs in = idle(0);
        in.busy   = true;
        bool none = true;
        for (uint32_t t = 0; t < 30000; t += 1000) {
            in.nowMs = t;
            none     = none && r.tick(in) == Action::None;
        }
        expect(none, "busy - no action");
        expect(r.tick(idle(30000)) == Action::Connect, "once free - attempt immediately (deadline has passed)");
    }
    {
        // Second earbud of the set: attempt kJoinMinMs after connecting, then less often up to kJoinMaxMs
        Reconnect r;
        Action a = Action::None;
        r.tick(idle(0));
        const uint32_t c = 5000;
        r.tick(connectedAt(c, true));
        const uint32_t t1 = firstAction(r, c + 1000, c + kJoinMaxMs, &a, joinNeeded);
        expect(a == Action::JoinSet && t1 >= c + kJoinMinMs, "second one missing - join after kJoinMinMs");
        r.done(Action::JoinSet, false, t1 + 4000);
        const uint32_t t2 = firstAction(r, t1 + 5000, t1 + 4000 + kJoinMaxMs + 2000, &a, joinNeeded);
        expect(a == Action::JoinSet && t2 >= t1 + 4000 + 2 * kJoinMinMs, "failed join - longer gap");
        // After kJoinTries failures stop until the next connection: each attempt is a 4 s RSI scan, which is audible
        // while a stream is playing (review finding: it used to retry every minute without end)
        uint32_t t  = t2;
        int joins   = 2;
        for (int i = 0; i < 10; i++) {
            r.done(Action::JoinSet, false, t + 4000);
            const uint32_t n = firstAction(r, t + 5000, t + 4000 + kJoinMaxMs + 2000, &a, joinNeeded);
            if (n == UINT32_MAX) {
                break;
            }
            joins++;
            t = n;
        }
        expect(joins == kJoinTries, "joining - at most kJoinTries fast attempts per connection");
        // Then rarely, but without end (measured: a second earbud taken out of its case after 4 attempts never joined)
        r.done(Action::JoinSet, false, t + 4000);
        expect(firstAction(r, t + 5000, t + 4000 + kJoinSlowMs - 1000, &a, joinNeeded) == UINT32_MAX,
               "after kJoinTries - nothing before kJoinSlowMs");
        const uint32_t ts = firstAction(r, t + 4000 + kJoinSlowMs - 1000, t + 4000 + kJoinSlowMs + 2000, &a,
                                        joinNeeded);
        expect(ts != UINT32_MAX && a == Action::JoinSet, "after kJoinTries - slow attempt every kJoinSlowMs");
        r.done(Action::JoinSet, false, ts + 4000);
        const uint32_t ts2 = firstAction(r, ts + 5000, ts + 4000 + kJoinSlowMs + 2000, &a, joinNeeded);
        expect(ts2 != UINT32_MAX && ts2 >= ts + 4000 + kJoinSlowMs - 1000, "slow attempts continue every kJoinSlowMs");
        t = ts2;
        r.tick(idle(t + 200000));  // earbuds disappeared
        r.tick(connectedAt(t + 201000, true));
        firstAction(r, t + 202000, t + 202000 + kJoinMaxMs, &a, joinNeeded);
        expect(a == Action::JoinSet, "new connection - joining starts over");
    }
    {
        // Our attempt in progress (busy): a momentary "connected" without encryption and our disconnect must not reset
        // the gap - otherwise it retries every ~9 s instead of ever less often (review finding)
        Reconnect r;
        Action a   = Action::None;
        uint32_t t = firstAction(r, 0, kBootMs + 5000, &a, idle);
        expect(a == Action::Connect, "first attempt");
        uint32_t gap  = kScanMinMs;
        bool backsOff = true;
        for (int i = 0; i < 4; i++) {
            for (uint32_t k = 1; k <= 6; k++) {  // btConnect: connected for a few s (pairing), then our disconnect
                Inputs in = k <= 4 ? connectedAt(t + k * 1000) : idle(t + k * 1000);
                in.busy   = true;
                r.tick(in);
            }
            const uint32_t end = t + 6500;
            r.done(Action::Connect, false, end);
            const uint32_t next = firstAction(r, end + 500, end + kScanMaxMs + 5000, &a, idle);
            gap                 = gap * 2 > kScanMaxMs ? kScanMaxMs : gap * 2;
            backsOff            = backsOff && a == Action::Connect && next >= end + gap;
            t                   = next;
        }
        expect(backsOff, "connection without encryption - the gap keeps growing");
    }
    {
        // Disconnect (earbuds disappeared): quick retry, not after the whole gap built up by earlier failures
        Reconnect r;
        Action a        = Action::None;
        uint32_t t      = firstAction(r, 0, kBootMs + 5000, &a, idle);
        r.done(Action::Connect, false, t + 3000);
        r.done(Action::Connect, false, t + 20000);
        r.tick(connectedAt(t + 30000));
        const uint32_t lost = t + 90000;
        const uint32_t next = firstAction(r, lost, lost + kScanMaxMs, &a, idle);
        expect(a == Action::Connect && next >= lost + kFirstMs && next <= lost + kFirstMs + 1000,
               "after a disconnect - attempt after kFirstMs");
    }
    {
        // Manual disconnect in the app: the automatic logic stays idle until the next connection
        Reconnect r;
        r.tick(connectedAt(0));
        r.userDisconnected();
        expect(r.paused(), "manual disconnect - automatic logic paused");
        Action a = Action::None;
        firstAction(r, 1000, 200000, &a, idle);
        expect(a == Action::None, "manual disconnect - no automatic action");
        r.tick(connectedAt(210000));  // the user connected manually
        firstAction(r, 211000, 211000 + kScanMaxMs, &a, idle);
        expect(a == Action::Connect, "after a manual connection and disappearance - automatic logic works again");
    }
    std::printf("bt_reconnect: %d ok, %d failed\n", g_ok, g_fail);
    return g_fail == 0 ? 0 : 1;
}
