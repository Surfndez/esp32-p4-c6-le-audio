#pragma once
// Automatic reconnection of saved earbuds, pure part (no HAL - host test
// test/host/test_bt_reconnect.cpp). Called from a background service once per second.
//
// Like Android: earbuds taken out of the case reconnect by themselves, without pairing mode, and the second earbud
// of a set also rejoins when it disappears for a moment. Earbuds outside the case advertise from a private
// address, which we resolve with the saved IRK (HAL: BtDevice_t::bonded). Rules: a short scan and connect, backing
// off after failures (saves radio and battery), a quick retry after a disconnect; a scan while streaming glitches
// the audio (shared SDIO), so the second earbud joins rarely; a manual disconnect pauses it until the next connection.

#include <cstdint>

namespace app::btreconnect {

constexpr uint32_t kBootMs    = 20000;  // after device start: network and OTA confirmation go first
constexpr uint32_t kFirstMs   = 2000;   // after a disconnect: first attempt
constexpr uint32_t kScanMinMs = 5000;   // gap between connect attempts, x2 after each failure...
constexpr uint32_t kScanMaxMs = 60000;  // ...up to this limit
constexpr uint32_t kJoinMinMs = 10000;  // second earbud: gap between attempts (4 s RSI scan while streaming)
constexpr uint32_t kJoinMaxMs = 60000;
constexpr int kJoinTries      = 4;      // quick attempts to join the second earbud per connection (scan is audible)
constexpr uint32_t kJoinSlowMs = 180000;  // then rarely, but endlessly: the second may leave the case later

enum class Action : uint8_t {
    None,
    RadioOn,  // turn the radio on (bt.on set, coprocessor not started yet)
    Connect,  // short scan and connect to the first recognised earbud
    JoinSet,  // join the missing earbud of the set (RSI scan)
};

struct Inputs {
    uint32_t nowMs  = 0;
    bool enabled    = false;  // bt.on, bt.leaudio and bt.autocon
    bool radioOn    = false;  // BT coprocessor started
    bool hasBonds   = false;  // saved earbud keys exist
    bool connected  = false;  // main connection
    bool setMissing = false;  // set of >= 2 earbuds, the second not connected and not joining
    bool busy       = false;  // HAL is scanning / connecting, or our attempt is still running
};

class Reconnect {
public:
    // Action to perform now (the caller runs it on a background thread and reports the result via done).
    Action tick(const Inputs& in);
    // Action result (time of completion): the gap to the next attempt counts from its end.
    void done(Action a, bool ok, uint32_t nowMs);
    // The user disconnected the earbuds in the UI, or the earbuds rejected the saved key (they need pairing):
    // the automation waits until the user connects them.
    void userDisconnected();
    // Paused (userDisconnected) until the next connection - the attempt thread checks it right before connecting.
    bool paused() const { return userOff_; }

private:
    bool started_      = false;
    bool wasConnected_ = false;
    bool userOff_      = false;
    uint32_t nextConnect_ = 0;
    uint32_t connectGap_  = kScanMinMs;
    uint32_t nextJoin_    = 0;
    uint32_t joinGap_     = kJoinMinMs;
    int joinTries_        = 0;  // attempts to join the second earbud in this connection
};

}  // namespace app::btreconnect
