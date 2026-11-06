#pragma once
// LE Audio auto-start after the earbuds connect, pure part (no HAL - host test
// test/host/test_bt_audio.cpp). Called from a background service once per second.
//
// The stream used to start only on a manual user action. It now starts by itself. Rules:
// after paired earbuds with ASCS connect, start after a short settle time (GATT and subscriptions ready); a manual
// stop disables the automation until the next connection; a failed start gets one retry, no loop.

#include <cstdint>

namespace app::btaudio {

constexpr uint32_t kSettleMs = 1500;  // from the connection to the first attempt
constexpr uint32_t kRetryMs  = 8000;  // from a failed attempt to the next one
constexpr int kMaxTries      = 2;

struct Inputs {
    uint32_t nowMs      = 0;
    bool enabled        = false;   // bt.leaudio and bt.leauto
    bool connected      = false;   // BtState::Connected
    bool ready          = false;   // paired (encrypted) and earbuds with ASCS
    uint16_t connHandle = 0xFFFF;  // a different handle = new connection
    bool streaming      = false;   // stream is playing or being set up
    bool busy           = false;   // start / stop in progress
};

class AutoStart {
public:
    // true = start the stream now (the caller does it on a background thread).
    bool tick(const Inputs& in);
    // Manual stream stop: the automation starts nothing until the next connection.
    void userStopped();

private:
    bool connected_    = false;
    uint16_t handle_   = 0xFFFF;
    uint32_t nextTry_  = 0;
    int tries_         = 0;
    bool userStopped_  = false;
};

}  // namespace app::btaudio
