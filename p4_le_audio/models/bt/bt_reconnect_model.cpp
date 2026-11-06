// Automatic reconnection of saved earbuds, pure part (bt_reconnect_model.h).
#include "bt_reconnect_model.h"

namespace app::btreconnect {

namespace {

bool due(uint32_t now, uint32_t at)
{
    return (int32_t)(now - at) >= 0;
}

uint32_t doubled(uint32_t gap, uint32_t max)
{
    return gap >= max / 2 ? max : gap * 2;
}

}  // namespace

Action Reconnect::tick(const Inputs& in)
{
    if (!in.enabled) {
        started_ = false;
        return Action::None;
    }
    if (!started_) {
        started_     = true;
        nextConnect_ = in.nowMs + kBootMs;
        connectGap_  = kScanMinMs;
    }
    // During an attempt (ours or the HAL's) we do not evaluate the connection state: btConnect is "connected" already
    // while encrypting and pairing, and a failed attempt disconnects - that is not a new connection or lost earbuds
    if (in.busy) {
        return Action::None;
    }
    if (in.connected) {
        if (!wasConnected_) {
            // New connection (automation or user): a manual disconnect no longer applies
            wasConnected_ = true;
            userOff_      = false;
            connectGap_   = kScanMinMs;
            joinGap_      = kJoinMinMs;
            joinTries_    = 0;
            nextJoin_     = in.nowMs + kJoinMinMs;
        }
        if (!in.setMissing || !due(in.nowMs, nextJoin_)) {
            return Action::None;
        }
        joinTries_++;
        nextJoin_ = in.nowMs + (joinTries_ > kJoinTries ? kJoinSlowMs : joinGap_);
        return Action::JoinSet;
    }
    if (wasConnected_) {
        // Earbuds disappeared (range, case): a quick attempt, not after the gap built up from earlier failures
        wasConnected_ = false;
        connectGap_   = kScanMinMs;
        nextConnect_  = in.nowMs + kFirstMs;
    }
    if (userOff_ || !in.hasBonds || !due(in.nowMs, nextConnect_)) {
        return Action::None;
    }
    if (!in.radioOn) {
        nextConnect_ = in.nowMs;  // right after the radio is turned on (busy holds it until the end)
        return Action::RadioOn;
    }
    nextConnect_ = in.nowMs + connectGap_;
    return Action::Connect;
}

void Reconnect::done(Action a, bool ok, uint32_t nowMs)
{
    if (a == Action::Connect) {
        connectGap_  = ok ? kScanMinMs : doubled(connectGap_, kScanMaxMs);
        nextConnect_ = nowMs + connectGap_;
    } else if (a == Action::RadioOn && !ok) {
        connectGap_  = doubled(connectGap_, kScanMaxMs);
        nextConnect_ = nowMs + connectGap_;
    } else if (a == Action::JoinSet) {
        joinGap_   = ok ? kJoinMinMs : doubled(joinGap_, kJoinMaxMs);
        joinTries_ = ok ? 0 : joinTries_;
        // After kJoinTries quick attempts, scan every kJoinSlowMs (a 4 s scan is audible while streaming)
        nextJoin_  = nowMs + (joinTries_ >= kJoinTries ? kJoinSlowMs : joinGap_);
    }
}

void Reconnect::userDisconnected()
{
    userOff_ = true;
}

}  // namespace app::btreconnect
