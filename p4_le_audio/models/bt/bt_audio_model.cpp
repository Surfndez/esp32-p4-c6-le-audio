// LE Audio auto-start, pure part (bt_audio_model.h).
#include "bt_audio_model.h"

namespace app::btaudio {

bool AutoStart::tick(const Inputs& in)
{
    if (!in.connected) {
        connected_ = false;
        return false;
    }
    if (!connected_ || in.connHandle != handle_) {
        connected_   = true;
        handle_      = in.connHandle;
        nextTry_     = in.nowMs + kSettleMs;
        tries_       = 0;
        userStopped_ = false;
    }
    if (!in.enabled || !in.ready || userStopped_ || in.streaming || in.busy || tries_ >= kMaxTries ||
        (int32_t)(in.nowMs - nextTry_) < 0) {
        return false;
    }
    tries_++;
    nextTry_ = in.nowMs + kRetryMs;
    return true;
}

void AutoStart::userStopped()
{
    userStopped_ = true;
}

}  // namespace app::btaudio
