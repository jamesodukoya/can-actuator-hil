#include "sbw/alive_counter.hpp"

#include <cstdint>

#include "sbw/can_protocol.hpp"

namespace sbw {

CounterVerdict AliveCounterChecker::check(std::uint8_t counter) {
    const auto received = static_cast<std::uint8_t>(counter & kAliveCounterMask);
    if (!hasReference_) {
        hasReference_ = true;
        lastCounter_ = received;
        return CounterVerdict::Accepted;
    }
    // Modulo-16 distance: 15 -> 0 is a step of 1. The subtraction is done in int, and the mask keeps
    // the low 4 bits of its two's-complement value (0 - 15 = -15 -> 1).
    const auto delta = static_cast<std::uint8_t>((received - lastCounter_) & kAliveCounterMask);
    if (delta == 0u) {
        return CounterVerdict::Repeated;  // reference unchanged, so a stuck sender stays detectable
    }
    lastCounter_ = received;  // accepted frames and jumps both resynchronize
    if (delta <= kMaxAcceptedCounterDelta) {
        return CounterVerdict::Accepted;
    }
    return CounterVerdict::Jumped;
}

void AliveCounterChecker::reset() {
    hasReference_ = false;
}

}  // namespace sbw
