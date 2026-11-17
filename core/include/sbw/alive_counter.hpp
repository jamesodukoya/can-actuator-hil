#pragma once
#include <cstdint>

namespace sbw {

enum class CounterVerdict : std::uint8_t { Accepted, Repeated, Jumped };

class AliveCounterChecker {
 public:
    CounterVerdict check(std::uint8_t counter);
    void reset();

 private:
    bool hasReference_ = false;
    std::uint8_t lastCounter_ = 0;
};

inline constexpr std::uint8_t kMaxAcceptedCounterDelta = 3;  // tolerates up to 2 lost frames

}  // namespace sbw
