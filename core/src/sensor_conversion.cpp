#include "sbw/sensor_conversion.hpp"

#include "sbw/types.hpp"

namespace sbw {

float motorAngleRad(std::int32_t encoderCounts, std::int32_t countsPerRev) {
    return static_cast<float>(encoderCounts) * (kTwoPi / static_cast<float>(countsPerRev));
}

float kingpinAngleRad(std::uint16_t rawCount, std::uint16_t zeroCount) {
    constexpr std::int32_t kHalfTurnCounts = kAs5600CountsPerRev / 2;
    std::int32_t countsFromZero = static_cast<std::int32_t>(rawCount) - static_cast<std::int32_t>(zeroCount);
    if (countsFromZero >= kHalfTurnCounts) {
        countsFromZero -= kAs5600CountsPerRev;
    }
    if (countsFromZero < -kHalfTurnCounts) {
        countsFromZero += kAs5600CountsPerRev;
    }
    return static_cast<float>(countsFromZero) * (kTwoPi / static_cast<float>(kAs5600CountsPerRev));
}

}  // namespace sbw
