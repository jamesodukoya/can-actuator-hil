#include "sbw/sensor_conversion.hpp"

#include <cmath>

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

float CurrentSenseDecoder::decode(const CurrentSenseSample& sample, float appliedDuty) {
    const float phaseSign = appliedDuty >= 0.0f ? 1.0f : -1.0f;
    const bool isDrivePhaseLongEnough = std::fabs(appliedDuty) >= kMinSenseDuty;
    if (isDrivePhaseLongEnough) {
        // Halfway between "both samples equal" and "drive sample zero" leaves the most margin for noise on
        // either side. The 2u keeps the product in unsigned int, where uint16_t would overflow.
        const bool flowsWithPh = 2u * sample.driveCounts >= sample.brakeCounts;
        directionSign_ = flowsWithPh ? phaseSign : -phaseSign;
    }
    return directionSign_ * static_cast<float>(sample.brakeCounts) * kCurrentSenseAmpsPerCount;
}

}  // namespace sbw
