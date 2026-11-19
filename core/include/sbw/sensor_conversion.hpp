#pragma once
#include <cstdint>

namespace sbw {

inline constexpr std::int32_t kAs5600CountsPerRev = 4096;

// counts * 2pi / countsPerRev
float motorAngleRad(std::int32_t encoderCounts, std::int32_t countsPerRev);
// Angle from the magnet's zero position, wrapped to [-pi, pi).
float kingpinAngleRad(std::uint16_t rawCount, std::uint16_t zeroCount);

// Two ADC samples per PWM period: one in the drive phase and one in the brake phase of the same period.
struct CurrentSenseSample {
    std::uint16_t driveCounts = 0;
    std::uint16_t brakeCounts = 0;
};

// Pololu DRV8874 carrier CS ~1.1 V/A into a 12-bit ADC with a 3.3 V reference.
inline constexpr float kCurrentSenseAmpsPerCount = 3.3f / 4096.0f / 1.1f;
// A drive phase shorter than 3 us of the 50 us period cannot be sampled: the sense output has a 1.6 us delay
// and the ADC needs time on top of that (estimate, not measured).
inline constexpr float kMinSenseDuty = 0.06f;

// Turns the two samples into a signed current. The drive sample is the full current when it flows with the
// bridge's PH direction and zero when it flows against it; the brake sample is the full current either way.
class CurrentSenseDecoder {
 public:
    float decode(const CurrentSenseSample& sample, float appliedDuty);  // amps, signed

 private:
    float directionSign_ = 1.0f;  // last known direction relative to positive current
};

}  // namespace sbw
