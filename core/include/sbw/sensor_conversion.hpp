#pragma once
#include <cstdint>

namespace sbw {

inline constexpr std::int32_t kAs5600CountsPerRev = 4096;

// counts * 2pi / countsPerRev
float motorAngleRad(std::int32_t encoderCounts, std::int32_t countsPerRev);
// Angle from the magnet's zero position, wrapped to [-pi, pi).
float kingpinAngleRad(std::uint16_t rawCount, std::uint16_t zeroCount);

}  // namespace sbw
