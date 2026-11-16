#pragma once
#include <array>
#include <cstdint>

namespace sbw {

enum class ControlMode : std::uint8_t {
    Off = 0,
    MotorOnly = 1,
    KingpinOnly = 2,
    DualLoop = 3,
    ClearFault = 4,
};

// Bit positions match byte 7 of SBW_Status; bits 0-1 of that byte hold the active mode.
namespace fault_bits {
inline constexpr std::uint8_t kJam = static_cast<std::uint8_t>(1u << 2);
inline constexpr std::uint8_t kDisagree = static_cast<std::uint8_t>(1u << 3);
inline constexpr std::uint8_t kCrc = static_cast<std::uint8_t>(1u << 4);
inline constexpr std::uint8_t kCounter = static_cast<std::uint8_t>(1u << 5);
inline constexpr std::uint8_t kTimeout = static_cast<std::uint8_t>(1u << 6);
// Stale kingpin sensor or steering-step overrun; SBW_Diag tells which (plan G2 D23).
inline constexpr std::uint8_t kInternal = static_cast<std::uint8_t>(1u << 7);
}  // namespace fault_bits

struct CanFrame {
    std::uint32_t id = 0;
    std::uint8_t length = 0;
    std::array<std::uint8_t, 8> data{};
};

// What the runtime copies for the steering step at slot 0 (plan G2 D21).
struct SensorReadings {
    std::int32_t motorEncoderCounts = 0;
    std::uint16_t as5600RawCount = 0;
    bool isKingpinReadingFresh = true;  // false: the last AS5600 read failed; the raw count is old
    float motorCurrentA = 0.0f;         // decoded current-sense value
    float busVoltageV = 0.0f;
};

inline constexpr float kTwoPi = 6.283185307f;
inline constexpr float kControlPeriodSec = 0.001f;   // 1 kHz steering step
inline constexpr float kCurrentPeriodSec = 0.0001f;  // 10 kHz current loop

}  // namespace sbw
