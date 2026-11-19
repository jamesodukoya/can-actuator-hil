#pragma once
#include <array>
#include <cstddef>
#include <cstdint>

#include "sbw/types.hpp"

namespace sbw {

// The STM32G4 FDCAN receive FIFO 0 holds 3 frames (from ST training material; not yet checked against the
// reference manual RM0440). The runtime never hands the step more than that per tick.
inline constexpr std::size_t kMaxReceivedFramesPerTick = 3;
inline constexpr std::size_t kMaxSentFramesPerTick = 2;  // status + heartbeat

struct SteeringInputs {
    SensorReadings sensors{};
    std::array<CanFrame, kMaxReceivedFramesPerTick> receivedFrames{};
    std::uint8_t receivedFrameCount = 0;
    bool hasScheduleOverrun = false;  // the runtime saw two consecutive missed hand-offs
};

struct SteeringOutputs {
    float currentCommandA = 0.0f;
    bool isBridgeEnabled = false;
    std::array<CanFrame, kMaxSentFramesPerTick> frames{};  // status first
    std::uint8_t frameCount = 0;
};

}  // namespace sbw
