#pragma once
#include <cstdint>

#include "sbw/types.hpp"

namespace sbw {

inline constexpr std::uint32_t kHeartbeatId = 0x100;
inline constexpr std::uint32_t kCommandId = 0x110;
inline constexpr std::uint32_t kStatusId = 0x120;
inline constexpr std::uint32_t kDiagId = 0x1F0;
inline constexpr std::uint8_t kFrameLength = 8;
inline constexpr std::uint8_t kAliveCounterMask = 0x0F;
inline constexpr std::uint8_t kHighestModeValue = static_cast<std::uint8_t>(ControlMode::ClearFault);

enum class NodeState : std::uint8_t { Init = 0, Running = 1, Fault = 2 };

struct CommandMessage {
    std::int16_t targetKingpinAngleMrad = 0;
    std::uint16_t currentLimitMilliAmp = 0;
    std::uint8_t controlMode = 0;   // raw value; validated by unpackCommand
    std::uint8_t aliveCounter = 0;  // 0..15
};

enum class CommandDecodeResult : std::uint8_t { Ok, WrongId, WrongLength, CrcMismatch, InvalidMode };

CanFrame packCommand(const CommandMessage& message);
CommandDecodeResult unpackCommand(const CanFrame& frame, CommandMessage& message);

struct StatusMessage {
    std::int16_t kingpinAngleMrad = 0;
    std::int16_t motorSpeedDeciRadPerSec = 0;  // motor shaft, 0.1 rad/s per count
    std::int16_t motorCurrentMilliAmp = 0;
    std::int8_t estLoadTorqueMilliNm = 0;
    std::uint8_t stateAndFaults = 0;  // bits 0-1 active mode, bits 2-7 fault_bits
};

CanFrame packStatus(const StatusMessage& message);
StatusMessage unpackStatus(const CanFrame& frame);  // used by tests and tools; no validation

struct HeartbeatMessage {
    std::uint8_t nodeState = 0;
    std::uint32_t uptimeMs = 0;
    std::uint16_t busVoltageMilliVolt = 0;
};

CanFrame packHeartbeat(const HeartbeatMessage& message);
HeartbeatMessage unpackHeartbeat(const CanFrame& frame);  // used by tests and tools; no validation

// Sent by the runtime, not by SteeringEcu: it reports what only the runtime can measure (step overruns,
// step times, dropped received frames). Counts saturate at the field maximum.
struct DiagMessage {
    std::uint16_t steeringOverruns = 0;
    std::uint16_t steeringStepMaxCycles = 0;  // CPU cycles on the STM32; 0 in SIL
    std::uint16_t currentStepMaxCycles = 0;   // CPU cycles on the STM32; 0 in SIL
    std::uint8_t staleKingpinReads = 0;
    std::uint8_t droppedRxFrames = 0;
};

CanFrame packDiag(const DiagMessage& message);
DiagMessage unpackDiag(const CanFrame& frame);  // used by tests and tools; no validation

// Round to nearest (halves away from zero), clamp to the type's range, NaN -> 0.
std::int16_t saturateToInt16(float value);
std::int8_t saturateToInt8(float value);
std::uint16_t saturateToUint16(float value);

}  // namespace sbw
