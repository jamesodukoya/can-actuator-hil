#pragma once
#include <cstdint>

#include "sbw/types.hpp"

namespace sbw {

inline constexpr std::uint32_t kHeartbeatId = 0x100;
inline constexpr std::uint32_t kCommandId = 0x110;
inline constexpr std::uint32_t kStatusId = 0x120;
inline constexpr std::uint8_t kFrameLength = 8;
inline constexpr std::uint8_t kAliveCounterMask = 0x0F;
inline constexpr std::uint8_t kHighestModeValue = static_cast<std::uint8_t>(ControlMode::ClearFault);

struct CommandMessage {
    std::int16_t targetKingpinAngleMrad = 0;
    std::uint16_t currentLimitMilliAmp = 0;
    std::uint8_t controlMode = 0;   // raw value; validated by unpackCommand
    std::uint8_t aliveCounter = 0;  // 0..15
};

enum class CommandDecodeResult : std::uint8_t { Ok, WrongId, WrongLength, CrcMismatch, InvalidMode };

CanFrame packCommand(const CommandMessage& message);
CommandDecodeResult unpackCommand(const CanFrame& frame, CommandMessage& message);

}  // namespace sbw
