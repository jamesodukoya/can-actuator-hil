#include "sbw/can_protocol.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <span>

#include "sbw/crc8.hpp"

namespace sbw {
namespace {

constexpr std::size_t kCrcByteIndex = 7;

// Little-endian: the low byte goes first, matching the DBC `@1` notation.
void writeUint16Le(std::array<std::uint8_t, 8>& data, std::size_t offset, std::uint16_t value) {
    data[offset] = static_cast<std::uint8_t>(value & 0xFFu);
    data[offset + 1] = static_cast<std::uint8_t>(value >> 8);
}

std::uint16_t readUint16Le(const std::array<std::uint8_t, 8>& data, std::size_t offset) {
    return static_cast<std::uint16_t>(data[offset] | (data[offset + 1] << 8));
}

}  // namespace

CanFrame packCommand(const CommandMessage& message) {
    CanFrame frame;
    frame.id = kCommandId;
    frame.length = kFrameLength;
    writeUint16Le(frame.data, 0, static_cast<std::uint16_t>(message.targetKingpinAngleMrad));
    writeUint16Le(frame.data, 2, message.currentLimitMilliAmp);
    frame.data[4] = message.controlMode;
    frame.data[5] = message.aliveCounter & kAliveCounterMask;
    frame.data[6] = 0;  // reserved
    frame.data[kCrcByteIndex] = crc8SaeJ1850(std::span(frame.data).first(kCrcByteIndex));
    return frame;
}

CommandDecodeResult unpackCommand(const CanFrame& frame, CommandMessage& message) {
    if (frame.id != kCommandId) {
        return CommandDecodeResult::WrongId;
    }
    if (frame.length != kFrameLength) {
        return CommandDecodeResult::WrongLength;
    }
    const std::uint8_t expectedCrc = crc8SaeJ1850(std::span(frame.data).first(kCrcByteIndex));
    if (expectedCrc != frame.data[kCrcByteIndex]) {
        return CommandDecodeResult::CrcMismatch;
    }
    if (frame.data[4] > kHighestModeValue) {
        return CommandDecodeResult::InvalidMode;
    }

    // uint16_t -> int16_t is defined modulo 2^16 in C++20 (two's complement is required), so 0xFDF4 becomes -524.
    message.targetKingpinAngleMrad = static_cast<std::int16_t>(readUint16Le(frame.data, 0));
    message.currentLimitMilliAmp = readUint16Le(frame.data, 2);
    message.controlMode = frame.data[4];
    message.aliveCounter = frame.data[5] & kAliveCounterMask;
    return CommandDecodeResult::Ok;
}

}  // namespace sbw
