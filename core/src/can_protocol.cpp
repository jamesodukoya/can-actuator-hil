#include "sbw/can_protocol.hpp"

#include <array>
#include <cmath>
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

void writeUint32Le(std::array<std::uint8_t, 8>& data, std::size_t offset, std::uint32_t value) {
    data[offset] = static_cast<std::uint8_t>(value & 0xFFu);
    data[offset + 1] = static_cast<std::uint8_t>((value >> 8) & 0xFFu);
    data[offset + 2] = static_cast<std::uint8_t>((value >> 16) & 0xFFu);
    data[offset + 3] = static_cast<std::uint8_t>((value >> 24) & 0xFFu);
}

std::uint16_t readUint16Le(const std::array<std::uint8_t, 8>& data, std::size_t offset) {
    return static_cast<std::uint16_t>(data[offset] | (data[offset + 1] << 8));
}

std::uint32_t readUint32Le(const std::array<std::uint8_t, 8>& data, std::size_t offset) {
    return static_cast<std::uint32_t>(data[offset]) |
           (static_cast<std::uint32_t>(data[offset + 1]) << 8) |
           (static_cast<std::uint32_t>(data[offset + 2]) << 16) |
           (static_cast<std::uint32_t>(data[offset + 3]) << 24);
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

CanFrame packStatus(const StatusMessage& message) {
    CanFrame frame;
    frame.id = kStatusId;
    frame.length = kFrameLength;
    writeUint16Le(frame.data, 0, static_cast<std::uint16_t>(message.kingpinAngleMrad));
    writeUint16Le(frame.data, 2, static_cast<std::uint16_t>(message.motorSpeedDeciRadPerSec));
    writeUint16Le(frame.data, 4, static_cast<std::uint16_t>(message.motorCurrentMilliAmp));
    frame.data[6] = static_cast<std::uint8_t>(message.estLoadTorqueMilliNm);
    frame.data[7] = message.stateAndFaults;
    return frame;
}

StatusMessage unpackStatus(const CanFrame& frame) {
    StatusMessage message;
    message.kingpinAngleMrad = static_cast<std::int16_t>(readUint16Le(frame.data, 0));
    message.motorSpeedDeciRadPerSec = static_cast<std::int16_t>(readUint16Le(frame.data, 2));
    message.motorCurrentMilliAmp = static_cast<std::int16_t>(readUint16Le(frame.data, 4));
    message.estLoadTorqueMilliNm = static_cast<std::int8_t>(frame.data[6]);
    message.stateAndFaults = frame.data[7];
    return message;
}

CanFrame packHeartbeat(const HeartbeatMessage& message) {
    CanFrame frame;
    frame.id = kHeartbeatId;
    frame.length = kFrameLength;
    frame.data[0] = message.nodeState;
    writeUint32Le(frame.data, 1, message.uptimeMs);
    writeUint16Le(frame.data, 5, message.busVoltageMilliVolt);
    frame.data[7] = 0;  // reserved
    return frame;
}

HeartbeatMessage unpackHeartbeat(const CanFrame& frame) {
    HeartbeatMessage message;
    message.nodeState = frame.data[0];
    message.uptimeMs = readUint32Le(frame.data, 1);
    message.busVoltageMilliVolt = readUint16Le(frame.data, 5);
    return message;
}

CanFrame packDiag(const DiagMessage& message) {
    CanFrame frame;
    frame.id = kDiagId;
    frame.length = kFrameLength;
    writeUint16Le(frame.data, 0, message.steeringOverruns);
    writeUint16Le(frame.data, 2, message.steeringStepMaxCycles);
    writeUint16Le(frame.data, 4, message.currentStepMaxCycles);
    frame.data[6] = message.staleKingpinReads;
    frame.data[7] = message.droppedRxFrames;
    return frame;
}

DiagMessage unpackDiag(const CanFrame& frame) {
    DiagMessage message;
    message.steeringOverruns = readUint16Le(frame.data, 0);
    message.steeringStepMaxCycles = readUint16Le(frame.data, 2);
    message.currentStepMaxCycles = readUint16Le(frame.data, 4);
    message.staleKingpinReads = frame.data[6];
    message.droppedRxFrames = frame.data[7];
    return message;
}

CanFrame packBuild(const BuildMessage& message) {
    CanFrame frame;
    frame.id = kBuildId;
    frame.length = kFrameLength;
    writeUint32Le(frame.data, 0, message.commitHash);
    frame.data[4] = static_cast<std::uint8_t>(message.buildType);
    frame.data[5] = message.hasUncommittedChanges ? 1 : 0;
    return frame;
}

BuildMessage unpackBuild(const CanFrame& frame) {
    BuildMessage message;
    message.commitHash = readUint32Le(frame.data, 0);
    message.buildType = static_cast<BuildType>(frame.data[4]);
    message.hasUncommittedChanges = frame.data[5] != 0;
    return message;
}

std::int16_t saturateToInt16(float value) {
    constexpr float kMax = 32767.0f;
    constexpr float kMin = -32768.0f;
    if (std::isnan(value)) {
        return 0;
    }
    if (value >= kMax) {
        return 32767;
    }
    if (value <= kMin) {
        return -32768;
    }
    return static_cast<std::int16_t>(std::lround(value));
}

std::int8_t saturateToInt8(float value) {
    constexpr float kMax = 127.0f;
    constexpr float kMin = -128.0f;
    if (std::isnan(value)) {
        return 0;
    }
    if (value >= kMax) {
        return 127;
    }
    if (value <= kMin) {
        return -128;
    }
    return static_cast<std::int8_t>(std::lround(value));
}

std::uint16_t saturateToUint16(float value) {
    constexpr float kMax = 65535.0f;
    constexpr float kMin = 0.0f;
    if (std::isnan(value)) {
        return 0;
    }
    if (value >= kMax) {
        return 65535;
    }
    if (value <= kMin) {
        return 0;
    }
    return static_cast<std::uint16_t>(std::lround(value));
}

}  // namespace sbw
