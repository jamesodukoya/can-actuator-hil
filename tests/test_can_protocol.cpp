#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <cstdint>

#include "sbw/can_protocol.hpp"

TEST(CommandFrame, PacksToKnownBytes) {
    RecordProperty("requirements", "REQ-CAN-003");
    const sbw::CommandMessage message{
        .targetKingpinAngleMrad = 100, .currentLimitMilliAmp = 1500, .controlMode = 3, .aliveCounter = 1};

    const sbw::CanFrame frame = sbw::packCommand(message);

    const std::array<std::uint8_t, 8> kExpectedBytes{0x64, 0x00, 0xDC, 0x05, 0x03, 0x01, 0x00, 0x51};
    EXPECT_EQ(frame.id, sbw::kCommandId);
    EXPECT_EQ(frame.length, sbw::kFrameLength);
    EXPECT_EQ(frame.data, kExpectedBytes);
}

TEST(CommandFrame, NegativeTargetIsTwosComplement) {
    const sbw::CommandMessage message{.targetKingpinAngleMrad = -524};

    const sbw::CanFrame frame = sbw::packCommand(message);

    // -524 = 0xFDF4 in two's complement; little-endian puts F4 first.
    EXPECT_EQ(frame.data[0], 0xF4);
    EXPECT_EQ(frame.data[1], 0xFD);
}

TEST(CommandFrame, CounterIsMaskedToFourBits) {
    const sbw::CommandMessage message{.aliveCounter = 0x1F};

    const sbw::CanFrame frame = sbw::packCommand(message);

    EXPECT_EQ(frame.data[5], 0x0F);
}

TEST(CommandFrame, RoundTripsExtremeValues) {
    const sbw::CommandMessage extremes[] = {
        {.targetKingpinAngleMrad = -32768, .currentLimitMilliAmp = 65535, .controlMode = 4, .aliveCounter = 15},
        {.targetKingpinAngleMrad = 32767, .currentLimitMilliAmp = 0, .controlMode = 0, .aliveCounter = 0},
    };
    for (const sbw::CommandMessage& original : extremes) {
        sbw::CommandMessage decoded;

        const sbw::CommandDecodeResult result = sbw::unpackCommand(sbw::packCommand(original), decoded);

        EXPECT_EQ(result, sbw::CommandDecodeResult::Ok);
        EXPECT_EQ(decoded.targetKingpinAngleMrad, original.targetKingpinAngleMrad);
        EXPECT_EQ(decoded.currentLimitMilliAmp, original.currentLimitMilliAmp);
        EXPECT_EQ(decoded.controlMode, original.controlMode);
        EXPECT_EQ(decoded.aliveCounter, original.aliveCounter);
    }
}

TEST(CommandFrame, RejectsWrongId) {
    constexpr std::uint32_t kNeighbourId = 0x111;  // one above the command ID
    sbw::CanFrame frame = sbw::packCommand({.controlMode = 1});
    frame.id = kNeighbourId;
    sbw::CommandMessage decoded;

    EXPECT_EQ(sbw::unpackCommand(frame, decoded), sbw::CommandDecodeResult::WrongId);
}

TEST(CommandFrame, RejectsWrongLength) {
    RecordProperty("requirements", "REQ-CAN-001");
    sbw::CanFrame frame = sbw::packCommand({.controlMode = 1});
    frame.length = 7;
    sbw::CommandMessage decoded;

    EXPECT_EQ(sbw::unpackCommand(frame, decoded), sbw::CommandDecodeResult::WrongLength);
}

TEST(CommandFrame, RejectsCorruptedPayload) {
    RecordProperty("requirements", "REQ-CAN-001");
    sbw::CanFrame frame = sbw::packCommand({.targetKingpinAngleMrad = 100, .controlMode = 3});
    frame.data[0] = static_cast<std::uint8_t>(frame.data[0] ^ 0x01);  // flip bit 0 of byte 0
    const sbw::CommandMessage kSentinel{
        .targetKingpinAngleMrad = 1234, .currentLimitMilliAmp = 4321, .controlMode = 2, .aliveCounter = 9};
    sbw::CommandMessage decoded = kSentinel;

    EXPECT_EQ(sbw::unpackCommand(frame, decoded), sbw::CommandDecodeResult::CrcMismatch);
    EXPECT_EQ(decoded.targetKingpinAngleMrad, kSentinel.targetKingpinAngleMrad);
    EXPECT_EQ(decoded.currentLimitMilliAmp, kSentinel.currentLimitMilliAmp);
    EXPECT_EQ(decoded.controlMode, kSentinel.controlMode);
    EXPECT_EQ(decoded.aliveCounter, kSentinel.aliveCounter);
}

TEST(CommandFrame, RejectsInvalidMode) {
    // packCommand does not validate the mode, so the CRC of this frame is correct.
    const sbw::CanFrame frame = sbw::packCommand({.controlMode = 5});
    sbw::CommandMessage decoded;

    EXPECT_EQ(sbw::unpackCommand(frame, decoded), sbw::CommandDecodeResult::InvalidMode);
}

TEST(Saturate, Int16) {
    EXPECT_EQ(sbw::saturateToInt16(40000.0f), 32767);
    EXPECT_EQ(sbw::saturateToInt16(-40000.0f), -32768);
    EXPECT_EQ(sbw::saturateToInt16(1.5f), 2);
    EXPECT_EQ(sbw::saturateToInt16(-1.5f), -2);
    EXPECT_EQ(sbw::saturateToInt16(0.4f), 0);
    EXPECT_EQ(sbw::saturateToInt16(std::nanf("")), 0);
}

TEST(Saturate, Int8) {
    EXPECT_EQ(sbw::saturateToInt8(200.0f), 127);
    EXPECT_EQ(sbw::saturateToInt8(-200.0f), -128);
    EXPECT_EQ(sbw::saturateToInt8(-5.4f), -5);
}

TEST(Saturate, Uint16) {
    EXPECT_EQ(sbw::saturateToUint16(-3.0f), 0);
    EXPECT_EQ(sbw::saturateToUint16(70000.0f), 65535);
    EXPECT_EQ(sbw::saturateToUint16(12000.4f), 12000);
}

TEST(StatusFrame, PacksToKnownBytes) {
    RecordProperty("requirements", "REQ-CAN-003");
    const sbw::StatusMessage message{.kingpinAngleMrad = -1,
                                     .motorSpeedDeciRadPerSec = 10055,
                                     .motorCurrentMilliAmp = 1500,
                                     .estLoadTorqueMilliNm = -5,
                                     .stateAndFaults = 0x07};

    const sbw::CanFrame frame = sbw::packStatus(message);

    EXPECT_EQ(frame.id, 0x120u);
    EXPECT_EQ(frame.length, 8);
    const std::array<std::uint8_t, 8> expected{0xFF, 0xFF, 0x47, 0x27, 0xDC, 0x05, 0xFB, 0x07};
    EXPECT_EQ(frame.data, expected);
}

TEST(StatusFrame, RoundTripsExtremes) {
    const sbw::StatusMessage message{.kingpinAngleMrad = 32767,
                                     .motorSpeedDeciRadPerSec = -32768,
                                     .motorCurrentMilliAmp = -32768,
                                     .estLoadTorqueMilliNm = -128,
                                     .stateAndFaults = 0xFF};

    const sbw::StatusMessage decoded = sbw::unpackStatus(sbw::packStatus(message));

    EXPECT_EQ(decoded.kingpinAngleMrad, message.kingpinAngleMrad);
    EXPECT_EQ(decoded.motorSpeedDeciRadPerSec, message.motorSpeedDeciRadPerSec);
    EXPECT_EQ(decoded.motorCurrentMilliAmp, message.motorCurrentMilliAmp);
    EXPECT_EQ(decoded.estLoadTorqueMilliNm, message.estLoadTorqueMilliNm);
    EXPECT_EQ(decoded.stateAndFaults, message.stateAndFaults);
}

TEST(HeartbeatFrame, PacksToKnownBytes) {
    RecordProperty("requirements", "REQ-CAN-003");
    const sbw::HeartbeatMessage message{
        .nodeState = 1, .uptimeMs = 0x01020304, .busVoltageMilliVolt = 12000};

    const sbw::CanFrame frame = sbw::packHeartbeat(message);

    EXPECT_EQ(frame.id, 0x100u);
    EXPECT_EQ(frame.length, 8);
    const std::array<std::uint8_t, 8> expected{0x01, 0x04, 0x03, 0x02, 0x01, 0xE0, 0x2E, 0x00};
    EXPECT_EQ(frame.data, expected);
}

TEST(HeartbeatFrame, RoundTripsMaxUptime) {
    const sbw::HeartbeatMessage message{
        .nodeState = 2, .uptimeMs = 4294967295u, .busVoltageMilliVolt = 65535};

    const sbw::HeartbeatMessage decoded = sbw::unpackHeartbeat(sbw::packHeartbeat(message));

    EXPECT_EQ(decoded.nodeState, message.nodeState);
    EXPECT_EQ(decoded.uptimeMs, message.uptimeMs);
    EXPECT_EQ(decoded.busVoltageMilliVolt, message.busVoltageMilliVolt);
}

TEST(DiagFrame, PacksToKnownBytes) {
    RecordProperty("requirements", "REQ-CAN-003");
    const sbw::DiagMessage message{.steeringOverruns = 2,
                                   .steeringStepMaxCycles = 0x1234,
                                   .currentStepMaxCycles = 0x0567,
                                   .staleKingpinReads = 3,
                                   .droppedRxFrames = 1};

    const sbw::CanFrame frame = sbw::packDiag(message);

    EXPECT_EQ(frame.id, 0x1F0u);
    EXPECT_EQ(frame.length, 8);
    const std::array<std::uint8_t, 8> expected{0x02, 0x00, 0x34, 0x12, 0x67, 0x05, 0x03, 0x01};
    EXPECT_EQ(frame.data, expected);
}

TEST(DiagFrame, RoundTripsMaxima) {
    const sbw::DiagMessage message{.steeringOverruns = 65535,
                                   .steeringStepMaxCycles = 65535,
                                   .currentStepMaxCycles = 65535,
                                   .staleKingpinReads = 255,
                                   .droppedRxFrames = 255};

    const sbw::CanFrame frame = sbw::packDiag(message);
    const sbw::DiagMessage decoded = sbw::unpackDiag(frame);

    const std::array<std::uint8_t, 8> allOnes{0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
    EXPECT_EQ(frame.data, allOnes);
    EXPECT_EQ(decoded.steeringOverruns, message.steeringOverruns);
    EXPECT_EQ(decoded.steeringStepMaxCycles, message.steeringStepMaxCycles);
    EXPECT_EQ(decoded.currentStepMaxCycles, message.currentStepMaxCycles);
    EXPECT_EQ(decoded.staleKingpinReads, message.staleKingpinReads);
    EXPECT_EQ(decoded.droppedRxFrames, message.droppedRxFrames);
}
