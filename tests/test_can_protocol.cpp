#include <gtest/gtest.h>

#include <array>
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
