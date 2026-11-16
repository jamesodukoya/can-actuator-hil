#include <gtest/gtest.h>

#include <array>
#include <cstddef>
#include <cstdint>
#include <span>

#include "sbw/crc8.hpp"

TEST(Crc8, MatchesStandardCheckValue) {
    RecordProperty("requirements", "REQ-CAN-001");
    const std::array<std::uint8_t, 9> kCheckInput{'1', '2', '3', '4', '5', '6', '7', '8', '9'};
    EXPECT_EQ(sbw::crc8SaeJ1850(kCheckInput), 0x4B);
}

TEST(Crc8, SevenZeroBytes) {
    const std::array<std::uint8_t, 7> kZeros{};
    EXPECT_EQ(sbw::crc8SaeJ1850(kZeros), 0x0A);
}

TEST(Crc8, SampleCommandBytes) {
    // Target 100 mrad, current limit 1500 mA, DUAL_LOOP, alive counter 1.
    const std::array<std::uint8_t, 7> kCommandBytes{0x64, 0x00, 0xDC, 0x05, 0x03, 0x01, 0x00};
    EXPECT_EQ(sbw::crc8SaeJ1850(kCommandBytes), 0x51);
}

TEST(Crc8, DetectsEverySingleBitFlip) {
    RecordProperty("requirements", "REQ-CAN-001");
    constexpr std::size_t kPayloadLength = 7;
    std::array<std::uint8_t, 8> frame{0x64, 0x00, 0xDC, 0x05, 0x03, 0x01, 0x00, 0x51};
    for (std::size_t bitIndex = 0; bitIndex < frame.size() * 8; ++bitIndex) {
        const std::size_t byteIndex = bitIndex / 8;
        const auto bitMask = static_cast<std::uint8_t>(1u << (bitIndex % 8));
        frame[byteIndex] = static_cast<std::uint8_t>(frame[byteIndex] ^ bitMask);
        const std::uint8_t recomputedCrc = sbw::crc8SaeJ1850(std::span(frame).first(kPayloadLength));
        EXPECT_NE(recomputedCrc, frame[kPayloadLength]) << "undetected flip of bit " << bitIndex;
        frame[byteIndex] = static_cast<std::uint8_t>(frame[byteIndex] ^ bitMask);  // restore the frame
    }
}

TEST(Crc8, ZeroLengthReturnsInitXorFinal) {
    // No bytes: the register keeps its initial 0xFF and the final XOR with 0xFF turns it into 0x00.
    EXPECT_EQ(sbw::crc8SaeJ1850(std::span<const std::uint8_t>{}), 0x00);
}
