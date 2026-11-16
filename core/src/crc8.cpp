#include "sbw/crc8.hpp"

namespace sbw {

std::uint8_t crc8SaeJ1850(std::span<const std::uint8_t> bytes) {
    constexpr std::uint8_t kPolynomial = 0x1D;
    constexpr std::uint8_t kInitialValue = 0xFF;
    constexpr std::uint8_t kFinalXor = 0xFF;
    std::uint8_t crc = kInitialValue;
    for (const std::uint8_t byte : bytes) {
        crc = static_cast<std::uint8_t>(crc ^ byte);
        for (int bit = 0; bit < 8; ++bit) {
            const bool isTopBitSet = (crc & 0x80u) != 0u;
            crc = static_cast<std::uint8_t>(crc << 1);
            if (isTopBitSet) {
                crc = static_cast<std::uint8_t>(crc ^ kPolynomial);
            }
        }
    }
    return static_cast<std::uint8_t>(crc ^ kFinalXor);
}

}  // namespace sbw
