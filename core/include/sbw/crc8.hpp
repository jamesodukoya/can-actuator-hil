#pragma once
#include <cstdint>
#include <span>

namespace sbw {

// CRC-8/SAE-J1850: polynomial 0x1D, initial value 0xFF, no bit reflection, final XOR 0xFF, check value 0x4B.
std::uint8_t crc8SaeJ1850(std::span<const std::uint8_t> bytes);

}  // namespace sbw
