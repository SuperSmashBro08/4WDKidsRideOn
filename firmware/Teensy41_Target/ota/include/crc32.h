#pragma once

#include <cstddef>
#include <cstdint>

namespace ota
{
std::uint32_t crc32_update(std::uint32_t crc, const void *data, std::size_t length) noexcept;
std::uint32_t crc32_compute(const void *data, std::size_t length) noexcept;
}
