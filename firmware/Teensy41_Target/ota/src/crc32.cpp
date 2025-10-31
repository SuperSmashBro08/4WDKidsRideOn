#include "crc32.h"

namespace ota
{
std::uint32_t crc32_update(std::uint32_t crc, const void *data, std::size_t length) noexcept
{
    const auto *p = static_cast<const std::uint8_t *>(data);
    crc ^= 0xFFFFFFFFu;
    while (length--)
    {
        crc ^= static_cast<std::uint32_t>(*p++);
        for (unsigned i = 0; i < 8; ++i)
        {
            if (crc & 1)
                crc = (crc >> 1) ^ 0xEDB88320u;
            else
                crc >>= 1;
        }
    }
    return crc ^ 0xFFFFFFFFu;
}

std::uint32_t crc32_compute(const void *data, std::size_t length) noexcept
{
    return crc32_update(0u, data, length);
}
} // namespace ota
