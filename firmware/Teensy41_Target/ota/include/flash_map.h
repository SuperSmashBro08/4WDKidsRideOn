#pragma once

#include <cstddef>
#include <cstdint>

namespace ota
{
constexpr std::uint32_t QSPI_FLASH_BASE = 0x60000000u;
constexpr std::size_t QSPI_FLASH_SIZE = 8 * 1024 * 1024; // 8 MiB device on Teensy 4.1

constexpr std::size_t STAGE0_RESERVED_SIZE = 512 * 1024; // 512 KiB reserved for stage-0 loader

constexpr std::uint32_t FLASH_BASE_ACTIVE = QSPI_FLASH_BASE + STAGE0_RESERVED_SIZE;
constexpr std::size_t FLASH_SIZE_ACTIVE = 2 * 1024 * 1024; // 2 MiB active application slot

constexpr std::uint32_t FLASH_BASE_STAGING = FLASH_BASE_ACTIVE + FLASH_SIZE_ACTIVE;
constexpr std::size_t FLASH_SIZE_STAGING = 2 * 1024 * 1024; // 2 MiB staging slot

constexpr std::uint32_t FLASH_BASE_FLAG_SECTOR = QSPI_FLASH_BASE + STAGE0_RESERVED_SIZE - 0x1000; // last 4 KiB before app region
constexpr std::size_t FLASH_FLAG_SECTOR_SIZE = 0x1000; // 4 KiB sectors

constexpr std::size_t FLASH_SECTOR_SIZE = 0x1000; // 4 KiB
constexpr std::size_t FLASH_PAGE_SIZE = 256;        // 256 B programming granularity

constexpr std::uint32_t UPDATE_FLAG_MAGIC = 0x5550464Cu; // 'UPFL'

struct UpdateDescriptor
{
    std::uint32_t magic;
    std::uint32_t image_size;
    std::uint32_t crc32;
    std::uint32_t reserved;
};

constexpr std::size_t MAX_ACTIVE_SECTORS = FLASH_SIZE_ACTIVE / FLASH_SECTOR_SIZE;
constexpr std::size_t MAX_STAGING_SECTORS = FLASH_SIZE_STAGING / FLASH_SECTOR_SIZE;
} // namespace ota
