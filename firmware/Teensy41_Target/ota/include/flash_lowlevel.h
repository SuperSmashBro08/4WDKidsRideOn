#pragma once

#include <cstddef>
#include <cstdint>

namespace ota
{
void flash_read(std::uint32_t address, void *buffer, std::size_t length);
void flash_erase_sector(std::uint32_t address);
void flash_program_page(std::uint32_t address, const void *data, std::size_t length);
std::uint32_t flash_compute_crc32(std::uint32_t address, std::size_t length);
}
