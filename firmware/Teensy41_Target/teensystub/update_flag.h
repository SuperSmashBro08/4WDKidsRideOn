#pragma once

#include <cstdint>
#include "flash_map.h"

namespace ota
{
struct UpdateFlag
{
    bool pending;
    std::uint32_t image_size;
    std::uint32_t crc32;
};

UpdateFlag read_update_flag();
void write_update_flag(std::uint32_t size, std::uint32_t crc32);
void clear_update_flag();
}
