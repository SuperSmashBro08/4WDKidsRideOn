#pragma once

#include <cstdint>

namespace ota
{
[[noreturn]] void jump_to_app(std::uint32_t app_base_address);
}
