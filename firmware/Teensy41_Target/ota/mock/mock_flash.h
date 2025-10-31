#pragma once

#include <cstddef>

namespace ota::mock
{
void init_flash(std::size_t size);
void deinit_flash();
}
