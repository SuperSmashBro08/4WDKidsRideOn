#include "mock_flash.h"

#include "flash_lowlevel.h"
#include "flash_map.h"

#include <cstdlib>
#include <cstring>

namespace ota
{
extern std::uint8_t *g_mock_flash_base;
extern std::size_t g_mock_flash_size;
}

namespace ota::mock
{
void init_flash(std::size_t size)
{
    deinit_flash();
    g_mock_flash_size = size;
    g_mock_flash_base = static_cast<std::uint8_t *>(std::malloc(size));
    std::memset(g_mock_flash_base, 0xFF, size);
}

void deinit_flash()
{
    if (g_mock_flash_base)
    {
        std::free(g_mock_flash_base);
        g_mock_flash_base = nullptr;
        g_mock_flash_size = 0;
    }
}
}
