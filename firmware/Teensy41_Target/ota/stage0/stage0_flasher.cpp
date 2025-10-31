#include "crc32.h"
#include "flash_lowlevel.h"
#include "flash_map.h"
#include "jump_to_app.h"
#include "update_flag.h"

#include <algorithm>
#include <cstdint>
#include <cstring>

namespace ota
{
namespace
{
constexpr std::size_t BUFFER_PAGES = 1;

struct alignas(32) PageBuffer
{
    std::uint8_t data[FLASH_PAGE_SIZE];
};

__attribute__((section(".dtcm_data"))) PageBuffer staging_page;

__attribute__((section(".itcm_text"))) void erase_active_region(std::uint32_t length)
{
    std::uint32_t erase_length = (length + FLASH_SECTOR_SIZE - 1u) & ~(FLASH_SECTOR_SIZE - 1u);
    for (std::uint32_t offset = 0; offset < erase_length; offset += FLASH_SECTOR_SIZE)
    {
        flash_erase_sector(FLASH_BASE_ACTIVE + offset);
    }
}

__attribute__((section(".itcm_text"))) void program_active_from_staging(std::uint32_t length)
{
    std::uint32_t padded_length = (length + FLASH_PAGE_SIZE - 1u) & ~(FLASH_PAGE_SIZE - 1u);
    for (std::uint32_t offset = 0; offset < padded_length; offset += FLASH_PAGE_SIZE)
    {
        std::memset(staging_page.data, 0xFF, FLASH_PAGE_SIZE);
        flash_read(FLASH_BASE_STAGING + offset, staging_page.data, FLASH_PAGE_SIZE);
        flash_program_page(FLASH_BASE_ACTIVE + offset, staging_page.data, FLASH_PAGE_SIZE);
    }
}

__attribute__((section(".itcm_text"))) bool verify_region(std::uint32_t base, std::uint32_t length, std::uint32_t expected_crc)
{
    std::uint32_t crc = flash_compute_crc32(base, length);
    return crc == expected_crc;
}

__attribute__((section(".itcm_text"))) void reset_into_app()
{
    jump_to_app(FLASH_BASE_ACTIVE);
}
}

__attribute__((section(".itcm_text"))) void stage0_entry()
{
    auto flag = read_update_flag();
    if (!flag.pending)
    {
        reset_into_app();
    }

    if (!verify_region(FLASH_BASE_STAGING, flag.image_size, flag.crc32))
    {
        clear_update_flag();
        reset_into_app();
    }

    erase_active_region(flag.image_size);
    program_active_from_staging(flag.image_size);

    if (!verify_region(FLASH_BASE_ACTIVE, flag.image_size, flag.crc32))
    {
        // leave flag set to retry next boot
        reset_into_app();
    }

    clear_update_flag();
    reset_into_app();
}
} // namespace

} // namespace ota

extern "C" __attribute__((section(".itcm_text"))) void stage0_main()
{
    ota::stage0_entry();
}
