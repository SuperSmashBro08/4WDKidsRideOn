#include "update_flag.h"

#include "crc32.h"
#include "flash_lowlevel.h"
#include "hw_defs.h"

#include <cstring>

namespace ota
{
namespace
{
struct FlagLayout
{
    UpdateDescriptor descriptor;
    std::uint32_t checksum;
};

constexpr std::uint32_t RTC_MAGIC = 0x55465041u; // 'UFPA'

#if OTA_HAS_NATIVE_IMXRT
constexpr std::uint32_t SNVS_LPGPR_BASE = 0x400A4010u;
volatile std::uint32_t &rtc_word(std::size_t index)
{
    return *reinterpret_cast<volatile std::uint32_t *>(SNVS_LPGPR_BASE + index * sizeof(std::uint32_t));
}
#elif OTA_TARGET_IMXRT
volatile std::uint32_t &rtc_word(std::size_t index)
{
    return ota::hw::snvs_gpr(index);
}
#else
static std::uint32_t rtc_storage[4] = {0};
volatile std::uint32_t &rtc_word(std::size_t index)
{
    return rtc_storage[index];
}
#endif

void rtc_write_descriptor(const UpdateDescriptor &desc)
{
    rtc_word(0) = RTC_MAGIC;
    rtc_word(1) = desc.image_size;
    rtc_word(2) = desc.crc32;
}

bool rtc_descriptor_valid()
{
    return rtc_word(0) == RTC_MAGIC;
}

UpdateDescriptor rtc_read_descriptor()
{
    UpdateDescriptor desc{};
    if (!rtc_descriptor_valid())
        return desc;
    desc.magic = UPDATE_FLAG_MAGIC;
    desc.image_size = rtc_word(1);
    desc.crc32 = rtc_word(2);
    return desc;
}

UpdateDescriptor flash_read_descriptor()
{
    FlagLayout layout{};
    flash_read(FLASH_BASE_FLAG_SECTOR, &layout, sizeof(layout));
    if (layout.descriptor.magic != UPDATE_FLAG_MAGIC)
        return UpdateDescriptor{};
    std::uint32_t calc_crc = crc32_compute(&layout.descriptor, sizeof(layout.descriptor));
    if (calc_crc != layout.checksum)
        return UpdateDescriptor{};
    return layout.descriptor;
}

void flash_write_descriptor(const UpdateDescriptor &desc)
{
    FlagLayout layout{};
    layout.descriptor = desc;
    layout.checksum = crc32_compute(&layout.descriptor, sizeof(layout.descriptor));

    flash_erase_sector(FLASH_BASE_FLAG_SECTOR);
    alignas(32) std::uint8_t page_buffer[FLASH_PAGE_SIZE];
    std::memset(page_buffer, 0xFF, sizeof(page_buffer));
    std::memcpy(page_buffer, &layout, sizeof(layout));
    flash_program_page(FLASH_BASE_FLAG_SECTOR, page_buffer, FLASH_PAGE_SIZE);
}
}

UpdateFlag read_update_flag()
{
    UpdateFlag flag{false, 0, 0};
    auto desc = flash_read_descriptor();
    if (desc.magic != UPDATE_FLAG_MAGIC)
    {
        desc = rtc_read_descriptor();
    }

    if (desc.magic == UPDATE_FLAG_MAGIC)
    {
        flag.pending = true;
        flag.image_size = desc.image_size;
        flag.crc32 = desc.crc32;
        rtc_write_descriptor(desc);
    }

    return flag;
}

void write_update_flag(std::uint32_t size, std::uint32_t crc32)
{
    UpdateDescriptor desc{};
    desc.magic = UPDATE_FLAG_MAGIC;
    desc.image_size = size;
    desc.crc32 = crc32;
    flash_write_descriptor(desc);
    rtc_write_descriptor(desc);
}

void clear_update_flag()
{
    flash_erase_sector(FLASH_BASE_FLAG_SECTOR);
    rtc_word(0) = 0;
    rtc_word(1) = 0;
    rtc_word(2) = 0;
}
}
