#include "jump_to_app.h"

#include "flash_map.h"
#include "hw_defs.h"

#include <cstdint>

extern "C" void __set_MSP(std::uint32_t topOfMainStack);

namespace ota
{
[[noreturn]] void jump_to_app(std::uint32_t app_base_address)
{
    auto stack_pointer = *reinterpret_cast<volatile std::uint32_t *>(app_base_address);
    auto reset_vector = *reinterpret_cast<volatile std::uint32_t *>(app_base_address + 4);

    __disable_irq();
    __set_MSP(stack_pointer);
#if OTA_HAS_NATIVE_IMXRT
    SCB->VTOR = app_base_address;
#elif OTA_TARGET_IMXRT
    ota::hw::set_vtor(app_base_address);
#endif
    __DSB();
    __ISB();
    auto entry = reinterpret_cast<void (*)()>(reset_vector);
    __enable_irq();
    entry();
    while (true)
    {
    }
}
} // namespace ota
