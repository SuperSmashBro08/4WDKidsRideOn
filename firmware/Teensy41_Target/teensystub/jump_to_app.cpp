#include "jump_to_app.h"

#include "flash_map.h"

#include <cstdint>

#ifdef ARDUINO_TEENSY41
#include "imxrt.h"
#endif

extern "C" void __set_MSP(std::uint32_t topOfMainStack);

namespace ota
{
[[noreturn]] void jump_to_app(std::uint32_t app_base_address)
{
    auto stack_pointer = *reinterpret_cast<volatile std::uint32_t *>(app_base_address);
    auto reset_vector = *reinterpret_cast<volatile std::uint32_t *>(app_base_address + 4);

    __disable_irq();
    __set_MSP(stack_pointer);
#ifdef ARDUINO_TEENSY41
    SCB->VTOR = app_base_address;
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
