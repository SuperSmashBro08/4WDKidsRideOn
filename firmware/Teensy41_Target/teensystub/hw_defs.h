#pragma once

#include <cstdint>

#if defined(ARDUINO_TEENSY41)
#  if __has_include(<Arduino.h>)
#    include <Arduino.h>
#  endif
#endif

#if defined(ARDUINO_TEENSY41) && __has_include(<imxrt.h>) && __has_include(<cmsis_version.h>)
#define OTA_HAS_NATIVE_IMXRT 1
#  include <imxrt.h>
#else
#define OTA_HAS_NATIVE_IMXRT 0
namespace ota::hw
{
constexpr std::uintptr_t FLEXSPI2_BASE = 0x402A4000u;
inline volatile std::uint32_t &reg(std::uintptr_t offset)
{
    return *reinterpret_cast<volatile std::uint32_t *>(FLEXSPI2_BASE + offset);
}
inline volatile std::uint32_t &sts0()
{
    return reg(0x40u);
}
inline volatile std::uint32_t &intr()
{
    return reg(0x14u);
}
inline volatile std::uint32_t &ipcr0()
{
    return reg(0xA0u);
}
inline volatile std::uint32_t &ipcr1()
{
    return reg(0xA4u);
}
inline volatile std::uint32_t &ipcmd()
{
    return reg(0xB0u);
}
inline volatile std::uint32_t &iptxfcr()
{
    return reg(0x180u);
}
inline volatile std::uint32_t &iprxfcr()
{
    return reg(0x188u);
}
inline volatile std::uint32_t *tfdr()
{
    return reinterpret_cast<volatile std::uint32_t *>(FLEXSPI2_BASE + 0x200u);
}
inline volatile std::uint32_t &snvs_gpr(std::size_t index)
{
    return *reinterpret_cast<volatile std::uint32_t *>(0x400A4010u + index * sizeof(std::uint32_t));
}
inline void set_vtor(std::uint32_t address)
{
    *reinterpret_cast<volatile std::uint32_t *>(0xE000ED08u) = address;
}
} // namespace ota::hw

constexpr std::uint32_t FLEXSPI_STS0_ARBIDLE = (1u << 9);
constexpr std::uint32_t FLEXSPI_STS0_SEQIDLE = (1u << 10);
constexpr std::uint32_t FLEXSPI_INTR_IPCMDDONE = (1u << 1);
constexpr std::uint32_t FLEXSPI_IPTXFCR_CLRIPTXF = (1u << 0);
constexpr std::uint32_t FLEXSPI_IPRXFCR_CLRIPRXF = (1u << 0);
inline std::uint32_t FLEXSPI_IPCR1_ISEQID(std::uint32_t seq)
{
    return seq << 16;
}
inline std::uint32_t FLEXSPI_IPCR1_ISEQNUM(std::uint32_t num)
{
    return num << 24;
}
constexpr std::uint32_t FLEXSPI_IPCMD_TRG = 1u;

#ifndef __disable_irq
inline void __disable_irq()
{
    __asm__ volatile("cpsid i" ::: "memory");
}
#endif
#ifndef __enable_irq
inline void __enable_irq()
{
    __asm__ volatile("cpsie i" ::: "memory");
}
#endif
#ifndef __DSB
inline void __DSB()
{
    __asm__ volatile("dsb 0xF" ::: "memory");
}
#endif
#ifndef __ISB
inline void __ISB()
{
    __asm__ volatile("isb 0xF" ::: "memory");
}
#endif
#ifndef NVIC_SystemReset
inline void NVIC_SystemReset()
{
    __disable_irq();
    constexpr std::uint32_t AIRCR_ADDRESS = 0xE000ED0Cu;
    constexpr std::uint32_t VECTKEY = 0x5FAu << 16;
    *reinterpret_cast<volatile std::uint32_t *>(AIRCR_ADDRESS) = VECTKEY | (1u << 2);
    __DSB();
    while (true)
    {
        __asm__ volatile("nop");
    }
}
#endif
#endif

#if defined(ARDUINO_TEENSY41) || defined(__IMXRT1062__)
#define OTA_TARGET_IMXRT 1
#else
#define OTA_TARGET_IMXRT 0
#endif
