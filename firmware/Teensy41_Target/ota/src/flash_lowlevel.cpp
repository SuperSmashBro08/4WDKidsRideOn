#include "flash_lowlevel.h"

#include "crc32.h"
#include "flash_map.h"

#include <algorithm>
#include <cstring>

#ifdef ARDUINO_TEENSY41
#include "imxrt.h"

extern "C" void arm_dcache_flush_delete(void *addr, unsigned size);
extern "C" void arm_dcache_delete(void *addr, unsigned size);
extern "C" void arm_icache_flush(void);
#endif

namespace ota
{
#ifdef ARDUINO_TEENSY41
namespace
{
constexpr std::uint32_t SEQID_ERASE_SECTOR = 5; // mapped in custom LUT
constexpr std::uint32_t SEQID_PAGE_PROGRAM = 6;
constexpr std::size_t FLEXSPI_TXFIFO_WORDS = 32;

std::uint32_t to_ip_address(std::uint32_t absolute)
{
    return absolute - QSPI_FLASH_BASE;
}

void flexspi_wait_arbitration_idle()
{
    while (FLEXSPI2->STS0 & FLEXSPI_STS0_ARBUSY)
    {
    }
}

void flexspi_wait_sequence_idle()
{
    while (!(FLEXSPI2->STS0 & FLEXSPI_STS0_SEQIDLE))
    {
    }
}

void flexspi_wait_ipcmd_done()
{
    while (!(FLEXSPI2->INTR & FLEXSPI_INTR_IPCMDDONE))
    {
    }
    FLEXSPI2->INTR = FLEXSPI_INTR_IPCMDDONE;
}

void flexspi_clear_fifos()
{
    FLEXSPI2->IPTXFCR = FLEXSPI_IPTXFCR_CLRIPTXF;
    FLEXSPI2->IPRXFCR = FLEXSPI_IPRXFCR_CLRIPRXF;
}

void flexspi_issue_command(std::uint32_t seq_id, std::uint32_t absolute)
{
    flexspi_wait_arbitration_idle();
    flexspi_clear_fifos();
    FLEXSPI2->IPCR0 = to_ip_address(absolute);
    FLEXSPI2->IPCR1 = FLEXSPI_IPCR1_SEQID(seq_id) | FLEXSPI_IPCR1_SEQNUM(0);
    FLEXSPI2->IPCMD = FLEXSPI_IPCMD_TRG;
    flexspi_wait_ipcmd_done();
    flexspi_wait_sequence_idle();
}

void flexspi_program(std::uint32_t absolute, const void *data, std::size_t length)
{
    const std::uint8_t *src = static_cast<const std::uint8_t *>(data);
    std::size_t total_words = (length + 3u) / 4u;
    std::size_t consumed = 0;

    flexspi_wait_arbitration_idle();
    flexspi_clear_fifos();
    FLEXSPI2->IPCR0 = to_ip_address(absolute);
    FLEXSPI2->IPCR1 = FLEXSPI_IPCR1_SEQID(SEQID_PAGE_PROGRAM) | FLEXSPI_IPCR1_SEQNUM(0);

    volatile std::uint32_t *fifo = &FLEXSPI2->TFDR[0];
    while (consumed < total_words)
    {
        std::size_t batch = std::min<std::size_t>(FLEXSPI_TXFIFO_WORDS, total_words - consumed);
        for (std::size_t i = 0; i < batch; ++i)
        {
            std::uint32_t word = 0xFFFFFFFFu;
            std::size_t byte_index = (consumed + i) * 4u;
            std::size_t remain = (byte_index < length) ? (length - byte_index) : 0;
            if (remain > 0)
            {
                std::memcpy(&word, src + byte_index, std::min<std::size_t>(4, remain));
            }
            fifo[i] = word;
        }
        consumed += batch;
        FLEXSPI2->IPCMD = FLEXSPI_IPCMD_TRG;
        flexspi_wait_ipcmd_done();
        if (consumed < total_words)
        {
            flexspi_clear_fifos();
            FLEXSPI2->IPCR0 = to_ip_address(absolute) + consumed * 4u;
        }
    }
    flexspi_wait_sequence_idle();
}
} // namespace

void flash_read(std::uint32_t address, void *buffer, std::size_t length)
{
    std::memcpy(buffer, reinterpret_cast<const void *>(address), length);
}

void flash_erase_sector(std::uint32_t address)
{
    address = (address / FLASH_SECTOR_SIZE) * FLASH_SECTOR_SIZE;
    __disable_irq();
    flexspi_issue_command(SEQID_ERASE_SECTOR, address);
    arm_dcache_delete(reinterpret_cast<void *>(address), FLASH_SECTOR_SIZE);
    arm_icache_flush();
    __enable_irq();
}

void flash_program_page(std::uint32_t address, const void *data, std::size_t length)
{
    if (length == 0)
        return;
    address = (address / FLASH_PAGE_SIZE) * FLASH_PAGE_SIZE;
    __disable_irq();
    arm_dcache_flush_delete(const_cast<void *>(data), length);
    flexspi_program(address, data, length);
    arm_dcache_delete(reinterpret_cast<void *>(address), length);
    arm_icache_flush();
    __enable_irq();
}

std::uint32_t flash_compute_crc32(std::uint32_t address, std::size_t length)
{
    std::uint32_t crc = 0u;
    const std::uint8_t *ptr = reinterpret_cast<const std::uint8_t *>(address);
    std::size_t remaining = length;
    while (remaining)
    {
        std::size_t chunk = std::min<std::size_t>(256, remaining);
        crc = crc32_update(crc, ptr, chunk);
        ptr += chunk;
        remaining -= chunk;
    }
    return crc;
}

#else // Mock build

std::uint8_t *g_mock_flash_base = nullptr;
std::size_t g_mock_flash_size = 0;

void flash_read(std::uint32_t address, void *buffer, std::size_t length)
{
    std::memcpy(buffer, g_mock_flash_base + (address - QSPI_FLASH_BASE), length);
}

void flash_erase_sector(std::uint32_t address)
{
    std::uint32_t offset = address - QSPI_FLASH_BASE;
    std::memset(g_mock_flash_base + offset, 0xFF, FLASH_SECTOR_SIZE);
}

void flash_program_page(std::uint32_t address, const void *data, std::size_t length)
{
    std::uint32_t offset = address - QSPI_FLASH_BASE;
    std::memcpy(g_mock_flash_base + offset, data, length);
}

std::uint32_t flash_compute_crc32(std::uint32_t address, std::size_t length)
{
    return crc32_update(0u, g_mock_flash_base + (address - QSPI_FLASH_BASE), length);
}

#endif
} // namespace ota
