#include <Arduino.h>
#include <algorithm>
#include <cstring>
#include <string_view>

#include "crc32.h"
#include "flash_lowlevel.h"
#include "flash_map.h"
#include "intel_hex.h"
#include "update_flag.h"

#ifdef ARDUINO_TEENSY41
#include "imxrt.h"
#endif

using namespace ota;

namespace
{
class StagingWriter
{
public:
    void begin()
    {
        std::fill(std::begin(sector_erased_), std::end(sector_erased_), false);
        current_page_base_ = 0xFFFFFFFFu;
        page_dirty_ = false;
    }

    void write(std::uint32_t address, const std::uint8_t *data, std::size_t length)
    {
        while (length > 0)
        {
            std::uint32_t page_base = (address / FLASH_PAGE_SIZE) * FLASH_PAGE_SIZE;
            if (page_base != current_page_base_)
            {
                flush();
                current_page_base_ = page_base;
                std::memset(page_buffer_, 0xFF, sizeof(page_buffer_));
            }

            ensure_sector_erased(page_base);

            std::size_t offset = address - page_base;
            std::size_t chunk = std::min<std::size_t>(FLASH_PAGE_SIZE - offset, length);
            std::memcpy(page_buffer_ + offset, data, chunk);

            address += chunk;
            data += chunk;
            length -= chunk;
            page_dirty_ = true;
        }
    }

    void finalize()
    {
        flush();
    }

private:
    void ensure_sector_erased(std::uint32_t address)
    {
        std::uint32_t index = (address - FLASH_BASE_STAGING) / FLASH_SECTOR_SIZE;
        if (index >= MAX_STAGING_SECTORS)
            throw IntelHexError("Staging overflow");
        if (!sector_erased_[index])
        {
            flash_erase_sector(FLASH_BASE_STAGING + index * FLASH_SECTOR_SIZE);
            sector_erased_[index] = true;
        }
    }

    void flush()
    {
        if (page_dirty_ && current_page_base_ != 0xFFFFFFFFu)
        {
            flash_program_page(current_page_base_, page_buffer_, FLASH_PAGE_SIZE);
            page_dirty_ = false;
        }
        current_page_base_ = 0xFFFFFFFFu;
    }

    bool sector_erased_[MAX_STAGING_SECTORS];
    std::uint32_t current_page_base_ = 0xFFFFFFFFu;
    bool page_dirty_ = false;
    std::uint8_t page_buffer_[FLASH_PAGE_SIZE];
};

StagingWriter staging_writer;
IntelHexParser *parser = nullptr;
std::uint32_t running_crc = 0u;
std::uint32_t image_size = 0;
std::uint32_t image_base = 0xFFFFFFFFu;
bool eof_received = false;
bool update_active = false;
bool base_is_absolute = false;

void process_record(const HexRecord &record)
{
    if (record.type == 0x00 && record.length > 0)
    {
        if (!update_active)
            throw IntelHexError("Data outside of update session");
        if (image_base == 0xFFFFFFFFu)
        {
            if (record.address >= FLASH_BASE_ACTIVE &&
                record.address < FLASH_BASE_ACTIVE + FLASH_SIZE_ACTIVE)
            {
                image_base = FLASH_BASE_ACTIVE;
                base_is_absolute = true;
            }
            else
            {
                image_base = record.address;
                base_is_absolute = false;
            }
        }

        if (!base_is_absolute && record.address < image_base)
        {
            throw IntelHexError("Non-monotonic address");
        }
        if (base_is_absolute && record.address < FLASH_BASE_ACTIVE)
        {
            throw IntelHexError("Address below active region");
        }

        std::uint32_t relative = base_is_absolute ? (record.address - FLASH_BASE_ACTIVE)
                                                  : (record.address - image_base);
        if (relative + record.length > FLASH_SIZE_STAGING)
            throw IntelHexError("Image too large for staging");
        staging_writer.write(FLASH_BASE_STAGING + relative, record.data, record.length);
        running_crc = crc32_update(running_crc, record.data, record.length);
        image_size = std::max(image_size, relative + static_cast<std::uint32_t>(record.length));
    }
    else if (record.type == 0x01)
    {
        eof_received = true;
    }
}

void reset_state()
{
    update_active = false;
    eof_received = false;
    image_size = 0;
    image_base = 0xFFFFFFFFu;
    running_crc = 0u;
    base_is_absolute = false;
    staging_writer.begin();
    if (parser)
    {
        parser->reset();
    }
}

void start_update()
{
    reset_state();
    update_active = true;
    staging_writer.begin();
}

void finalize_update()
{
    staging_writer.finalize();
    if (!eof_received || image_size == 0)
    {
        Serial2.println("ERR EOF");
        reset_state();
        return;
    }

    std::uint32_t staging_crc = flash_compute_crc32(FLASH_BASE_STAGING, image_size);
    if (staging_crc != running_crc)
    {
        Serial2.println("ERR CRC");
        reset_state();
        return;
    }

    write_update_flag(image_size, staging_crc);
    Serial2.println("OK FLASH");
    Serial2.flush();
#ifdef ARDUINO_TEENSY41
    delay(10);
    NVIC_SystemReset();
#endif
}

String line_buffer;

void process_line(const String &line)
{
    if (line.equalsIgnoreCase("HEX BEGIN"))
    {
        start_update();
        Serial2.println("ACK BEGIN");
        return;
    }
    if (!update_active)
        return;
    if (line.length() > 0 && line[0] == ':')
    {
        try
        {
            parser->feed_line(std::string_view(line.c_str(), line.length()));
        }
        catch (const IntelHexError &err)
        {
            Serial2.print("ERR ");
            Serial2.println(err.what());
            reset_state();
        }
        return;
    }
    if (line.equalsIgnoreCase("HEX END"))
    {
        finalize_update();
        reset_state();
        return;
    }
}
}

void setup()
{
    Serial.begin(115200);
    Serial2.begin(115200);
    staging_writer.begin();
    static IntelHexParser parser_instance(process_record);
    parser = &parser_instance;
    Serial2.println("READY");
}

void loop()
{
    while (Serial2.available())
    {
        char c = Serial2.read();
        if (c == '\r')
            continue;
        if (c == '\n')
        {
            process_line(line_buffer);
            line_buffer.clear();
        }
        else
        {
            line_buffer += c;
        }
    }
}
