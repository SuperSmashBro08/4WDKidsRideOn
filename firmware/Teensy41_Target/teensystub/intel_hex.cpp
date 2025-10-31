#include "intel_hex.h"

#include <array>

namespace ota
{
namespace
{
bool parse_byte(std::string_view hex, std::uint8_t &value, std::string &error_message)
{
    if (hex.size() != 2)
    {
        error_message = "Invalid byte width";
        return false;
    }

    auto decode = [](char c, std::uint8_t &out) -> bool {
        if (c >= '0' && c <= '9')
        {
            out = static_cast<std::uint8_t>(c - '0');
            return true;
        }
        if (c >= 'A' && c <= 'F')
        {
            out = static_cast<std::uint8_t>(c - 'A' + 10);
            return true;
        }
        if (c >= 'a' && c <= 'f')
        {
            out = static_cast<std::uint8_t>(c - 'a' + 10);
            return true;
        }
        return false;
    };

    std::uint8_t hi = 0;
    std::uint8_t lo = 0;
    if (!decode(hex[0], hi) || !decode(hex[1], lo))
    {
        error_message = "Invalid hex digit";
        return false;
    }

    value = static_cast<std::uint8_t>((hi << 4) | lo);
    return true;
}

std::uint8_t checksum(const std::uint8_t *data, std::size_t len)
{
    std::uint32_t sum = 0;
    for (std::size_t i = 0; i < len; ++i)
    {
        sum += data[i];
    }
    return static_cast<std::uint8_t>((~sum + 1u) & 0xFFu);
}
} // namespace

IntelHexParser::IntelHexParser(RecordCallback cb) : callback_(std::move(cb)) {}

void IntelHexParser::reset()
{
    upper_linear_base_ = 0;
    base_address_ = 0;
    max_address_ = 0;
}

bool IntelHexParser::feed_line(std::string_view line, std::string *error_message)
{
    std::string dummy;
    std::string &err = error_message ? *error_message : dummy;
    err.clear();

    if (line.empty())
        return true;
    if (line.front() != ':')
    {
        err = "Line missing start colon";
        return false;
    }

    line.remove_prefix(1);
    if (line.size() < 10 || (line.size() & 1u))
    {
        err = "Malformed record length";
        return false;
    }

    auto get_byte = [&](std::size_t idx, std::uint8_t &value) -> bool {
        std::size_t pos = idx * 2;
        if (pos + 2 > line.size())
        {
            err = "Unexpected end of line";
            return false;
        }
        return parse_byte(line.substr(pos, 2), value, err);
    };

    std::uint8_t length = 0;
    if (!get_byte(0, length))
        return false;

    std::uint8_t addr_hi = 0;
    std::uint8_t addr_lo = 0;
    if (!get_byte(1, addr_hi) || !get_byte(2, addr_lo))
        return false;
    std::uint16_t address_hi = (static_cast<std::uint16_t>(addr_hi) << 8) | addr_lo;

    std::uint8_t type = 0;
    if (!get_byte(3, type))
        return false;

    std::array<std::uint8_t, 255> payload{};
    for (std::size_t i = 0; i < length; ++i)
    {
        std::uint8_t value = 0;
        if (!get_byte(4 + i, value))
            return false;
        payload[i] = value;
    }

    std::uint8_t expected_checksum = 0;
    if (!get_byte(4 + length, expected_checksum))
        return false;
    std::array<std::uint8_t, 5 + 255> raw{};
    raw[0] = length;
    raw[1] = static_cast<std::uint8_t>(address_hi >> 8);
    raw[2] = static_cast<std::uint8_t>(address_hi & 0xFFu);
    raw[3] = type;
    for (std::size_t i = 0; i < length; ++i)
    {
        raw[4 + i] = payload[i];
    }

    if (checksum(raw.data(), 4 + length) != expected_checksum)
    {
        err = "Checksum mismatch";
        return false;
    }

    std::uint32_t absolute_address = upper_linear_base_ + address_hi;

    switch (type)
    {
    case 0x00: // data record
    {
        HexRecord record{absolute_address, type, length, payload.data()};
        std::uint32_t end_address = absolute_address + length;
        if (end_address > max_address_)
            max_address_ = end_address;
        if (callback_)
        {
            if (!callback_(record, err))
                return false;
        }
        break;
    }
    case 0x01: // EOF
        if (callback_)
        {
            HexRecord record{absolute_address, type, 0, nullptr};
            if (!callback_(record, err))
                return false;
        }
        break;
    case 0x04: // extended linear address
        if (length != 2)
        {
            err = "Unexpected ELA length";
            return false;
        }
        upper_linear_base_ = (static_cast<std::uint32_t>(payload[0]) << 24) |
                             (static_cast<std::uint32_t>(payload[1]) << 16);
        base_address_ = upper_linear_base_;
        break;
    case 0x05: // start linear address (ignored)
        break;
    default:
        err = "Unsupported record type";
        return false;
    }
    return true;
}
} // namespace ota
