#include "intel_hex.h"

#include <array>
#include <charconv>

namespace ota
{
namespace
{
std::uint8_t parse_byte(std::string_view hex)
{
    unsigned value = 0;
    for (char c : hex)
    {
        value <<= 4;
        if (c >= '0' && c <= '9')
            value |= static_cast<unsigned>(c - '0');
        else if (c >= 'A' && c <= 'F')
            value |= static_cast<unsigned>(c - 'A' + 10);
        else if (c >= 'a' && c <= 'f')
            value |= static_cast<unsigned>(c - 'a' + 10);
        else
            throw IntelHexError("Invalid hex digit");
    }
    return static_cast<std::uint8_t>(value & 0xFFu);
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

void IntelHexParser::feed_line(std::string_view line)
{
    if (line.empty())
        return;
    if (line.front() != ':')
        throw IntelHexError("Line missing start colon");

    line.remove_prefix(1);
    if (line.size() < 10)
        throw IntelHexError("Line too short");

    auto get_byte = [&](std::size_t idx) {
        if (idx * 2 + 2 > line.size())
            throw IntelHexError("Unexpected end of line");
        return parse_byte(line.substr(idx * 2, 2));
    };

    std::uint8_t length = get_byte(0);
    std::uint16_t address_hi = (static_cast<std::uint16_t>(get_byte(1)) << 8) | get_byte(2);
    std::uint8_t type = get_byte(3);

    std::array<std::uint8_t, 255> payload{};
    for (std::size_t i = 0; i < length; ++i)
    {
        payload[i] = get_byte(4 + i);
    }

    std::uint8_t expected_checksum = get_byte(4 + length);
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
        throw IntelHexError("Checksum mismatch");

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
            callback_(record);
        break;
    }
    case 0x01: // EOF
        if (callback_)
        {
            HexRecord record{absolute_address, type, 0, nullptr};
            callback_(record);
        }
        break;
    case 0x04: // extended linear address
        if (length != 2)
            throw IntelHexError("Unexpected ELA length");
        upper_linear_base_ = (static_cast<std::uint32_t>(payload[0]) << 24) |
                             (static_cast<std::uint32_t>(payload[1]) << 16);
        base_address_ = upper_linear_base_;
        break;
    case 0x05: // start linear address (ignored)
        break;
    default:
        throw IntelHexError("Unsupported record type");
    }
}
} // namespace ota
