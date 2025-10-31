#pragma once

#include <cstdint>
#include <cstddef>
#include <functional>
#include <stdexcept>
#include <string>
#include <string_view>

namespace ota
{
struct HexRecord
{
    std::uint32_t address;
    std::uint8_t type;
    std::uint8_t length;
    const std::uint8_t *data;
};

class IntelHexError : public std::runtime_error
{
public:
    explicit IntelHexError(const std::string &msg) : std::runtime_error(msg) {}
};

class IntelHexParser
{
public:
    using RecordCallback = std::function<void(const HexRecord &)>;

    explicit IntelHexParser(RecordCallback cb);

    void feed_line(std::string_view line);
    void reset();

    std::uint32_t max_address() const noexcept { return max_address_; }
    std::uint32_t base_address() const noexcept { return base_address_; }

private:
    RecordCallback callback_;
    std::uint32_t upper_linear_base_ = 0;
    std::uint32_t base_address_ = 0;
    std::uint32_t max_address_ = 0;
};
}
