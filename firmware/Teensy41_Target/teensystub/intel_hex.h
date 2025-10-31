#pragma once

#include <cstdint>
#include <cstddef>
#include <functional>
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

class IntelHexParser
{
public:
    using RecordCallback = std::function<bool(const HexRecord &, std::string &)>;

    explicit IntelHexParser(RecordCallback cb);

    bool feed_line(std::string_view line, std::string *error_message = nullptr);
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
