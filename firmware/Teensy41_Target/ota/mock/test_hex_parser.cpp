#include "intel_hex.h"

#include "crc32.h"
#include "flash_map.h"
#include "mock_flash.h"

#include <cassert>
#include <iostream>
#include <vector>

using namespace ota;

int main()
{
    mock::init_flash(FLASH_SIZE_STAGING);

    std::vector<std::uint8_t> captured;
    std::uint32_t last_address = 0;

    IntelHexParser parser([&](const HexRecord &record) {
        if (record.type == 0x00)
        {
            captured.assign(record.data, record.data + record.length);
            last_address = record.address;
        }
    });

    parser.feed_line(":10000000000102030405060708090A0B0C0D0E0F78");
    parser.feed_line(":00000001FF");

    assert(captured.size() == 16);
    assert(captured.front() == 0x00);
    assert(captured.back() == 0x0F);
    assert(last_address == 0x00000000);
    assert(parser.max_address() == 0x10);

    IntelHexParser parser_ela([&](const HexRecord &record) {
        if (record.type == 0x00)
        {
            last_address = record.address;
        }
    });
    parser_ela.feed_line(":020000040003F7");
    parser_ela.feed_line(":10001000101112131415161718191A1B1C1D1E1F68");
    assert(last_address == 0x00030010);

    const char *crc_sample = "123456789";
    auto crc = crc32_compute(crc_sample, 9);
    assert(crc == 0xCBF43926u);

    mock::deinit_flash();
    std::cout << "All tests passed\n";
    return 0;
}
