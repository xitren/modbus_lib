#pragma once

#include <loveka/components/modbus/data.hpp>

#include <array>
#include <concepts>
#include <cstdint>
#include <cstring>

namespace loveka::components::modbus {

template <typename T>
concept crc_concept = requires
{
    typename T::value_type;
    T::calculate(std::array<std::uint8_t, 8>::iterator{nullptr},
                 std::array<std::uint8_t, 8>::iterator{reinterpret_cast<unsigned char*>(10)});
};

template<typename T>
concept crc_iterator =
    std::input_iterator<T>;

class crc16ansi {
    constexpr static uint16_t polynome = 0xA001;
    constexpr static std::array<std::uint16_t, 256>
    prepare_table(const uint16_t poly)
    {
        const std::uint16_t            table[2] = {0x0000, poly};
        std::array<std::uint16_t, 256> array{};
        int                            i = 0;
        for (std::uint16_t& item : array) {
            std::uint16_t crc_mb = i++;
            for (char bit = 0; bit < 8; bit++) {
                std::uint8_t xort = crc_mb & 0x01;
                crc_mb >>= 1;
                crc_mb ^= table[xort];
            }
            item = crc_mb;
        }
        return array;
    }

public:
    using value_type = lsb_t<std::uint16_t>;

    template <crc_iterator InputIterator>
    static constexpr value_type
    calculate(InputIterator begin, InputIterator end)
    {
        static_assert(sizeof(*begin) == 1);
        constexpr auto crc16ansi_tbl = crc16ansi::prepare_table(polynome);
        value_type     crc_mb{0xFFFF};
        for (std::input_iterator auto byte = begin; byte < end; byte++)
            crc_mb = (crc_mb.get() >> 8) ^ crc16ansi_tbl[(crc_mb.get() & 0xFF) ^ (*byte)];
        return crc_mb;
    }
};
}    // namespace xitren::components::modbus
