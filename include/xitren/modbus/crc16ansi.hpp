#pragma once

#include <xitren/crc16.hpp>
#include <xitren/func/data.hpp>

#include <array>
#include <concepts>
#include <cstdint>
#include <cstring>

namespace xitren::modbus {

class crc16ansi {
public:
    using value_type = func::lsb_t<std::uint16_t>;

    template <crc::crc_iterator InputIterator>
    static constexpr value_type
    calculate(InputIterator begin, InputIterator end) noexcept
    {
        static_assert(sizeof(*begin) == sizeof(std::uint8_t));
        return crc::crc16::calculate(begin, end);
    }

    template <std::size_t Size>
    static consteval value_type
    calculate(std::array<std::uint8_t, Size> const& data) noexcept
    {
        return crc::crc16::calculate(data);
    }
};
}    // namespace xitren::modbus
