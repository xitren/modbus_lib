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
    static inline constexpr value_type
    calculate(InputIterator begin, InputIterator end)
    {
        static_assert(sizeof(*begin) == 1);
        return crc::crc16::calculate(begin, end);
    }
};
}    // namespace xitren::modbus
