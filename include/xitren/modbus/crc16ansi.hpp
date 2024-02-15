/*!
_ _
__ _(_) |_ _ _ ___ _ _
\ \ / |  _| '_/ -_) ' \
/_\_\_|\__|_| \___|_||_|
* @date 15.02.2024
*/
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

    /**
     * @brief Calculates the CRC-16 ANSI checksum
     *
     * @tparam InputIterator An input iterator that points to a sequence of bytes
     * @param begin An iterator to the first element in the sequence
     * @param end An iterator to one past the last element in the sequence
     * @return The calculated CRC-16 ANSI checksum
     */
    template <crc::crc_iterator InputIterator>
    static constexpr value_type
    calculate(InputIterator begin, InputIterator end) noexcept
    {
        static_assert(sizeof(*begin) == sizeof(std::uint8_t));
        return crc::crc16::calculate(begin, end);
    }

    /**
     * @brief Calculates the CRC-16 ANSI checksum
     *
     * @tparam Size The size of the data array
     * @param data The data array
     * @return The calculated CRC-16 ANSI checksum
     */
    template <std::size_t Size>
    static consteval value_type
    calculate(std::array<std::uint8_t, Size> const& data) noexcept
    {
        return crc::crc16::calculate(data);
    }
};
}    // namespace xitren::modbus
