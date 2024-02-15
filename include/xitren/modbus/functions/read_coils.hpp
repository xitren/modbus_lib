/*!
     _ _
__ _(_) |_ _ _ ___ _ _
\ \ / |  _| '_/ -_) ' \
/_\_\_|\__|_| \___|_||_|
* @date 15.02.2024
*/
#pragma once

#include <xitren/modbus/modbus.hpp>
#include <xitren/modbus/packet.hpp>

namespace xitren::modbus::functions {

/**
 * @brief The function is used to read a set of coils from a device.
 *
 * @tparam TInputs The input data type.
 * @tparam TCoils The coil data type.
 * @tparam TInputRegisters The input register data type.
 * @tparam THoldingRegisters The holding register data type.
 * @tparam Fifo The fifo size.
 *
 * @param slave The reference to the slave object.
 *
 * @return The exception object.
 *
 * @details The function is used to read a set of coils from a device. The function checks the parameters
 * passed in the request, and if the parameters are valid, it reads the coils from the device and returns
 * the data in the response. If the parameters are not valid, the function returns an exception.
 */
template <typename TInputs, typename TCoils, typename TInputRegisters, typename THoldingRegisters, std::uint16_t Fifo>
exception
read_coils(slave_base<TInputs, TCoils, TInputRegisters, THoldingRegisters, Fifo>& slave)
{
    using slave_type  = slave_base<TInputs, TCoils, TInputRegisters, THoldingRegisters, Fifo>;
    using return_type = typename slave_type::msg_type::template fields_in<header, std::uint8_t, std::uint8_t>;
    //=========Check parameters=====================================================================
    if (slave_type::request_type_read::length != slave.input().size()) {
        return exception::bad_data;
    }
    auto pack
        = slave.input()
              .template deserialize_no_check<header, request_fields_read, func::msb_t<std::uint16_t>, crc16ansi>();
    if ((pack.fields->quantity.get() < 1) || (pack.fields->quantity.get() > slave_type::max_read_bits)) {
        return exception::illegal_data_value;
    }
    if (!slave_type::address_valid(pack.fields->starting_address.get(), pack.fields->quantity.get(),
                                   slave.coils().size())) {
        return exception::illegal_data_address;
    }
    //=========Request processing===================================================================
    if (pack.fields->quantity.get() == 0) {
        packet<header, std::uint8_t, crc16ansi> ret_pack{{slave.id(), pack.header->function_code}, {0}};
    } else {
        static std::array<std::uint8_t, slave_type::max_read_bits / 8> coils_collect;
        std::uint16_t const coils_collect_num{static_cast<std::uint16_t>((pack.fields->quantity.get() % 8)
                                                                             ? (pack.fields->quantity.get() / 8 + 1)
                                                                             : (pack.fields->quantity.get() / 8))};
        std::uint16_t const coils_collect_start{pack.fields->starting_address.get()};
        std::uint16_t const max_read_bytes = slave_type::max_read_bits / 8;
        for (std::uint16_t i = 0; (i < max_read_bytes) && (i < coils_collect_num); i++) {
            coils_collect[i] = 0;
            for (std::uint16_t j = 0;
                 (j < 8) && (static_cast<std::size_t>(i * 8 + j + coils_collect_start) < slave.coils().size()); j++) {
                if (slave.coils()[i * 8 + j + coils_collect_start]) {
                    coils_collect[i] |= 1 << j;
                }
            }
        }
        return_type data{{slave.id(), pack.header->function_code},
                         static_cast<std::uint8_t>(coils_collect_num),
                         coils_collect_num,
                         coils_collect.begin()};
        slave.output().template serialize<header, std::uint8_t, std::uint8_t, crc16ansi>(data);
    }
    return exception::no_error;
}

}    // namespace xitren::modbus::functions
