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
 * @brief This function is used to process the request of the read_holding.
 *
 * @tparam TInputs The input data type.
 * @tparam TCoils The coil data type.
 * @tparam TInputRegisters The input register data type.
 * @tparam THoldingRegisters The holding register data type.
 * @tparam Fifo The FIFO size.
 * @param slave The reference to the Modbus slave object.
 * @param pack The input packet of the request.
 *
 * @return An exception object indicating the result of the operation.
 */
template <typename TInputs, typename TCoils, typename TInputRegisters, typename THoldingRegisters, std::uint16_t Fifo>
exception
read_holding(slave_base<TInputs, TCoils, TInputRegisters, THoldingRegisters, Fifo>& slave)
{
    using slave_type = slave_base<TInputs, TCoils, TInputRegisters, THoldingRegisters, Fifo>;
    using return_type =
        typename slave_type::msg_type::template fields_in<header, std::uint8_t, func::msb_t<std::uint16_t>>;
    //=========Check parameters=====================================================================
    if (slave_type::request_type_read::length != slave.input().size()) {
        return exception::bad_data;
    }
    auto pack
        = slave.input()
              .template deserialize_no_check<header, request_fields_read, func::msb_t<std::uint16_t>, crc16ansi>();
    if ((pack.fields->quantity.get() < 1) || (pack.fields->quantity.get() > slave_type::max_read_registers)) {
        return exception::illegal_data_value;
    }
    if (!slave_type::address_valid(pack.fields->starting_address.get(), pack.fields->quantity.get(),
                                   slave.holding_registers().size())) {
        return exception::illegal_data_address;
    }
    //=========Request processing===================================================================
    if (pack.fields->quantity.get() == 0) {
        packet<header, std::uint8_t, crc16ansi> ret_pack{{slave.id(), pack.header->function_code}, {0}};
    } else {
        static std::array<func::msb_t<std::uint16_t>, slave_type::max_read_registers> holding_collect;
        std::uint16_t const holding_collect_num{static_cast<std::uint16_t>(pack.fields->quantity.get())};
        std::uint16_t const holding_collect_start{pack.fields->starting_address.get()};
        for (std::uint16_t i = 0;
             (i < slave_type::max_read_registers) && ((i + holding_collect_start) < slave.holding_registers().size())
             && (i < holding_collect_num);
             i++) {
            holding_collect[i] = slave.holding_registers()[i + holding_collect_start];
        }
        return_type data{{slave.id(), pack.header->function_code},
                         static_cast<std::uint8_t>(holding_collect_num * 2),
                         holding_collect_num,
                         holding_collect.begin()};

        slave.output().template serialize<header, std::uint8_t, func::msb_t<std::uint16_t>, crc16ansi>(data);
    }
    return exception::no_error;
}

}    // namespace xitren::modbus::functions
