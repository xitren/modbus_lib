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
 * @brief This function is used to process the request of the write_coils.
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
write_coils(slave_base<TInputs, TCoils, TInputRegisters, THoldingRegisters, Fifo>& slave)
{
    using slave_type  = slave_base<TInputs, TCoils, TInputRegisters, THoldingRegisters, Fifo>;
    using return_type = typename slave_type::msg_type::template fields_in<header, request_fields_read, std::uint8_t>;
    //=========Check parameters=====================================================================
    auto pack
        = slave.input().template deserialize_no_check<header, request_fields_wr_single, std::uint8_t, crc16ansi>();
    std::uint8_t const coils_collect_num{static_cast<std::uint8_t>(
        (pack.fields->quantity.get() % 8) ? (pack.fields->quantity.get() / 8 + 1) : (pack.fields->quantity.get() / 8))};
    if ((pack.fields->quantity.get() < 1) || (slave_type::max_write_bits < pack.fields->quantity.get())
        || (coils_collect_num != pack.fields->count)) {
        return exception::illegal_data_value;
    }
    if (!slave_type::address_valid(pack.fields->starting_address.get(), pack.fields->quantity.get(),
                                   slave.coils().size())) {
        return exception::illegal_data_address;
    }
    //=========Request processing===================================================================
    for (std::size_t i{}; i < pack.fields->quantity.get(); i++) {
        std::uint8_t const i_bytes = i / 8;
        std::uint8_t const ii      = 1 << (i % 8);
        slave.changed_coil(pack.fields->starting_address.get() + i,
                           slave.coils()[pack.fields->starting_address.get() + i] = (pack.data[i_bytes] & ii));
    }
    return_type data{{slave.id(), pack.header->function_code},
                     {pack.fields->starting_address.get(), pack.fields->quantity.get()},
                     0,
                     nullptr};
    slave.output().template serialize<header, request_fields_read, std::uint8_t, crc16ansi>(data);
    return exception::no_error;
}

}    // namespace xitren::modbus::functions
