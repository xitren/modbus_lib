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
 * @brief This function is used to process the request of the write_single_coil.
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
write_single_coil(slave_base<TInputs, TCoils, TInputRegisters, THoldingRegisters, Fifo>& slave)
{
    using slave_type = slave_base<TInputs, TCoils, TInputRegisters, THoldingRegisters, Fifo>;
    //=========Check parameters=====================================================================
    if (slave_type::request_type_read::length != slave.input().size()) {
        return exception::bad_data;
    }
    auto pack = slave.input().template deserialize_no_check<header, request_fields_read, std::uint8_t, crc16ansi>();
    if ((pack.fields->quantity.get() != slave_type::on_coil_value)
        && (pack.fields->quantity.get() != slave_type::off_coil_value)) {
        return exception::illegal_data_value;
    }
    if (pack.fields->starting_address.get() >= slave.coils().size()) {
        return exception::illegal_data_address;
    }
    //=========Request processing===================================================================
    slave.coils()[pack.fields->starting_address.get()] = pack.fields->quantity.get() == slave_type::on_coil_value;
    slave.changed_coil(pack.fields->starting_address.get(), pack.fields->quantity.get() == slave_type::on_coil_value);
    std::copy(slave.input().storage().begin(), slave.input().storage().begin() + slave.input().size(),
              slave.output().storage().begin());
    slave.output().size(slave.input().size());
    return exception::no_error;
}

}    // namespace xitren::modbus::functions
