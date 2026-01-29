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
 * @brief Handle Write Single Coil (0x05) request.
 *
 * The request writes one coil to either ON (0xFF00) or OFF (0x0000) and
 * echoes the request back as the response.
 *
 * Validation performed:
 * - Request length must match `request_type_read` (fixed size).
 * - Value must be either `on_coil_value` or `off_coil_value`.
 * - Address must be within coil storage bounds.
 *
 * @tparam TInputs Input discretes container type.
 * @tparam TCoils Coils container type.
 * @tparam TInputRegisters Input registers container type.
 * @tparam THoldingRegisters Holding registers container type.
 * @tparam Fifo FIFO depth.
 * @param slave Reference to the slave instance handling the request.
 * @return exception::no_error on success or a Modbus exception code otherwise.
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
