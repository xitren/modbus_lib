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
 * @brief Writes single register values to the device.
 *
 * @tparam TInputs Type of the input buffer.
 * @tparam TCoils Type of the coils buffer.
 * @tparam TInputRegisters Type of the input registers buffer.
 * @tparam THoldingRegisters Type of the holding registers buffer.
 * @tparam Fifo Size of the FIFO queue.
 *
 * @param slave A reference to the Modbus slave object.
 * @param pack An object that contains the request parameters.
 * @return An `exception` value indicating the result of the operation.
 *
 * This function is used to write single holding register values to the device. The function takes a reference to the
 * Modbus slave object, which contains the input and output buffers for the request. The function deserializes the
 * request data, checks the parameters, and then processes the request. The function updates the holding register values
 * in the slave object and serializes the response data.
 */
template <typename TInputs, typename TCoils, typename TInputRegisters, typename THoldingRegisters, std::uint16_t Fifo>
exception
write_single_register(slave_base<TInputs, TCoils, TInputRegisters, THoldingRegisters, Fifo>& slave)
{
    using slave_type = slave_base<TInputs, TCoils, TInputRegisters, THoldingRegisters, Fifo>;
    //=========Check parameters=====================================================================
    if (slave_type::request_type_read::length != slave.input().size()) {
        return exception::bad_data;
    }
    auto pack = slave.input().template deserialize_no_check<header, request_fields_read, std::uint8_t, crc16ansi>();
    if (pack.fields->starting_address.get() >= slave.holding_registers().size()) {
        return exception::illegal_data_address;
    }
    //=========Request processing===================================================================
    slave.holding_registers()[pack.fields->starting_address.get()] = pack.fields->quantity.get();
    slave.changed_holding(pack.fields->starting_address.get(), pack.fields->quantity.get());
    std::copy(slave.input().storage().begin(), slave.input().storage().begin() + slave.input().size(),
              slave.output().storage().begin());
    slave.output().size(slave.input().size());
    return exception::no_error;
}

}    // namespace xitren::modbus::functions
