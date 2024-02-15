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
 * @brief Writes multiple holding register values to the device.
 *
 * @tparam TInputs Type of the input buffer.
 * @tparam TCoils Type of the coils buffer.
 * @tparam TInputRegisters Type of the input registers buffer.
 * @tparam THoldingRegisters Type of the holding registers buffer.
 * @tparam Fifo Size of the FIFO queue.
 *
 * @param slave A reference to the Modbus slave object.
 * @param pack A `request_fields_wr_mask` object that contains the request parameters.
 * @return An `exception` value indicating the result of the operation.
 *
 * This function is used to write multiple holding register values to the device. The function takes a reference to the
 * Modbus slave object, which contains the input and output buffers for the request. The function deserializes the
 * request data, checks the parameters, and then processes the request. The function updates the holding register values
 * in the slave object and serializes the response data.
 */
template <typename TInputs, typename TCoils, typename TInputRegisters, typename THoldingRegisters, std::uint16_t Fifo>
exception
write_registers(slave_base<TInputs, TCoils, TInputRegisters, THoldingRegisters, Fifo>& slave)
{
    using slave_type  = slave_base<TInputs, TCoils, TInputRegisters, THoldingRegisters, Fifo>;
    using return_type = typename slave_type::msg_type::template fields_in<header, request_fields_read, std::uint16_t>;
    //=========Check parameters=====================================================================
    auto pack
        = slave.input()
              .template deserialize_no_check<header, request_fields_wr_multi, func::msb_t<std::uint16_t>, crc16ansi>();
    if (!slave_type::address_valid(pack.fields->starting_address.get(), pack.fields->quantity.get(),
                                   slave.holding_registers().size())) {
        return exception::illegal_data_address;
    }
    //=========Request processing===================================================================
    for (std::size_t i{}; i < pack.fields->quantity.get(); i++) {
        slave.changed_holding(pack.fields->starting_address.get() + i,
                              slave.holding_registers()[pack.fields->starting_address.get() + i] = pack.data[i].get());
    }
    return_type data{{slave.id(), pack.header->function_code},
                     {pack.fields->starting_address.get(), pack.fields->quantity.get()},
                     0,
                     nullptr};
    slave.output().template serialize<header, request_fields_read, std::uint16_t, crc16ansi>(data);
    return exception::no_error;
}

}    // namespace xitren::modbus::functions
