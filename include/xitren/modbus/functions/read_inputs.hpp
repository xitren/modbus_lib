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
 * @brief Reads input registers from a Modbus slave device.
 *
 * @tparam TInputs Type of the input bit-field.
 * @tparam TCoils Type of the coil-field.
 * @tparam TInputRegisters Type of the input register-field.
 * @tparam THoldingRegisters Type of the holding register-field.
 * @tparam Fifo Maximum size of the input queue.
 * @param slave The Modbus slave device to read from.
 * @return exception Returns an exception code indicating the result of the operation.
 *
 * This function is used to read input registers from a Modbus slave device. It checks the parameters
 * of the request, and if they are valid, it processes the request by reading the requested input
 * registers and returning the data in a response packet.
 *
 * The input registers are read from the slave device's input bit-field, which is a bit-field that
 * contains the values of the input registers. The number of input registers that can be read
 * at once is determined by the quantity field of the request packet. If the quantity field is 0,
 * the function returns an exception code indicating that no data was read.
 *
 * The function checks that the starting address of the requested registers is valid, and that the
 * requested number of registers is within the range of valid addresses for the input registers. If
 * the requested number of registers is larger than the number of available input registers, the
 * function returns an exception code indicating that the request is invalid.
 *
 * If the request is valid, the function reads the requested number of input registers from the
 * input bit-field and returns them in a response packet. The response packet contains the number
 * of registers that were read, and the data for the registers.
 */
template <typename TInputs, typename TCoils, typename TInputRegisters, typename THoldingRegisters, std::uint16_t Fifo>
exception
read_inputs(slave_base<TInputs, TCoils, TInputRegisters, THoldingRegisters, Fifo>& slave)
{
    using slave_type = slave_base<TInputs, TCoils, TInputRegisters, THoldingRegisters, Fifo>;
    using return_type =
        typename slave_type::slave_type::msg_type::template fields_in<header, std::uint8_t, std::uint8_t>;
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
                                   slave.inputs().size())) {
        return exception::illegal_data_address;
    }
    //=========Request processing===================================================================
    if (pack.fields->quantity.get() == 0) {
        packet<header, std::uint8_t, crc16ansi> ret_pack{{slave.id(), pack.header->function_code}, {0}};
    } else {
        static std::array<std::uint8_t, slave_type::max_read_bits / 8> inputs_collect;
        std::uint16_t const inputs_collect_num{static_cast<std::uint16_t>((pack.fields->quantity.get() % 8)
                                                                              ? (pack.fields->quantity.get() / 8 + 1)
                                                                              : (pack.fields->quantity.get() / 8))};
        std::uint16_t const inputs_collect_start{pack.fields->starting_address.get()};
        std::uint16_t const max_read_bytes = slave_type::max_read_bits / 8;
        for (std::uint16_t i = 0; (i < max_read_bytes) && (i < inputs_collect_num); i++) {
            inputs_collect[i] = 0;
            for (std::uint16_t j = 0;
                 (j < 8) && (static_cast<std::size_t>(i * 8 + j + inputs_collect_start) < slave.inputs().size()); j++) {
                if (slave.inputs()[i * 8 + j + inputs_collect_start]) {
                    inputs_collect[i] |= 1 << j;
                }
            }
        }
        return_type data{{slave.id(), pack.header->function_code},
                         static_cast<std::uint8_t>(inputs_collect_num),
                         inputs_collect_num,
                         inputs_collect.begin()};
        slave.output().template serialize<header, std::uint8_t, std::uint8_t, crc16ansi>(data);
    }
    return exception::no_error;
}

}    // namespace xitren::modbus::functions
