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
 * @brief Reads the exception status register of a Modbus slave device.
 *
 * @tparam TInputs Type of the input registers of the slave device.
 * @tparam TCoils Type of the coils of the slave device.
 * @tparam TInputRegisters Type of the input registers of the slave device.
 * @tparam THoldingRegisters Type of the holding registers of the slave device.
 * @tparam Fifo The size of the input queue of the slave device.
 * @param slave The Modbus slave device to read the exception status from.
 * @return exception The exception code returned by the slave device.
 *
 * This function is used to read the exception status register of a Modbus slave device. The exception status register
 * contains information about any errors that occurred during the processing of a Modbus request. The function code for
 * this request is 0x01.
 *
 * The function checks the length of the input buffer to ensure that it is the correct size for a request of this type.
 * If the length is incorrect, the function returns an exception code of `exception::bad_data`.
 *
 * The function then deserializes the request data from the input buffer, using the `deserialize_no_check` method. The
 * request data consists of a header, which contains the function code, and a number of input registers.
 *
 * The function then constructs a response data structure, which consists of a header, the exception status of the slave
 * device, and a null-terminated string containing any error messages. The function then serializes the response data to
 * the output buffer, using the `serialize` method.
 *
 * Finally, the function returns the exception code of the slave device. If the slave device returned an exception code,
 * this will be returned by the function. Otherwise, an exception code of `exception::no_error` will be returned.
 */
template <typename TInputs, typename TCoils, typename TInputRegisters, typename THoldingRegisters, std::uint16_t Fifo>
exception
read_exception_status(slave_base<TInputs, TCoils, TInputRegisters, THoldingRegisters, Fifo>& slave)
{
    using slave_type  = slave_base<TInputs, TCoils, TInputRegisters, THoldingRegisters, Fifo>;
    using return_type = typename slave_type::msg_type::template fields_in<header, std::uint8_t, std::uint8_t>;
    //=========Check parameters=====================================================================
    if (slave_type::request_type_err::length != slave.input().size()) {
        return exception::bad_data;
    }
    auto pack
        = slave.input()
              .template deserialize_no_check<header, request_fields_read, func::msb_t<std::uint16_t>, crc16ansi>();
    //=========Request processing===================================================================
    return_type data{{slave.id(), pack.header->function_code}, slave.exception_status(), 0, nullptr};
    slave.output().template serialize<header, std::uint8_t, std::uint8_t, crc16ansi>(data);
    return exception::no_error;
}

}    // namespace xitren::modbus::functions
