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
 * @brief This function is used to process the request of the get current log level function.
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
get_current_log_level(slave_base<TInputs, TCoils, TInputRegisters, THoldingRegisters, Fifo>& slave)
{
    using slave_type  = slave_base<TInputs, TCoils, TInputRegisters, THoldingRegisters, Fifo>;
    using return_type = typename slave_type::msg_type::template fields_in<header, std::uint8_t, std::uint8_t>;
    //=========Check parameters=====================================================================
    auto pack = slave.input().template deserialize_no_check<header, std::uint8_t, std::uint8_t, crc16ansi>();
    //=========Request processing===================================================================
    auto        log_mode = GET_LEVEL();
    return_type data{slave.id(), pack.header->function_code, static_cast<std::uint8_t>(log_mode), 0, nullptr};
    slave.output().template serialize<header, std::uint8_t, std::uint8_t, crc16ansi>(data);
    return exception::no_error;
}

}    // namespace xitren::modbus::functions
