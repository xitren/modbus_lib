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
 * @brief This function is used to process the request of the set_max_log_level.
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
set_max_log_level(slave_base<TInputs, TCoils, TInputRegisters, THoldingRegisters, Fifo>& slave [[maybe_unused]])
{
    using slave_type  = slave_base<TInputs, TCoils, TInputRegisters, THoldingRegisters, Fifo>;
    using return_type = typename slave_type::msg_type::template fields_in<header, std::uint8_t, std::uint8_t>;
    //=========Check parameters=====================================================================
    auto pack = slave.input().template deserialize_no_check<header, std::uint8_t, std::uint8_t, crc16ansi>();
    auto lvl  = static_cast<int>(*(pack.fields));
    if ((LOG_LEVEL_TRACE > lvl) || (lvl > LOG_LEVEL_CRITICAL)) {
        return exception::bad_data;
    }
    //=========Request processing===================================================================
    //    LEVEL(MODULE(modbus), lvl);
    return_type data{{slave.id(), pack.header->function_code}, *(pack.fields), 0, nullptr};
    slave.output().template serialize<header, std::uint8_t, std::uint8_t, crc16ansi>(data);
    return exception::no_error;
}

}    // namespace xitren::modbus::functions
