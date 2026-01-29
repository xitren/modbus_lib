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
 * @brief Handle custom Set Max Log Level request (0x42).
 *
 * The request payload contains the desired log level as a single byte.
 * The handler validates the level against the supported logging range and
 * echoes the accepted value back to the requester.
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
