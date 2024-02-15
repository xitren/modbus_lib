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
 * @brief This function is used to process the request of the diagnostics.
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
diagnostics(slave_base<TInputs, TCoils, TInputRegisters, THoldingRegisters, Fifo>& slave)
{
    using slave_type  = slave_base<TInputs, TCoils, TInputRegisters, THoldingRegisters, Fifo>;
    using return_type = typename slave_type::msg_type::template fields_in<header, func::msb_t<std::uint16_t>,
                                                                          func::msb_t<std::uint16_t>>;
    //=========Check parameters=====================================================================
    auto pack = slave.input()
                    .template deserialize_no_check<header, func::msb_t<std::uint16_t>, func::msb_t<std::uint16_t>,
                                                   crc16ansi>();
    //=========Request processing===================================================================
    switch (pack.fields->get()) {
    case static_cast<std::uint16_t>(diagnostics_sub_function::return_query_data):
        /**
         * @brief This function is used to return the query data.
         *
         * @param slave The Modbus slave object.
         */
        std::copy(slave.input().storage().begin(), slave.input().storage().begin() + slave.input().size(),
                  slave.output().storage().begin());
        slave.output().size(slave.input().size());
        break;
    case static_cast<std::uint16_t>(diagnostics_sub_function::restart_comm_option):
        /**
         * @brief This function is used to restart the communication options.
         *
         * @param slave The Modbus slave object.
         */
        // ToDo: Fix Listen only mode suppress
        slave.silent(false);
        slave.clear_counters();
        slave.restart_comm();
        break;
    case static_cast<std::uint16_t>(diagnostics_sub_function::return_diagnostic_register): {
        /**
         * @brief This function is used to return the diagnostic register.
         *
         * @param slave The Modbus slave object.
         */
        func::msb_t<std::uint16_t> val{slave.diagnostic_register()};
        return_type                data{{slave.id(), pack.header->function_code}, *(pack.fields), 1, &val};
        slave.output().template serialize<header, func::msb_t<std::uint16_t>, func::msb_t<std::uint16_t>, crc16ansi>(
            data);
    } break;
    case static_cast<std::uint16_t>(diagnostics_sub_function::force_listen_only_mode):
        /**
         * @brief This function is used to force the listen only mode.
         *
         * @param slave The Modbus slave object.
         */
        slave.silent(true);
        break;
    case static_cast<std::uint16_t>(diagnostics_sub_function::clear_counters):
        /**
         * @brief This function is used to clear the counters.
         *
         * @param slave The Modbus slave object.
         */
        slave.clear_counters();
        break;
    case static_cast<std::uint16_t>(diagnostics_sub_function::return_bus_message_count):
    case static_cast<std::uint16_t>(diagnostics_sub_function::return_bus_comm_error_count):
    case static_cast<std::uint16_t>(diagnostics_sub_function::return_server_exception_error_count):
    case static_cast<std::uint16_t>(diagnostics_sub_function::return_server_message_count):
    case static_cast<std::uint16_t>(diagnostics_sub_function::return_server_no_response_count):
    case static_cast<std::uint16_t>(diagnostics_sub_function::return_server_nak_count):
    case static_cast<std::uint16_t>(diagnostics_sub_function::return_server_busy_count):
    case static_cast<std::uint16_t>(diagnostics_sub_function::return_bus_char_overrun_count): {
        /**
         * @brief This function is used to return the bus/server error count.
         *
         * @param slave The Modbus slave object.
         * @param pack The request packet to be processed.
         */
        func::msb_t<std::uint16_t> val{slave.get_counter(pack.fields->get())};
        return_type                data{{slave.id(), pack.header->function_code}, *(pack.fields), 1, &val};
        slave.output().template serialize<header, func::msb_t<std::uint16_t>, func::msb_t<std::uint16_t>, crc16ansi>(
            data);
    } break;
    default:
        return exception::illegal_function;
    }
    return exception::no_error;
}

}    // namespace xitren::modbus::functions
