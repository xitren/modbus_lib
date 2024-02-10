#pragma once

#include <xitren/modbus/modbus.hpp>
#include <xitren/modbus/packet.hpp>

namespace xitren::modbus {

template <typename TInputs, typename TCoils, typename TInputRegisters, typename THoldingRegisters, std::uint16_t Fifo>
exception
diagnostics(modbus_slave_base<TInputs, TCoils, TInputRegisters, THoldingRegisters, Fifo>& slave)
{
    using slave_type = modbus_slave_base<TInputs, TCoils, TInputRegisters, THoldingRegisters, Fifo>;
    using return_type =
        typename slave_type::msg_type::template fields_in<header, msb_t<std::uint16_t>, msb_t<std::uint16_t>>;
    //=========Check parameters=====================================================================
    auto pack
        = slave.input().template deserialize_no_check<header, msb_t<std::uint16_t>, msb_t<std::uint16_t>, crc16ansi>();
    //=========Request processing===================================================================
    switch (pack.fields->get()) {
    case static_cast<std::uint16_t>(diagnostics_sub_function::return_query_data):
        std::copy(slave.input().storage().begin(), slave.input().storage().begin() + slave.input().size(),
                  slave.output().storage().begin());
        slave.output().size(slave.input().size());
        break;
    case static_cast<std::uint16_t>(diagnostics_sub_function::restart_comm_option):
        // ToDo: Fix Listen only mode suppress
        slave.silent(false);
        slave.clear_counters();
        slave.restart_comm();
        break;
    case static_cast<std::uint16_t>(diagnostics_sub_function::return_diagnostic_register): {
        msb_t<std::uint16_t> val{slave.diagnostic_register()};
        return_type          data{{slave.id(), pack.header->function_code}, *(pack.fields), 1, &val};
        slave.output().template serialize<header, msb_t<std::uint16_t>, msb_t<std::uint16_t>, crc16ansi>(data);
    } break;
    case static_cast<std::uint16_t>(diagnostics_sub_function::force_listen_only_mode):
        slave.silent(true);
        break;
    case static_cast<std::uint16_t>(diagnostics_sub_function::clear_counters):
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
        msb_t<std::uint16_t> val{slave.get_counter(pack.fields->get())};
        return_type          data{{slave.id(), pack.header->function_code}, *(pack.fields), 1, &val};
        slave.output().template serialize<header, msb_t<std::uint16_t>, msb_t<std::uint16_t>, crc16ansi>(data);
    } break;
    default:
        return exception::illegal_function;
    }
    return exception::no_error;
}

}    // namespace xitren::modbus
