#pragma once

#include <xitren/modbus/modbus.hpp>
#include <xitren/modbus/packet.hpp>

namespace xitren::modbus {

template <std::uint8_t Id, std::uint16_t Inputs, std::uint16_t Coils, std::uint16_t InputRegisters,
          std::uint16_t HoldingRegisters, std::uint16_t Fifo>
exception
diagnostics(modbus_slave<Id, Inputs, Coils, InputRegisters, HoldingRegisters, Fifo>& slave)
{
    using slave_type  = modbus_slave<Id, Inputs, Coils, InputRegisters, HoldingRegisters, Fifo>;
    using return_type = typename slave_type::msg_type::template fields_in<header, func::msb_t<std::uint16_t>,
                                                                          func::msb_t<std::uint16_t>>;
    //=========Check parameters=====================================================================
    auto pack = slave.input()
                    .template deserialize<header, func::msb_t<std::uint16_t>, func::msb_t<std::uint16_t>, crc16ansi>();
    if (!pack.valid) {
        slave.increment_counter(diagnostics_sub_function::return_bus_comm_error_count);
        return exception::bad_crc;
    }
    slave.increment_counter(diagnostics_sub_function::return_bus_message_count);
    //=========Request processing===================================================================
    switch (pack.fields.get()) {
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
        func::msb_t<std::uint16_t> val{slave.diagnostic_register()};
        return_type                data{{slave.id(), pack.header.function_code}, pack.fields, 1, &val};
        slave.output().template serialize<header, func::msb_t<std::uint16_t>, func::msb_t<std::uint16_t>, crc16ansi>(
            data);
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
        func::msb_t<std::uint16_t> val{slave.get_counter(pack.fields.get())};
        return_type                data{{slave.id(), pack.header.function_code}, pack.fields, 1, &val};
        slave.output().template serialize<header, func::msb_t<std::uint16_t>, func::msb_t<std::uint16_t>, crc16ansi>(
            data);
    } break;
    default:
        return exception::illegal_function;
    }
    return exception::no_error;
}

}    // namespace xitren::modbus
