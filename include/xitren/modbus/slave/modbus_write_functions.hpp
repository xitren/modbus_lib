#pragma once

#include <xitren/modbus/modbus.hpp>
#include <xitren/modbus/packet.hpp>

namespace xitren::modbus {

template <std::uint8_t Id, std::uint16_t Inputs, std::uint16_t Coils, std::uint16_t InputRegisters,
          std::uint16_t HoldingRegisters, std::uint16_t Fifo>
exception
write_registers(modbus_slave<Id, Inputs, Coils, InputRegisters, HoldingRegisters, Fifo>& slave)
{
    using slave_type  = modbus_slave<Id, Inputs, Coils, InputRegisters, HoldingRegisters, Fifo>;
    using return_type = typename slave_type::msg_type::template fields_in<header, request_fields_read, std::uint16_t>;
    //=========Check parameters=====================================================================
    auto pack
        = slave.input().template deserialize<header, request_fields_wr_single, func::msb_t<std::uint16_t>, crc16ansi>();
    if (!pack.valid) {
        slave.increment_counter(diagnostics_sub_function::return_bus_comm_error_count);
        return exception::bad_crc;
    }
    slave.increment_counter(diagnostics_sub_function::return_bus_message_count);
    if (!slave_type::address_valid(pack.fields.starting_address.get(), pack.fields.quantity.get(),
                                   slave.holding_registers().size())) {
        return exception::illegal_data_address;
    }
    //=========Request processing===================================================================
    for (std::size_t i{}; i < pack.fields.quantity.get(); i++) {
        slave.changed_holding(pack.fields.starting_address.get() + i,
                              slave.holding_registers()[pack.fields.starting_address.get() + i] = pack.data[i].get());
    }
    return_type data{{slave.id(), pack.header.function_code},
                     {pack.fields.starting_address.get(), pack.fields.quantity.get()},
                     0,
                     nullptr};
    slave.output().template serialize<header, request_fields_read, std::uint16_t, crc16ansi>(data);
    return exception::no_error;
}

template <std::uint8_t Id, std::uint16_t Inputs, std::uint16_t Coils, std::uint16_t InputRegisters,
          std::uint16_t HoldingRegisters, std::uint16_t Fifo>
exception
write_coils(modbus_slave<Id, Inputs, Coils, InputRegisters, HoldingRegisters, Fifo>& slave)
{
    using slave_type  = modbus_slave<Id, Inputs, Coils, InputRegisters, HoldingRegisters, Fifo>;
    using return_type = typename slave_type::msg_type::template fields_in<header, request_fields_read, std::uint8_t>;
    //=========Check parameters=====================================================================
    auto pack = slave.input().template deserialize<header, request_fields_wr_single, std::uint8_t, crc16ansi>();
    if (!pack.valid) {
        slave.increment_counter(diagnostics_sub_function::return_bus_comm_error_count);
        return exception::bad_crc;
    }
    slave.increment_counter(diagnostics_sub_function::return_bus_message_count);
    std::uint8_t const coils_collect_num{static_cast<std::uint8_t>(
        (pack.fields.quantity.get() % 8) ? (pack.fields.quantity.get() / 8 + 1) : (pack.fields.quantity.get() / 8))};
    if ((pack.fields.quantity.get() < 1) || (slave_type::max_write_bits < pack.fields.quantity.get())
        || (coils_collect_num != pack.fields.count)) {
        return exception::illegal_data_value;
    }
    if (!slave_type::address_valid(pack.fields.starting_address.get(), pack.fields.quantity.get(),
                                   slave.coils().size())) {
        return exception::illegal_data_address;
    }
    //=========Request processing===================================================================
    for (std::size_t i{}; i < pack.fields.quantity.get(); i++) {
        std::uint8_t const i_bytes = i / 8;
        std::uint8_t const ii      = 1 << (i % 8);
        slave.changed_coil(pack.fields.starting_address.get() + i,
                           slave.coils()[pack.fields.starting_address.get() + i] = (pack.data[i_bytes] & ii));
    }
    return_type data{{slave.id(), pack.header.function_code},
                     {pack.fields.starting_address.get(), pack.fields.quantity.get()},
                     0,
                     nullptr};
    slave.output().template serialize<header, request_fields_read, std::uint8_t, crc16ansi>(data);
    return exception::no_error;
}

template <std::uint8_t Id, std::uint16_t Inputs, std::uint16_t Coils, std::uint16_t InputRegisters,
          std::uint16_t HoldingRegisters, std::uint16_t Fifo>
exception
write_single_coil(modbus_slave<Id, Inputs, Coils, InputRegisters, HoldingRegisters, Fifo>& slave)
{
    using slave_type = modbus_slave<Id, Inputs, Coils, InputRegisters, HoldingRegisters, Fifo>;
    //=========Check parameters=====================================================================
    if (slave_type::request_type_read::length != slave.input().size()) {
        return exception::bad_data;
    }
    auto pack = slave_type::request_type_read::deserialize(slave.input().storage().begin());
    if (!pack.valid()) {
        slave.increment_counter(diagnostics_sub_function::return_bus_comm_error_count);
        return exception::bad_crc;
    }
    if ((pack.fields().quantity != slave_type::on_coil_value)
        && (pack.fields().quantity != slave_type::off_coil_value)) {
        return exception::illegal_data_value;
    }
    if (pack.fields().starting_address >= slave.coils().size()) {
        return exception::illegal_data_address;
    }
    //=========Request processing===================================================================
    slave.coils()[pack.fields().starting_address.get()] = pack.fields().quantity == slave_type::on_coil_value;
    slave.changed_coil(pack.fields().starting_address.get(), pack.fields().quantity.get() == slave_type::on_coil_value);
    std::copy(slave.input().storage().begin(), slave.input().storage().begin() + slave.input().size(),
              slave.output().storage().begin());
    slave.output().size(slave.input().size());
    return exception::no_error;
}

template <std::uint8_t Id, std::uint16_t Inputs, std::uint16_t Coils, std::uint16_t InputRegisters,
          std::uint16_t HoldingRegisters, std::uint16_t Fifo>
exception
write_single_register(modbus_slave<Id, Inputs, Coils, InputRegisters, HoldingRegisters, Fifo>& slave)
{
    using slave_type = modbus_slave<Id, Inputs, Coils, InputRegisters, HoldingRegisters, Fifo>;
    //=========Check parameters=====================================================================
    if (slave_type::request_type_read::length != slave.input().size()) {
        return exception::bad_data;
    }
    auto pack = slave_type::request_type_read::deserialize(slave.input().storage().begin());
    if (!pack.valid()) {
        slave.increment_counter(diagnostics_sub_function::return_bus_comm_error_count);
        return exception::bad_crc;
    }
    if (pack.fields().starting_address >= slave.holding_registers().size()) {
        return exception::illegal_data_address;
    }
    //=========Request processing===================================================================
    slave.holding_registers()[pack.fields().starting_address.get()] = pack.fields().quantity.get();
    slave.changed_holding(pack.fields().starting_address.get(), pack.fields().quantity.get());
    std::copy(slave.input().storage().begin(), slave.input().storage().begin() + slave.input().size(),
              slave.output().storage().begin());
    slave.output().size(slave.input().size());
    return exception::no_error;
}

template <std::uint8_t Id, std::uint16_t Inputs, std::uint16_t Coils, std::uint16_t InputRegisters,
          std::uint16_t HoldingRegisters, std::uint16_t Fifo>
exception
write_register_mask(modbus_slave<Id, Inputs, Coils, InputRegisters, HoldingRegisters, Fifo>& slave)
{
    using slave_type = modbus_slave<Id, Inputs, Coils, InputRegisters, HoldingRegisters, Fifo>;
    //=========Check parameters=====================================================================
    if (slave_type::request_type_read::length != slave.input().size()) {
        return exception::bad_data;
    }
    auto pack = slave_type::request_type_wr_mask::deserialize(slave.input().storage().begin());
    if (!pack.valid()) {
        slave.increment_counter(diagnostics_sub_function::return_bus_comm_error_count);
        return exception::bad_crc;
    }
    if (pack.fields().starting_address >= slave.holding_registers().size()) {
        return exception::illegal_data_address;
    }
    //=========Request processing===================================================================
    std::uint16_t const current = slave.holding_registers()[pack.fields().starting_address.get()];
    std::uint16_t const value
        = (current & pack.fields().and_mask.get()) | (pack.fields().or_mask.get() & (~pack.fields().and_mask.get()));
    slave.holding_registers()[pack.fields().starting_address.get()] = value;
    slave.changed_holding(pack.fields().starting_address.get(), value);
    std::copy(slave.input().storage().begin(), slave.input().storage().begin() + slave.input().size(),
              slave.output().storage().begin());
    slave.output().size(slave.input().size());
    return exception::no_error;
}

}    // namespace xitren::modbus
