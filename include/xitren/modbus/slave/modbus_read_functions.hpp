#pragma once

#include "loveka/components/modbus/modbus.hpp"
#include "loveka/components/modbus/packet.hpp"

namespace loveka::components::modbus {

template <std::uint8_t Id, std::uint16_t Inputs, std::uint16_t Coils, std::uint16_t InputRegisters,
          std::uint16_t HoldingRegisters, std::uint16_t Fifo>
exception
read_exception_status(
    modbus_slave<Id, Inputs, Coils, InputRegisters, HoldingRegisters, Fifo>& slave)
{
    using slave_type = modbus_slave<Id, Inputs, Coils, InputRegisters, HoldingRegisters, Fifo>;
    using return_type =
        typename slave_type::msg_type::template fields_in<header, std::uint8_t, std::uint8_t>;
    //=========Check parameters=====================================================================
    if (slave_type::request_type_err::length != slave.input().size()) {
        return exception::bad_data;
    }
    auto pack = slave_type::request_type_read::deserialize(slave.input().storage().begin());
    if (!pack.valid()) {
        slave.increment_counter(diagnostics_sub_function::return_bus_comm_error_count);
        return exception::bad_crc;
    }
    //=========Request processing===================================================================
    return_type data{
        {slave.id(), pack.header().function_code}, slave.exception_status(), 0, nullptr};
    slave.output().template serialize<header, std::uint8_t, std::uint8_t, crc16ansi>(data);
    return exception::no_error;
}

template <std::uint8_t Id, std::uint16_t Inputs, std::uint16_t Coils, std::uint16_t InputRegisters,
          std::uint16_t HoldingRegisters, std::uint16_t Fifo>
exception
read_coils(modbus_slave<Id, Inputs, Coils, InputRegisters, HoldingRegisters, Fifo>& slave)
{
    using slave_type = modbus_slave<Id, Inputs, Coils, InputRegisters, HoldingRegisters, Fifo>;
    using return_type =
        typename slave_type::msg_type::template fields_in<header, std::uint8_t, std::uint8_t>;
    //=========Check parameters=====================================================================
    if (slave_type::request_type_read::length != slave.input().size()) {
        return exception::bad_data;
    }
    auto pack = slave_type::request_type_read::deserialize(slave.input().storage().begin());
    if (!pack.valid()) {
        slave.increment_counter(diagnostics_sub_function::return_bus_comm_error_count);
        return exception::bad_crc;
    }
    if ((pack.fields().quantity < 1) || (pack.fields().quantity > slave_type::max_read_bits)) {
        return exception::illegal_data_value;
    }
    if (!slave_type::address_valid(pack.fields().starting_address.get(),
                                   pack.fields().quantity.get(), slave.coils().size())) {
        return exception::illegal_data_address;
    }
    //=========Request processing===================================================================
    if (pack.fields().quantity == 0) {
        packet<header, std::uint8_t, crc16ansi> ret_pack{{slave.id(), pack.header().function_code},
                                                         {0}};
    } else {
        static std::array<std::uint8_t, slave_type::max_read_bits / 8> coils_collect;
        const std::uint16_t coils_collect_num{static_cast<std::uint16_t>(
            (pack.fields().quantity.get() % 8) ? (pack.fields().quantity.get() / 8 + 1)
                                               : (pack.fields().quantity.get() / 8))};
        const std::uint16_t coils_collect_start{pack.fields().starting_address.get()};
        const std::uint16_t max_read_bytes = slave_type::max_read_bits / 8;
        for (std::uint16_t i = 0; (i < max_read_bytes) && (i < coils_collect_num); i++) {
            coils_collect[i] = 0;
            for (std::uint16_t j = 0; (j < 8)
                                      && (static_cast<std::size_t>(i * 8 + j + coils_collect_start)
                                          < slave.coils().size());
                 j++) {
                if (slave.coils()[i * 8 + j + coils_collect_start]
                    && (pack.fields().quantity > (i * 8 + j + coils_collect_start))) {
                    coils_collect[i] |= 1 << j;
                }
            }
        }
        return_type data{{slave.id(), pack.header().function_code},
                                static_cast<std::uint8_t>(coils_collect_num),
                                coils_collect_num,
                                coils_collect.begin()};
        slave.output().template serialize<header, std::uint8_t, std::uint8_t, crc16ansi>(data);
    }
    return exception::no_error;
}

template <std::uint8_t Id, std::uint16_t Inputs, std::uint16_t Coils, std::uint16_t InputRegisters,
          std::uint16_t HoldingRegisters, std::uint16_t Fifo>
exception
read_inputs(modbus_slave<Id, Inputs, Coils, InputRegisters, HoldingRegisters, Fifo>& slave)
{
    using slave_type = modbus_slave<Id, Inputs, Coils, InputRegisters, HoldingRegisters, Fifo>;
    using return_type =
        typename slave_type::slave_type::msg_type::template fields_in<header, std::uint8_t,
                                                                      std::uint8_t>;
    //=========Check parameters=====================================================================
    if (slave_type::request_type_read::length != slave.input().size()) {
        return exception::bad_data;
    }
    auto pack = slave_type::request_type_read::deserialize(slave.input().storage().begin());
    if (!pack.valid()) {
        slave.increment_counter(diagnostics_sub_function::return_bus_comm_error_count);
        return exception::bad_crc;
    }
    if ((pack.fields().quantity < 1) || (pack.fields().quantity > slave_type::max_read_bits)) {
        return exception::illegal_data_value;
    }
    if (!slave_type::address_valid(pack.fields().starting_address.get(),
                                   pack.fields().quantity.get(), slave.inputs().size())) {
        return exception::illegal_data_address;
    }
    //=========Request processing===================================================================
    if (pack.fields().quantity == 0) {
        packet<header, std::uint8_t, crc16ansi> ret_pack{{slave.id(), pack.header().function_code},
                                                         {0}};
    } else {
        static std::array<std::uint8_t, slave_type::max_read_bits / 8> inputs_collect;
        const std::uint16_t inputs_collect_num{static_cast<std::uint16_t>(
            (pack.fields().quantity.get() % 8) ? (pack.fields().quantity.get() / 8 + 1)
                                               : (pack.fields().quantity.get() / 8))};
        const std::uint16_t inputs_collect_start{pack.fields().starting_address.get()};
        const std::uint16_t max_read_bytes = slave_type::max_read_bits / 8;
        for (std::uint16_t i = 0; (i < max_read_bytes) && (i < inputs_collect_num); i++) {
            inputs_collect[i] = 0;
            for (std::uint16_t j = 0; (j < 8)
                                      && (static_cast<std::size_t>(i * 8 + j + inputs_collect_start)
                                          < slave.inputs().size());
                 j++) {
                if ((slave.inputs()[i * 8 + j + inputs_collect_start])
                    && (pack.fields().quantity > (i * 8 + j + inputs_collect_start))) {
                    inputs_collect[i] |= 1 << j;
                }
            }
        }
        return_type data{{slave.id(), pack.header().function_code},
                                static_cast<std::uint8_t>(inputs_collect_num),
                                inputs_collect_num,
                                inputs_collect.begin()};
        slave.output().template serialize<header, std::uint8_t, std::uint8_t, crc16ansi>(data);
    }
    return exception::no_error;
}

template <std::uint8_t Id, std::uint16_t Inputs, std::uint16_t Coils, std::uint16_t InputRegisters,
          std::uint16_t HoldingRegisters, std::uint16_t Fifo>
exception
read_holding(modbus_slave<Id, Inputs, Coils, InputRegisters, HoldingRegisters, Fifo>& slave)
{
    using slave_type  = modbus_slave<Id, Inputs, Coils, InputRegisters, HoldingRegisters, Fifo>;
    using return_type = typename slave_type::msg_type::template fields_in<header, std::uint8_t,
                                                                          msb_t<std::uint16_t>>;
    //=========Check parameters=====================================================================
    if (slave_type::request_type_read::length != slave.input().size()) {
        return exception::bad_data;
    }
    auto pack = slave_type::request_type_read::deserialize(slave.input().storage().begin());
    if (!pack.valid()) {
        slave.increment_counter(diagnostics_sub_function::return_bus_comm_error_count);
        return exception::bad_crc;
    }
    if ((pack.fields().quantity < 1) || (pack.fields().quantity > slave_type::max_read_registers)) {
        return exception::illegal_data_value;
    }
    if (!slave_type::address_valid(pack.fields().starting_address.get(),
                                   pack.fields().quantity.get(),
                                   slave.holding_registers().size())) {
        return exception::illegal_data_address;
    }
    //=========Request processing===================================================================
    if (pack.fields().quantity == 0) {
        packet<header, std::uint8_t, crc16ansi> ret_pack{{slave.id(), pack.header().function_code},
                                                         {0}};
    } else {
        static std::array<msb_t<std::uint16_t>, slave_type::max_read_registers> holding_collect;
        const std::uint16_t                                                     holding_collect_num{
            static_cast<std::uint16_t>(pack.fields().quantity.get())};
        const std::uint16_t holding_collect_start{pack.fields().starting_address.get()};
        for (std::uint16_t i = 0;
             (i < slave_type::max_read_registers)
             && ((i + holding_collect_start) < slave.holding_registers().size())
             && (i < holding_collect_num);
             i++) {
            holding_collect[i] = slave.holding_registers()[i + holding_collect_start];
        }
         return_type data{{slave.id(), pack.header().function_code},
                                static_cast<std::uint8_t>(holding_collect_num * 2),
                                holding_collect_num,
                                holding_collect.begin()};

        slave.output().template serialize<header, std::uint8_t, msb_t<std::uint16_t>, crc16ansi>(
            data);
    }
    return exception::no_error;
}

template <std::uint8_t Id, std::uint16_t Inputs, std::uint16_t Coils, std::uint16_t InputRegisters,
          std::uint16_t HoldingRegisters, std::uint16_t Fifo>
exception
read_input_regs(modbus_slave<Id, Inputs, Coils, InputRegisters, HoldingRegisters, Fifo>& slave)
{
    using slave_type  = modbus_slave<Id, Inputs, Coils, InputRegisters, HoldingRegisters, Fifo>;
    using return_type = typename slave_type::msg_type::template fields_in<header, std::uint8_t,
                                                                          msb_t<std::uint16_t>>;
    //=========Check parameters=====================================================================
    if (slave_type::request_type_read::length != slave.input().size()) {
        return exception::bad_data;
    }
    auto pack = slave_type::request_type_read::deserialize(slave.input().storage().begin());
    if (!pack.valid()) {
        slave.increment_counter(diagnostics_sub_function::return_bus_comm_error_count);
        return exception::bad_crc;
    }
    if ((pack.fields().quantity < 1) || (pack.fields().quantity > slave_type::max_read_registers)) {
        return exception::illegal_data_value;
    }
    if (!slave_type::address_valid(pack.fields().starting_address.get(),
                                   pack.fields().quantity.get(), slave.input_registers().size())) {
        return exception::illegal_data_address;
    }
    //=========Request processing===================================================================
    if (pack.fields().quantity == 0) {
        packet<header, std::uint8_t, crc16ansi> ret_pack{{slave.id(), pack.header().function_code},
                                                         {0}};
    } else {
        static std::array<msb_t<std::uint16_t>, slave_type::max_read_registers> inputs_collect;
        const std::uint16_t                                                     inputs_collect_num{
            static_cast<std::uint16_t>(pack.fields().quantity.get())};
        const std::uint16_t inputs_collect_start{pack.fields().starting_address.get()};
        for (std::uint16_t i = 0; (i < slave_type::max_read_registers)
                                  && ((i + inputs_collect_start) < slave.input_registers().size())
                                  && (i < inputs_collect_num);
             i++) {
            inputs_collect[i] = slave.input_registers()[i + inputs_collect_start];
        }
        return_type data{{slave.id(), pack.header().function_code},
                                static_cast<std::uint8_t>(inputs_collect_num * 2),
                                inputs_collect_num,
                                inputs_collect.begin()};
        slave.output().template serialize<header, std::uint8_t, msb_t<std::uint16_t>, crc16ansi>(
            data);
    }
    return exception::no_error;
}

}    // namespace xitren::components::modbus
