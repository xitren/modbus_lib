#pragma once

#include <xitren/modbus/modbus.hpp>
#include <xitren/modbus/packet.hpp>

namespace xitren::modbus {
template <typename TInputs, typename TCoils, typename TInputRegisters, typename THoldingRegisters, std::uint16_t Fifo>
exception
read_log(modbus_slave_base<TInputs, TCoils, TInputRegisters, THoldingRegisters, Fifo>& slave)
{
    using slave_type  = modbus_slave_base<TInputs, TCoils, TInputRegisters, THoldingRegisters, Fifo>;
    using return_type = typename slave_type::msg_type::template fields_in<header, request_fields_log, std::uint8_t>;
    //=========Check parameters=====================================================================
    if (slave_type::request_type_log::length != slave.input().size()) {
        return exception::bad_data;
    }
    auto pack = slave.input().template deserialize_no_check<header, request_fields_log, std::uint8_t, crc16ansi>();
    //=========Request processing===================================================================
    static std::array<std::uint8_t, slave_type::max_read_log_bytes> inputs_collect;
    auto                                                            address = pack.fields->address.get();
    auto                                                            size    = pack.fields->quantity.get();
    auto head = static_cast<std::uint16_t>(slave.log().head());
    auto tail = static_cast<std::uint16_t>(slave.log().tail());
    if (address < head || address > tail) {
        address = head;
    }
    size = std::min(size, static_cast<std::uint16_t>(tail - address));
    std::copy(slave.log().begin() + address, slave.log().begin() + address + size, inputs_collect.begin());
    return_type data{{slave.id(), pack.header->function_code}, {address, size}, size, inputs_collect.begin()};
    slave.output().template serialize<header, request_fields_log, std::uint8_t, crc16ansi>(data);
    return exception::no_error;
}

template <typename TInputs, typename TCoils, typename TInputRegisters, typename THoldingRegisters, std::uint16_t Fifo>
exception
set_max_log_level(modbus_slave_base<TInputs, TCoils, TInputRegisters, THoldingRegisters, Fifo>& slave [[maybe_unused]])
{
    using slave_type  = modbus_slave_base<TInputs, TCoils, TInputRegisters, THoldingRegisters, Fifo>;
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

template <typename TInputs, typename TCoils, typename TInputRegisters, typename THoldingRegisters, std::uint16_t Fifo>
exception
get_current_log_level(modbus_slave_base<TInputs, TCoils, TInputRegisters, THoldingRegisters, Fifo>& slave)
{
    using slave_type  = modbus_slave_base<TInputs, TCoils, TInputRegisters, THoldingRegisters, Fifo>;
    using return_type = typename slave_type::msg_type::template fields_in<header, std::uint8_t, std::uint8_t>;
    //=========Check parameters=====================================================================
    auto pack = slave.input().template deserialize_no_check<header, std::uint8_t, std::uint8_t, crc16ansi>();
    //=========Request processing===================================================================
    auto        log_mode = GET_LEVEL();
    return_type data{slave.id(), pack.header->function_code, static_cast<std::uint8_t>(log_mode), 0, nullptr};
    slave.output().template serialize<header, std::uint8_t, std::uint8_t, crc16ansi>(data);
    return exception::no_error;
}

}    // namespace xitren::modbus
