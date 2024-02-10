#pragma once

#include <xitren/modbus/modbus.hpp>
#include <xitren/modbus/packet.hpp>

namespace xitren::modbus {

template <typename TInputs, typename TCoils, typename TInputRegisters, typename THoldingRegisters, std::uint16_t Fifo>
exception
read_fifo(modbus_slave_base<TInputs, TCoils, TInputRegisters, THoldingRegisters, Fifo>& slave)
{
    using slave_type = modbus_slave_base<TInputs, TCoils, TInputRegisters, THoldingRegisters, Fifo>;
    using return_type =
        typename slave_type::msg_type::template fields_in<header, request_fields_fifo, func::msb_t<std::uint16_t>>;
    //=========Check parameters=====================================================================
    if (slave_type::request_type_fifo::length != slave.input().size()) {
        return exception::bad_data;
    }
    auto pack
        = slave.input().template deserialize_no_check<header, func::msb_t<std::uint16_t>, std::uint8_t, crc16ansi>();
    if (!((slave.fifo().head() <= pack.fields->get()) && (pack.fields->get() < slave.fifo().tail()))) {
        return exception::illegal_data_address;
    }
    //=========Request processing===================================================================
    std::size_t const   start{pack.fields->get() - slave.fifo().head()};
    std::uint16_t const count{
        static_cast<std::uint16_t>(std::min(slave_type::max_read_fifo, static_cast<std::uint16_t>(slave.fifo().size()))
                                   - static_cast<std::uint16_t>(start))};
    static std::array<func::msb_t<std::uint16_t>, slave_type::max_read_fifo> inputs_collect;
    std::copy(slave.fifo().begin() + start, slave.fifo().begin() + start + count, inputs_collect.begin());
    static return_type data{{slave.id(), pack.header->function_code},
                            {count * sizeof(std::uint16_t) + sizeof(std::uint16_t), count},
                            count,
                            inputs_collect.begin()};
    slave.output().template serialize<header, request_fields_fifo, func::msb_t<std::uint16_t>, crc16ansi>(data);

    return exception::no_error;
}

}    // namespace xitren::modbus
