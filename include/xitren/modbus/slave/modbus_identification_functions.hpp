#pragma once

#include <xitren/modbus/modbus.hpp>
#include <xitren/modbus/packet.hpp>

namespace xitren::modbus {

template <typename TInputs, typename TCoils, typename TInputRegisters, typename THoldingRegisters, std::uint16_t Fifo>
exception
identification(modbus_slave_base<TInputs, TCoils, TInputRegisters, THoldingRegisters, Fifo>& slave)
{
    using slave_type  = modbus_slave_base<TInputs, TCoils, TInputRegisters, THoldingRegisters, Fifo>;
    using return_type = typename slave_type::msg_type::template fields_in<header, response_identification, char>;
    //=========Check parameters=====================================================================
    auto pack = slave.input().template deserialize_no_check<header, request_identification, std::uint8_t, crc16ansi>();
    if (pack.fields->mei_type != modbus_base::mei_type) {
        return exception::illegal_data_value;
    }
    if (pack.fields->read_mode != static_cast<std::uint8_t>(read_device_id_code::individual_access)) {
        return exception::illegal_data_value;
    }
    if (pack.fields->object_id >= static_cast<std::uint8_t>(object_id_code::max)) {
        return exception::illegal_data_address;
    }
    //=========Request processing===================================================================
    return_type data{{slave.id(), pack.header->function_code},
                     {modbus_base::mei_type, pack.fields->read_mode,
                      static_cast<std::uint8_t>(conformity_code::basic_identification_ind),
                      modbus_base::no_more_follows, modbus_base::no_more_follows, 1, pack.fields->object_id, 0},
                     0,
                     {nullptr}};
    switch (pack.fields->object_id) {
    case static_cast<std::uint8_t>(object_id_code::vendor_name): {
        auto str = slave.vendor_name().substr(0, modbus_base::max_pdu_length - sizeof(response_identification));
        data.fields.object_len = data.size = str.size();
        data.data                          = str.data();
    } break;
    case static_cast<std::uint8_t>(object_id_code::product_code): {
        auto str = slave.product_code().substr(0, modbus_base::max_pdu_length - sizeof(response_identification));
        data.fields.object_len = data.size = str.size();
        data.data                          = str.data();
    } break;
    case static_cast<std::uint8_t>(object_id_code::major_minor_revision): {
        auto str
            = slave.major_minor_revision().substr(0, modbus_base::max_pdu_length - sizeof(response_identification));
        data.fields.object_len = data.size = str.size();
        data.data                          = str.data();
    } break;
    default:
        return exception::unknown_exception;
    }
    slave.output().template serialize<header, response_identification, char, crc16ansi>(data);
    return exception::no_error;
}

}    // namespace xitren::modbus
