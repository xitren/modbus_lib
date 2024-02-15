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
 * @brief This function is used to respond to a request for device identification information.
 *
 * @tparam TInputs The input data type.
 * @tparam TCoils The coil data type.
 * @tparam TInputRegisters The input register data type.
 * @tparam THoldingRegisters The holding register data type.
 * @tparam Fifo The FIFO size.
 * @param slave The Modbus slave object.
 * @return exception The exception code.
 */
template <typename TInputs, typename TCoils, typename TInputRegisters, typename THoldingRegisters, std::uint16_t Fifo>
exception
identification(slave_base<TInputs, TCoils, TInputRegisters, THoldingRegisters, Fifo>& slave)
{
    using slave_type  = slave_base<TInputs, TCoils, TInputRegisters, THoldingRegisters, Fifo>;
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
        /**
         * @brief Get the vendor name from the Modbus slave object.
         *
         * @param slave The Modbus slave object.
         * @return std::string The vendor name.
         */
        auto str = slave.vendor_name().substr(0, modbus_base::max_pdu_length - sizeof(response_identification));
        data.fields.object_len = data.size = str.size();
        data.data                          = str.data();
    } break;
    case static_cast<std::uint8_t>(object_id_code::product_code): {
        /**
         * @brief Get the product code from the Modbus slave object.
         *
         * @param slave The Modbus slave object.
         * @return std::string The product code.
         */
        auto str = slave.product_code().substr(0, modbus_base::max_pdu_length - sizeof(response_identification));
        data.fields.object_len = data.size = str.size();
        data.data                          = str.data();
    } break;
    case static_cast<std::uint8_t>(object_id_code::major_minor_revision): {
        /**
         * @brief Get the major and minor revision from the Modbus slave object.
         *
         * @param slave The Modbus slave object.
         * @return std::string The major and minor revision.
         */
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

}    // namespace xitren::modbus::functions
