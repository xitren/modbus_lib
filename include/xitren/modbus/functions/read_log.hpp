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
 * @brief Reads the log of the slave.
 *
 * @tparam TInputs The input bit field type.
 * @tparam TCoils The coil bit field type.
 * @tparam TInputRegisters The input register field type.
 * @tparam THoldingRegisters The holding register field type.
 * @tparam Fifo The fifo size.
 * @param slave The slave to read the log from.
 * @return exception An exception code indicating the result of the operation.
 *
 * This function reads the log of the slave. The log is a circular buffer that stores the last N requests that were made
 * to the slave. The size of the log is defined by the Fifo template parameter.
 *
 * The request structure for this function is defined as follows:
 *
 * | Byte  | Name | Size | Description |
 * | ----- | ---- | ---- | ----------- |
 * | 1     | Function Code | 1 | The function code for this request is 0x01. |
 * | 2-3   | Starting Address | 2 | The starting address of the log to read. |
 * | 4-5   | Quantity | 2 | The number of log entries to read. |
 *
 * The response structure for this function is defined as follows:
 *
 * | Byte  | Name | Size | Description |
 * | ----- | ---- | ---- | ----------- |
 * | 1     | Function Code | 1 | The function code for this response is 0x01. |
 * | 2-3   | Starting Address | 2 | The starting address of the log. |
 * | 4-5   | Quantity | 2 | The number of log entries returned. |
 * | 6-n   | Data | n | The data from the log. |
 *
 * The data returned is a sequence of log entries, where each log entry is a variable length depending on the data type
 * of the slave. For example, if the slave is using input registers, each log entry will be 2 bytes.
 *
 * The log is circular, so if the starting address specified is outside of the range of the log, the log will be wrapped
 * around to the beginning. For example, if the log size is 10 and the starting address is 15, then only 5 log entries
 * will be returned.
 */
template <typename TInputs, typename TCoils, typename TInputRegisters, typename THoldingRegisters, std::uint16_t Fifo>
exception
read_log(slave_base<TInputs, TCoils, TInputRegisters, THoldingRegisters, Fifo>& slave)
{
    using slave_type  = slave_base<TInputs, TCoils, TInputRegisters, THoldingRegisters, Fifo>;
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

}    // namespace xitren::modbus::functions
