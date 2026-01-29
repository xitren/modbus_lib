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
 * @brief Handle custom Read Log request (0x41).
 *
 * The log is a circular buffer of bytes recorded by the slave. The request
 * provides a starting address and quantity; the handler clamps the range to
 * the available log window and returns the requested slice.
 *
 * Request payload:
 * - address: starting log index
 * - quantity: number of bytes to return
 *
 * Response payload:
 * - address: actual start index used
 * - quantity: number of bytes returned
 * - data: raw log bytes
 *
 * @tparam TInputs Input discretes container type.
 * @tparam TCoils Coils container type.
 * @tparam TInputRegisters Input registers container type.
 * @tparam THoldingRegisters Holding registers container type.
 * @tparam Fifo FIFO depth.
 * @param slave Reference to the slave instance handling the request.
 * @return exception::no_error on success or a Modbus exception code otherwise.
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
    std::array<std::uint8_t, slave_type::max_read_log_bytes> inputs_collect{};
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
