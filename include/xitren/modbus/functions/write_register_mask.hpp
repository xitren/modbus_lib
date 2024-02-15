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

/*!
 * @brief Writes a 16-bit register using a mask.
 *
 * This function writes to a 16-bit register starting at the specified address, using a mask to modify the existing
 * contents of the register. The function takes the following parameters:
 *
 * - `slave`: A reference to the Modbus slave object.
 * - `pack`: A `request_fields_wr_mask` object that contains the request parameters.
 *
 * The function performs the following steps:
 *
 * 1. Checks the length of the input buffer. If the length is not correct, the function returns an `exception::bad_data`
 *    error.
 * 2. Deserializes the request parameters from the input buffer. If the deserialization fails, the function returns an
 *    `exception::bad_data` error.
 * 3. Checks if the starting address specified in the request is within the range of holding registers. If it is not,
 * the function returns an `exception::illegal_data_address` error.
 * 4. Reads the current value of the register at the specified address.
 * 5. Writes the new value to the register, calculated by applying the mask to the current value.
 * 6. Updates the changed_registers map with the new value.
 * 7. Copies the input buffer to the output buffer.
 * 8. Sets the size of the output buffer to the size of the input buffer.
 * 9. Returns an `exception::no_error` error.
 *
 * @tparam TInputs Type of the input buffer.
 * @tparam TCoils Type of the coils buffer.
 * @tparam TInputRegisters Type of the input registers buffer.
 * @tparam THoldingRegisters Type of the holding registers buffer.
 * @tparam Fifo Size of the FIFO queue.
 *
 * @param slave A reference to the Modbus slave object.
 * @param pack A `request_fields_wr_mask` object that contains the request parameters.
 * @return An `exception` value indicating the result of the operation.
 */
template <typename TInputs, typename TCoils, typename TInputRegisters, typename THoldingRegisters, std::uint16_t Fifo>
exception
write_register_mask(slave_base<TInputs, TCoils, TInputRegisters, THoldingRegisters, Fifo>& slave)
{
    using slave_type = slave_base<TInputs, TCoils, TInputRegisters, THoldingRegisters, Fifo>;
    //=========Check parameters=====================================================================
    if (slave_type::request_type_read::length != slave.input().size()) {
        return exception::bad_data;
    }
    auto pack = slave.input().template deserialize_no_check<header, request_fields_wr_mask, std::uint8_t, crc16ansi>();
    if (pack.fields->starting_address >= slave.holding_registers().size()) {
        return exception::illegal_data_address;
    }
    //=========Request processing===================================================================
    std::uint16_t const current = slave.holding_registers()[pack.fields->starting_address.get()];
    std::uint16_t const value
        = (current & pack.fields->and_mask.get()) | (pack.fields->or_mask.get() & (~pack.fields->and_mask.get()));
    slave.holding_registers()[pack.fields->starting_address.get()] = value;
    slave.changed_holding(pack.fields->starting_address.get(), value);
    std::copy(slave.input().storage().begin(), slave.input().storage().begin() + slave.input().size(),
              slave.output().storage().begin());
    slave.output().size(slave.input().size());
    return exception::no_error;
}

}    // namespace xitren::modbus::functions
