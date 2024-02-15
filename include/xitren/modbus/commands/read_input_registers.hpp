/*!
     _ _
__ _(_) |_ _ _ ___ _ _
\ \ / |  _| '_/ -_) ' \
/_\_\_|\__|_| \___|_||_|
* @date 15.02.2024
*/
#pragma once

#include "command.hpp"

namespace xitren::modbus::commands {

/*!
 * @brief Class for representing a Modbus Read Input Registers request
 *
 * This class represents a Modbus Read Input Registers request. It is used to request the
 * contents of a contiguous block of input registers from a Modbus device.
 *
 * @par Example Usage
 * @code
 * // Create a read input registers command
 * auto cmd = std::make_shared<read_input_registers>(0x01, 0x0000, 10,
 *                                                    [](auto err, auto begin, auto end) {
 *                                                        if (err == exception::no_error) {
 *                                                            // Do something with the data
 *                                                        }
 *                                                    });
 *
 * // Send the command
 * master.send(cmd);
 * @endcode
 */
class read_input_registers : public command {
public:
    read_input_registers(std::uint8_t slave, std::uint16_t address, std::size_t size,
                         types::callback_regs_type callback) noexcept
        : command{slave, address}, size_{size}, callback_{std::move(callback)}
    {
        std::uint32_t const max_address = address + size_ - 1;
        if (max_address > static_cast<std::uint32_t>(std::numeric_limits<std::uint16_t>::max())) [[unlikely]] {
            error(exception::illegal_data_address);
            return;
        }
        if (!msg_output_.template serialize<header, request_fields_read, func::msb_t<std::uint16_t>, crc16ansi>(
                {{slave, static_cast<uint8_t>(function::read_input_registers)}, {address, size_}, 0, nullptr})) {
            error(exception::illegal_data_address);
            return;
        }
    }

    inline iterator
    begin() noexcept override
    {
        return msg_output_.storage().begin();
    }

    inline const_iterator
    begin() const noexcept override
    {
        return msg_output_.storage().begin();
    }

    inline iterator
    end() noexcept override
    {
        return msg_output_.storage().end();
    }

    inline const_iterator
    end() const noexcept override
    {
        return msg_output_.storage().end();
    }

    inline std::size_t
    size() noexcept override
    {
        return msg_output_.size();
    }

    inline std::size_t
    size() const noexcept override
    {
        return msg_output_.size();
    }

    inline msg_type&
    msg() noexcept
    {
        return msg_output_;
    }

    void
    no_answer() noexcept override
    {
        callback_(error(exception::bad_slave), nullptr, nullptr);
    }

    command*
    clone(command_vault_type& vault) const noexcept override
    {
        static_assert(command::command_buffer_max >= sizeof(read_input_registers),
                      "Command realization size exceeded storage area!");
        return new (&vault) read_input_registers(*this);
    }

    std::shared_ptr<command>
    clone() const noexcept override
    {
        return std::make_shared<read_input_registers>(*this);
    }

    exception
    receive(msg_type const& message) noexcept override
    {
        static std::array<std::uint16_t, modbus_base::max_read_registers> values{};
        auto [pack, err] = input_msg<header, std::uint8_t, func::msb_t<std::uint16_t>>(slave(), message);
        if (error(err) != exception::no_error) [[unlikely]]
            return err;
        if (pack.size > modbus_base::max_read_registers) [[unlikely]]
            return exception::illegal_data_value;
        for (std::size_t i{}; (i < values.size()) && (i < pack.size); i++) {
            values[i] = pack.data[i].get();
        }
        callback_(exception::no_error, values.begin(), values.begin() + pack.size);
        return exception::no_error;
    }

    ~read_input_registers() noexcept override = default;

private:
    std::size_t               size_;
    types::callback_regs_type callback_;
    msg_type                  msg_output_{};
};

}    // namespace xitren::modbus::commands