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

/**
 * @brief A class representing a Modbus write single register request
 *
 * This class represents a Modbus write single register request, which is used to write a single 16-bit register
 * on a Modbus device. It contains the slave device address, the register address to write to, and the value to
 * write to the register.
 *
 * The write single register request is sent to the Modbus device, and the response is handled by the
 * receive() method. If the response indicates an error, the error is passed to the user-defined callback
 * function.
 */
class write_register : public command {
public:
    /**
     * @brief Construct a new write register object
     *
     * @param slave The slave device address
     * @param address The register address to write to
     * @param val The value to write to the register
     * @param callback The function to call when the response is received
     */
    write_register(std::uint8_t slave, std::uint16_t address, std::uint16_t val,
                   types::callback_function_type callback) noexcept
        : command{slave, address}, callback_{std::move(callback)}
    {
        std::uint32_t const max_address = address;
        if (max_address > static_cast<std::uint32_t>(std::numeric_limits<std::uint16_t>::max())) [[unlikely]] {
            error(exception::illegal_data_address);
            return;
        }
        value(val);
    }

    /**
     * @brief Set the value to write to the register
     *
     * @param val The value to write to the register
     */
    void
    value(std::uint16_t val) noexcept
    {
        if (!msg_output_.template serialize<header, request_fields_read, std::uint8_t, crc16ansi>(
                {{slave_, static_cast<uint8_t>(function::write_single_register)}, {address_, val}, 0, nullptr})) {
            error(exception::illegal_data_address);
            return;
        }
    }

    /**
     * @brief Get an iterator to the beginning of the message data
     *
     * @return iterator to the beginning of the message data
     */
    inline iterator
    begin() noexcept override
    {
        return msg_output_.storage().begin();
    }

    /**
     * @brief Get an iterator to the beginning of the message data
     *
     * @return const_iterator to the beginning of the message data
     */
    inline const_iterator
    begin() const noexcept override
    {
        return msg_output_.storage().begin();
    }

    /**
     * @brief Get an iterator to the end of the message data
     *
     * @return iterator to the end of the message data
     */
    inline iterator
    end() noexcept override
    {
        return msg_output_.storage().end();
    }

    /**
     * @brief Get an iterator to the end of the message data
     *
     * @return const_iterator to the end of the message data
     */
    inline const_iterator
    end() const noexcept override
    {
        return msg_output_.storage().end();
    }

    /**
     * @brief Get the size of the message data
     *
     * @return std::size_t size of the message data
     */
    inline std::size_t
    size() noexcept override
    {
        return msg_output_.size();
    }

    /**
     * @brief Get the size of the message data
     *
     * @return std::size_t size of the message data
     */
    inline std::size_t
    size() const noexcept override
    {
        return msg_output_.size();
    }

    /**
     * @brief Get a reference to the message data
     *
     * @return msg_type& reference to the message data
     */
    inline msg_type&
    msg() noexcept
    {
        return msg_output_;
    }

    /**
     * @brief Handle a response with no data
     */
    void
    no_answer() noexcept override
    {
        callback_(error(exception::bad_slave));
    }

    /**
     * @brief Clone the command into the given command vault
     *
     * @param vault The command vault to clone the command into
     * @return modbus_command* A pointer to the cloned command
     */
    command*
    clone(command_vault_type& vault) const noexcept override
    {
        static_assert(command::command_buffer_max >= sizeof(write_register),
                      "Command realization size exceeded storage area!");
        return new (&vault) write_register(*this);
    }

    /**
     * @brief Clone the command
     *
     * @return std::shared_ptr<modbus_command> A shared pointer to the cloned command
     */
    std::shared_ptr<command>
    clone() const noexcept override
    {
        return std::make_shared<write_register>(*this);
    }

    /**
     * @brief Handle a response to the command
     *
     * @param message The response message
     * @return exception The error code returned in the response
     */
    exception
    receive(msg_type const& message) noexcept override
    {
        auto [pack, err] = input_msg<header, std::uint8_t, std::uint8_t>(slave(), message);
        if (error(err) != exception::no_error) [[unlikely]]
            return err;
        callback_(exception::no_error);
        return exception::no_error;
    }

    /**
     * @brief Destroy the write register object
     */
    ~write_register() noexcept override = default;

private:
    types::callback_function_type callback_;
    msg_type                      msg_output_{};
};

}    // namespace xitren::modbus::commands
