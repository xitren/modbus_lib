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
 * @brief A Modbus Read Log request
 *
 * @param slave The Modbus slave device to send the request to
 * @param callback The function to call when the response is received
 */
class get_log_lvl : public command {
public:
    /**
     * @brief Constructs a new Get Log Level Modbus command
     *
     * @param slave The Modbus slave device to send the request to
     * @param callback The function to call when the response is received
     */
    get_log_lvl(std::uint8_t slave, types::callback_function_type callback) noexcept
        : command{slave, 0}, callback_{std::move(callback)}
    {
        if (!msg_output_.template serialize<header, std::uint8_t, std::uint8_t, crc16ansi>(
                {{slave, static_cast<uint8_t>(function::get_current_log_level)}, {}, 0, nullptr})) {
            error(exception::illegal_data_address);
            return;
        }
    }

    /**
     * @brief Returns an iterator to the beginning of the command data
     *
     * @return iterator
     */
    inline iterator
    begin() noexcept override
    {
        return msg_output_.storage().begin();
    }

    /**
     * @brief Returns an iterator to the beginning of the command data
     *
     * @return const_iterator
     */
    inline const_iterator
    begin() const noexcept override
    {
        return msg_output_.storage().begin();
    }

    /**
     * @brief Returns an iterator to the end of the command data
     *
     * @return iterator
     */
    inline iterator
    end() noexcept override
    {
        return msg_output_.storage().end();
    }

    /**
     * @brief Returns an iterator to the end of the command data
     *
     * @return const_iterator
     */
    inline const_iterator
    end() const noexcept override
    {
        return msg_output_.storage().end();
    }

    /**
     * @brief Returns the size of the command data
     *
     * @return std::size_t
     */
    inline std::size_t
    size() noexcept override
    {
        return msg_output_.size();
    }

    /**
     * @brief Returns the size of the command data
     *
     * @return std::size_t
     */
    inline std::size_t
    size() const noexcept override
    {
        return msg_output_.size();
    }

    /**
     * @brief Returns a reference to the command data
     *
     * @return msg_type&
     */
    inline msg_type&
    msg() noexcept
    {
        return msg_output_;
    }

    /**
     * @brief Called when no response is received
     *
     */
    void
    no_answer() noexcept override
    {
        callback_(error(exception::bad_slave));
    }

    /**
     * @brief Creates a new instance of the command with the same properties
     *
     * @param vault The memory area to allocate the new command in
     * @return modbus_command* A pointer to the new command
     */
    command*
    clone(command_vault_type& vault) const noexcept override
    {
        static_assert(command::command_buffer_max >= sizeof(get_log_lvl),
                      "Command realization size exceeded storage area!");
        return new (&vault) get_log_lvl(*this);
    }

    /**
     * @brief Creates a new instance of the command with the same properties
     *
     * @return std::shared_ptr<modbus_command> A shared pointer to the new command
     */
    std::shared_ptr<command>
    clone() const noexcept override
    {
        return std::make_shared<get_log_lvl>(*this);
    }

    /**
     * @brief Called when a response is received
     *
     * @param message The Modbus response message
     * @return exception The error code returned by the Modbus device
     */
    exception
    receive(msg_type const& message) noexcept override
    {
        auto [pack, err] = input_msg<header, std::uint8_t, std::uint8_t>(slave(), message);
        if (error(err) != exception::no_error) [[unlikely]]
            return err;
        if (pack.size > modbus_base::max_read_registers) [[unlikely]]
            return exception::illegal_data_value;
        callback_(exception::no_error);
        return exception::no_error;
    }

    /**
     * @brief Destructor
     *
     */
    ~get_log_lvl() noexcept override = default;

private:
    types::callback_function_type callback_;
    msg_type                      msg_output_{};
};

}    // namespace xitren::modbus::commands
