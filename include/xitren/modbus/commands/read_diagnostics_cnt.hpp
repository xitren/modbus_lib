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
 * @brief A class that represents a read diagnostics count Modbus command
 *
 * This class represents a Modbus command for reading the diagnostics count of a device. It inherits from the
 * modbus::master::command class.
 */
class read_diagnostics_cnt : public command {
public:
    /**
     * @brief Constructs a new read diagnostics count object
     *
     * @param slave The slave address of the device to read the diagnostics count from
     * @param sub The sub function of the read diagnostics count to execute
     * @param callback The function to be called with the results of the read diagnostics count
     */
    read_diagnostics_cnt(std::uint8_t slave, diagnostics_sub_function sub, types::callback_regs_type callback) noexcept
        : command{slave, static_cast<uint16_t>(sub)}, callback_{std::move(callback)}
    {
        if ((sub != diagnostics_sub_function::return_bus_message_count)
            && (sub != diagnostics_sub_function::return_bus_comm_error_count)
            && (sub != diagnostics_sub_function::return_server_exception_error_count)
            && (sub != diagnostics_sub_function::return_server_message_count)
            && (sub != diagnostics_sub_function::return_server_no_response_count)
            && (sub != diagnostics_sub_function::return_server_nak_count)
            && (sub != diagnostics_sub_function::return_server_busy_count)
            && (sub != diagnostics_sub_function::return_bus_char_overrun_count)) [[unlikely]] {
            error(exception::illegal_data_address);
            return;
        }
        if (!msg_output_.template serialize<header, func::msb_t<std::uint16_t>, func::msb_t<std::uint16_t>, crc16ansi>(
                {{slave, static_cast<uint8_t>(function::diagnostic)}, static_cast<uint16_t>(sub), 0, nullptr})) {
            error(exception::illegal_data_address);
            return;
        }
    }

    /**
     * @brief Returns an iterator to the beginning of the output message
     *
     * @return iterator An iterator to the beginning of the output message
     */
    inline iterator
    begin() noexcept override
    {
        return msg_output_.storage().begin();
    }

    /**
     * @brief Returns an iterator to the beginning of the output message
     *
     * @return const_iterator An iterator to the beginning of the output message
     */
    inline const_iterator
    begin() const noexcept override
    {
        return msg_output_.storage().begin();
    }

    /**
     * @brief Returns an iterator to the end of the output message
     *
     * @return iterator An iterator to the end of the output message
     */
    inline iterator
    end() noexcept override
    {
        return msg_output_.storage().end();
    }

    /**
     * @brief Returns an iterator to the end of the output message
     *
     * @return const_iterator An iterator to the end of the output message
     */
    inline const_iterator
    end() const noexcept override
    {
        return msg_output_.storage().end();
    }

    /**
     * @brief Returns the size of the output message
     *
     * @return std::size_t The size of the output message
     */
    inline std::size_t
    size() noexcept override
    {
        return msg_output_.size();
    }

    /**
     * @brief Returns the size of the output message
     *
     * @return std::size_t The size of the output message
     */
    inline std::size_t
    size() const noexcept override
    {
        return msg_output_.size();
    }

    /**
     * @brief Sets the error state of the command to exception::bad_slave
     *
     */
    void
    no_answer() noexcept override
    {
        callback_(error(exception::bad_slave), nullptr, nullptr);
    }

    /**
     * @brief Creates a new modbus_command object that is a clone of the current object
     *
     * @param vault The command_vault_type object used to allocate memory for the clone
     * @return modbus_command* A pointer to the newly created clone
     */
    command*
    clone(command_vault_type& vault) const noexcept override
    {
        static_assert(command::command_buffer_max >= sizeof(read_diagnostics_cnt),
                      "Command realization size exceeded storage area!");
        return new (&vault) read_diagnostics_cnt(*this);
    }

    /**
     * @brief Creates a new shared_ptr to a modbus_command object that is a clone of the current object
     *
     * @return std::shared_ptr<modbus_command> A shared_ptr to the newly created clone
     */
    std::shared_ptr<command>
    clone() const noexcept override
    {
        return std::make_shared<read_diagnostics_cnt>(*this);
    }

    /**
     * @brief Processes an incoming modbus message and updates the error state and values of the command
     *
     * @param message The incoming modbus message
     * @return exception The error state of the command after processing the message
     */
    exception
    receive(msg_type const& message) noexcept override
    {
        static std::array<std::uint16_t, modbus_base::max_read_registers> values{};
        auto [pack, err] = input_msg<header, func::msb_t<std::uint16_t>, func::msb_t<std::uint16_t>>(slave(), message);
        if (error(err) != exception::no_error) [[unlikely]] {
            callback_(err, nullptr, nullptr);
            return err;
        }
        if (pack.size > modbus_base::max_read_registers) [[unlikely]] {
            callback_(err, nullptr, nullptr);
            return exception::illegal_data_value;
        }
        for (std::size_t i{}; (i < values.size()) && (i < pack.size); i++) {
            values[i] = pack.data[i].get();
        }
        callback_(exception::no_error, values.begin(), values.begin() + pack.size);
        return exception::no_error;
    }

    /**
     * @brief Default destructor
     *
     */
    ~read_diagnostics_cnt() noexcept override = default;

private:
    types::callback_regs_type callback_;
    msg_type                  msg_output_{};
};

}    // namespace xitren::modbus::commands
