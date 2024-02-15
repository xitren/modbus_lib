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
 * @brief A class representing a Modbus write multiple registers request
 *
 * This class represents a Modbus write multiple registers request, which is used to write a series of 16-bit registers
 * on a Modbus device. It contains the slave device address, the first register address to write to, and the value to
 * write to the register.
 *
 * The write multiple registers request is sent to the Modbus device, and the response is handled by the
 * receive() method. If the response indicates an error, the error is passed to the user-defined callback
 * function.
 *
 * @tparam Size the size of the array of values to write
 */
class write_registers : public command {
public:
    /**
     * @brief Constructs a new write multiple registers command
     *
     * @param slave the slave device address
     * @param address the first register address to write to
     * @param vals the values to write to the registers
     */
    template <std::size_t Size>
    write_registers(std::uint8_t slave, std::uint16_t address, std::array<std::uint16_t, Size> const& vals) noexcept
        : write_registers{slave, address, vals, nullptr}
    {}

    /**
     * @brief Constructs a new write multiple registers command with a user-defined callback
     *
     * @param slave the slave device address
     * @param address the first register address to write to
     * @param vals the values to write to the registers
     * @param callback the user-defined function to call with the response
     */
    template <std::size_t Size>
    write_registers(std::uint8_t slave, std::uint16_t address, std::array<std::uint16_t, Size> const& vals,
                    types::callback_function_type callback) noexcept
        : command{slave, address}, callback_{std::move(callback)}
    {
        value(vals);
    }

    /**
     * @brief Sets the values to write to the registers
     *
     * @param vals the values to write to the registers
     */
    template <std::size_t Size>
    void
    value(std::array<std::uint16_t, Size> const& vals) noexcept
    {
        static_assert(Size < modbus_base::max_write_registers, "Too much to write!");
        static std::array<func::msb_t<std::uint16_t>, Size> data_formatted;
        auto                                                it1{vals.begin()};
        auto                                                it2{data_formatted.begin()};
        for (; (it1 != vals.end()) && (it2 != data_formatted.end()); it1++, it2++) {
            (*it2) = (*it1);
        }
        if (!msg_output_.template serialize<header, request_fields_wr_single, func::msb_t<std::uint16_t>, crc16ansi>(
                {{slave_, static_cast<std::uint8_t>(function::write_multiple_registers)},
                 {address_, static_cast<std::uint16_t>(vals.size()), static_cast<std::uint8_t>(Size * 2)},
                 Size,
                 data_formatted.data()})) {
            error(exception::illegal_data_address);
            return;
        }
    }

    /**
     * @brief Returns an iterator to the beginning of the command data
     *
     * @return an iterator to the beginning of the command data
     */
    inline iterator
    begin() noexcept override
    {
        return msg_output_.storage().begin();
    }

    /**
     * @brief Returns an iterator to the beginning of the command data
     *
     * @return an iterator to the beginning of the command data
     */
    inline const_iterator
    begin() const noexcept override
    {
        return msg_output_.storage().begin();
    }

    /**
     * @brief Returns an iterator to the end of the command data
     *
     * @return an iterator to the end of the command data
     */
    inline iterator
    end() noexcept override
    {
        return msg_output_.storage().end();
    }

    /**
     * @brief Returns an iterator to the end of the command data
     *
     * @return an iterator to the end of the command data
     */
    inline const_iterator
    end() const noexcept override
    {
        return msg_output_.storage().end();
    }

    /**
     * @brief Returns the size of the command data
     *
     * @return the size of the command data
     */
    inline std::size_t
    size() noexcept override
    {
        return msg_output_.size();
    }

    /**
     * @brief Returns the size of the command data
     *
     * @return the size of the command data
     */
    inline std::size_t
    size() const noexcept override
    {
        return msg_output_.size();
    }

    /**
     * @brief Returns a reference to the command message
     *
     * @return a reference to the command message
     */
    inline msg_type&
    msg() noexcept
    {
        return msg_output_;
    }

    /**
     * @brief Indicates that no response is expected from the device
     */
    void
    no_answer() noexcept override
    {
        callback_(error(exception::bad_slave));
    }

    /**
     * @brief Clones the command into the specified command vault
     *
     * @param vault the command vault
     * @return a pointer to the cloned command
     */
    command*
    clone(command_vault_type& vault) const noexcept override
    {
        static_assert(command::command_buffer_max >= sizeof(write_registers),
                      "Command realization size exceeded storage area!");
        return new (&vault) write_registers(*this);
    }

    /**
     * @brief Clones the command
     *
     * @return a shared pointer to the cloned command
     */
    std::shared_ptr<command>
    clone() const noexcept override
    {
        return std::make_shared<write_registers>(*this);
    }

    /**
     * @brief Processes the response to the command
     *
     * @param message the response message
     * @return the exception indicating any errors
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
     * @brief Destroys the write multiple registers command
     */
    ~write_registers() noexcept override = default;

private:
    /**
     * @brief The user-defined function to call with the response
     */
    types::callback_function_type callback_;

    /**
     * @brief The command message
     */
    msg_type msg_output_{};
};

}    // namespace xitren::modbus::commands