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
 * @brief A class that represents a Modbus read registers request
 *
 * This class represents a Modbus read registers request. It is used to request the
 * values of a set of registers from a Modbus device.
 *
 * @tparam Size The number of registers to read
 */
class read_registers : public command {
public:
    /**
     * @brief Constructs a read registers request
     *
     * Constructs a read registers request with the specified slave ID, starting register
     * address, and callback function.
     *
     * @param slave The Modbus slave ID
     * @param address The starting register address
     * @param callback The function to call with the register values
     */
    read_registers(std::uint8_t slave, std::uint16_t address, std::size_t size,
                   types::callback_regs_type callback) noexcept
        : command{slave, address}, size_{size}, callback_{std::move(callback)}
    {
        std::uint32_t const max_address = address + size_ - 1;
        if (max_address > static_cast<std::uint32_t>(std::numeric_limits<std::uint16_t>::max())) [[unlikely]] {
            error(exception::illegal_data_address);
            return;
        }
        if (!msg_output_.template serialize<header, request_fields_read, func::msb_t<std::uint16_t>, crc16ansi>(
                {{slave, static_cast<uint8_t>(function::read_holding_registers)}, {address, size_}, 0, nullptr})) {
            error(exception::illegal_data_address);
            return;
        }
    }

    /**
     * @brief Returns an iterator to the beginning of the request data
     *
     * Returns an iterator to the beginning of the request data.
     *
     * @return An iterator to the beginning of the request data
     */
    inline iterator
    begin() noexcept override
    {
        return msg_output_.storage().begin();
    }

    /**
     * @brief Returns an iterator to the beginning of the request data
     *
     * Returns an iterator to the beginning of the request data.
     *
     * @return An iterator to the beginning of the request data
     */
    inline const_iterator
    begin() const noexcept override
    {
        return msg_output_.storage().begin();
    }

    /**
     * @brief Returns an iterator to the end of the request data
     *
     * Returns an iterator to the end of the request data.
     *
     * @return An iterator to the end of the request data
     */
    inline iterator
    end() noexcept override
    {
        return msg_output_.storage().end();
    }

    /**
     * @brief Returns an iterator to the end of the request data
     *
     * Returns an iterator to the end of the request data.
     *
     * @return An iterator to the end of the request data
     */
    inline const_iterator
    end() const noexcept override
    {
        return msg_output_.storage().end();
    }

    /**
     * @brief Returns the size of the request data
     *
     * Returns the size of the request data, which is the number of registers to read.
     *
     * @return The size of the request data
     */
    inline std::size_t
    size() noexcept override
    {
        return msg_output_.size();
    }

    /**
     * @brief Returns the size of the request data
     *
     * Returns the size of the request data, which is the number of registers to read.
     *
     * @return The size of the request data
     */
    inline std::size_t
    size() const noexcept override
    {
        return msg_output_.size();
    }

    /**
     * @brief Returns a reference to the request message
     *
     * Returns a reference to the request message.
     *
     * @return A reference to the request message
     */
    inline msg_type&
    msg() noexcept
    {
        return msg_output_;
    }

    /**
     * @brief Called when no response is received
     *
     * Called when no response is received to this request. The default implementation
     * sets an error code and calls the callback function with a null pointer.
     */
    void
    no_answer() noexcept override
    {
        callback_(error(exception::bad_slave), nullptr, nullptr);
    }

    /**
     * @brief Clones the request
     *
     * Clones the request and returns a pointer to the new instance.
     *
     * @param vault A reference to the command vault
     * @return A pointer to the new instance
     */
    command*
    clone(command_vault_type& vault) const noexcept override
    {
        static_assert(command::command_buffer_max >= sizeof(read_registers),
                      "Command realization size exceeded storage area!");
        return new (&vault) read_registers(*this);
    }

    /**
     * @brief Creates a shared pointer to the request
     *
     * Creates a shared pointer to the request and returns it.
     *
     * @return A shared pointer to the request
     */
    std::shared_ptr<command>
    clone() const noexcept override
    {
        return std::make_shared<read_registers>(*this);
    }

    /**
     * @brief Called when a response is received
     *
     * Called when a response is received to this request. The default implementation
     * deserializes the response data and calls the callback function with the result.
     *
     * @param message The response message
     * @return The error code
     */
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

    /**
     * @brief Destructor
     */
    ~read_registers() noexcept override = default;

private:
    std::size_t               size_;
    types::callback_regs_type callback_;
    msg_type                  msg_output_{};
};

}    // namespace xitren::modbus::commands