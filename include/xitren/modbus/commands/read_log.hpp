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
 * @brief Command to read the custom log buffer (0x41).
 *
 * The request includes a log address and quantity. The response returns the
 * actual start address and the raw log bytes, which are forwarded to the
 * callback.
 *
 * @param slave The Modbus slave device to send the request to.
 * @param address The starting log index to read from.
 * @param size The number of log bytes to read.
 * @param callback The function to call when the response is received.
 */
class read_log : public command {
public:
    read_log(std::uint8_t slave, std::uint16_t address, std::size_t size, types::callback_logs_type callback) noexcept
        : command{slave, address}, size_{size}, callback_{std::move(callback)}
    {
        std::uint32_t const max_address = address + size_ - 1;
        if (max_address > static_cast<std::uint32_t>(std::numeric_limits<std::uint16_t>::max())) [[unlikely]] {
            error(exception::illegal_data_address);
            return;
        }
        // GCOVR_EXCL_START
        if (!msg_output_.template serialize<header, request_fields_log, func::msb_t<std::uint16_t>, crc16ansi>(
                {{slave, static_cast<uint8_t>(function::read_log)}, {address, size_}, 0, nullptr})) {
            error(exception::illegal_data_address);
            return;
        }
        // GCOVR_EXCL_STOP
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
        callback_(error(exception::bad_slave), 0, nullptr, nullptr);
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
        static_assert(command::command_buffer_max >= sizeof(read_log),
                      "Command realization size exceeded storage area!");
        return new (&vault) read_log(*this);
    }

    /**
     * @brief Creates a new instance of the command with the same properties
     *
     * @return std::shared_ptr<modbus_command> A shared pointer to the new command
     */
    std::shared_ptr<command>
    clone() const noexcept override
    {
        return std::make_shared<read_log>(*this);
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
        auto [pack, err] = input_msg<header, request_fields_log, std::uint8_t>(slave(), message);
        if (error(err) != exception::no_error) [[unlikely]]
            return err;
        // GCOVR_EXCL_START
        if (pack.size > modbus_base::max_read_registers) [[unlikely]]
            return exception::illegal_data_value;
        // GCOVR_EXCL_STOP
        std::copy(pack.data, pack.data + pack.size, values_.begin());
        callback_(exception::no_error, pack.fields->address.get(), values_.begin(), values_.begin() + pack.size);
        return exception::no_error;
    }

    /**
     * @brief Destructor
     *
     */
    ~read_log() noexcept override = default;

private:
    std::size_t               size_;
    types::callback_logs_type callback_;
    msg_type                  msg_output_{};
    std::array<std::uint8_t, modbus_base::max_read_log_bytes> values_{};
};

}    // namespace xitren::modbus::commands
