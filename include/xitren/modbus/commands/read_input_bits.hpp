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
 * @brief A class for reading input coils
 *
 *
 * @par Example
 * @code{.cpp}
 * xitren::modbus::read_input_bits<10> cmd(1, 100, [](auto err, auto begin, auto end) {
 *     if (err == xitren::modbus::exception::no_error) {
 *         std::cout << "Received coils: ";
 *         for (auto it = begin; it != end; ++it) {
 *             std::cout << static_cast<bool>(*it);
 *         }
 *         std::cout << std::endl;
 *     } else {
 *         std::cout << "Error: " << static_cast<int>(err) << std::endl;
 *     }
 * });
 * client.run_async(cmd);
 * @endcode
 *
 * @tparam Size The number of bits to read
 * @tparam Callback The type of the function to call when the response is received
 */
class read_input_bits : public command {
public:
    /**
     * @brief Constructs a new Read Input Bits object
     *
     * @param slave The slave ID to send the request to
     * @param address The starting address of the register to read
     * @param callback The function to call when the response is received
     */
    read_input_bits(std::uint8_t slave, std::uint16_t address, std::size_t size,
                    types::callback_bits_type callback) noexcept
        : command{slave, address}, size_{size}, callback_{std::move(callback)}
    {
        std::uint32_t const max_address = address + size_ - 1;
        if (max_address > static_cast<std::uint32_t>(std::numeric_limits<std::uint16_t>::max())) [[unlikely]] {
            error(exception::illegal_data_address);
            return;
        }
        if (!msg_output_.template serialize<header, request_fields_read, std::uint8_t, crc16ansi>(
                {{slave, static_cast<std::uint8_t>(function::read_discrete_inputs)},
                 {address, static_cast<std::uint16_t>(size_)},
                 0,
                 nullptr})) {
            error(exception::illegal_data_address);
            return;
        }
    }

    /**
     * @brief Returns an iterator to the beginning of the command data
     *
     * @return Iterator to the beginning of the command data
     */
    inline iterator
    begin() noexcept override
    {
        return msg_output_.storage().begin();
    }

    /**
     * @brief Returns an iterator to the beginning of the command data
     *
     * @return Iterator to the beginning of the command data
     */
    inline const_iterator
    begin() const noexcept override
    {
        return msg_output_.storage().begin();
    }

    /**
     * @brief Returns an iterator to the end of the command data
     *
     * @return Iterator to the end of the command data
     */
    inline iterator
    end() noexcept override
    {
        return msg_output_.storage().end();
    }

    /**
     * @brief Returns an iterator to the end of the command data
     *
     * @return Iterator to the end of the command data
     */
    inline const_iterator
    end() const noexcept override
    {
        return msg_output_.storage().end();
    }

    /**
     * @brief Returns the size of the command data
     *
     * @return Size of the command data
     */
    inline std::size_t
    size() noexcept override
    {
        return msg_output_.size();
    }

    /**
     * @brief Returns the size of the command data
     *
     * @return Size of the command data
     */
    inline std::size_t
    size() const noexcept override
    {
        return msg_output_.size();
    }

    /**
     * @brief Returns a reference to the message object containing the command data
     *
     * @return Reference to the message object containing the command data
     */
    inline msg_type&
    msg() noexcept
    {
        return msg_output_;
    }

    /**
     * @brief Called when no response is received
     */
    void
    no_answer() noexcept override
    {
        callback_(error(exception::bad_slave), nullptr, nullptr);
    }

    /**
     * @brief Clones the command object
     *
     * @param vault The command vault to use for allocation
     * @return A pointer to the cloned command object
     */
    command*
    clone(command_vault_type& vault) const noexcept override
    {
        static_assert(command::command_buffer_max >= sizeof(read_input_bits),
                      "Command realization size exceeded storage area!");
        return new (&vault) read_input_bits(*this);
    }

    /**
     * @brief Creates a shared pointer to the command object
     *
     * @return A shared pointer to the command object
     */
    std::shared_ptr<command>
    clone() const noexcept override
    {
        return std::make_shared<read_input_bits>(*this);
    }

    /**
     * @brief Called when a response is received
     *
     * @param message The Modbus message containing the response
     * @return The error code returned by the Modbus master
     */
    exception
    receive(msg_type const& message) noexcept override
    {
        static types::bits_array_type values{};
        auto [pack, err] = input_msg<header, std::uint8_t, std::uint8_t>(slave(), message);
        if (error(err) != exception::no_error) [[unlikely]]
            return err;
        if (pack.size > modbus_base::max_read_bits) [[unlikely]]
            return exception::illegal_data_value;
        std::size_t const i_max = pack.size * 8;
        for (std::size_t i{}; (i < values.size()) && (i < i_max); i++) {
            std::uint8_t const i_bits = i / 8;
            std::uint8_t const ii     = 1 << (i % 8);
            values[i]                 = pack.data[i_bits] & ii;
        }

        callback_(exception::no_error, values.begin(), values.begin() + i_max);
        return exception::no_error;
    }

    /**
     * @brief Default destructor
     */
    ~read_input_bits() noexcept override = default;

private:
    std::size_t               size_;
    types::callback_bits_type callback_;
    msg_type                  msg_output_{};
};

}    // namespace xitren::modbus::commands
