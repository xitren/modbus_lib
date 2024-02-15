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
 * @brief A class for writing a single coil.
 *
 * @par Example
 * @code{.cpp}
 * xitren::modbus::write_bits<10> cmd(1, 100, [](auto err) {
 *     if (err == xitren::modbus::exception::no_error) {
 *         std::cout << "Written coils";
 *         std::cout << std::endl;
 *     } else {
 *         std::cout << "Error: " << static_cast<int>(err) << std::endl;
 *     }
 * });
 * client.run_async(cmd);
 * @endcode
 *
 * @tparam Size The size of the coil.
 */
class write_bits : public command {
public:
    /**
     * @brief Constructs a new write bits object.
     *
     * @param slave The slave ID.
     * @param address The starting address.
     * @param vals The values to write.
     * @param callback The function to call when the response is received.
     */
    template <std::size_t Size>
    write_bits(std::uint8_t slave, std::uint16_t address, std::array<bool, Size> const& vals,
               types::callback_function_type callback) noexcept
        : command{slave, address}, callback_{std::move(callback)}
    {
        static_assert(Size < modbus_base::max_write_bits, "Too much to write!");
        std::uint32_t const max_address = address + Size - 1;
        if (max_address > static_cast<std::uint32_t>(std::numeric_limits<std::uint16_t>::max())) [[unlikely]] {
            error(exception::illegal_data_address);
            return;
        }
        value(vals);
    }

    /**
     * @brief Sets the values to write.
     *
     * @param vals The values to write.
     */
    template <std::size_t Size>
    void
    value(std::array<bool, Size> const& vals) noexcept
    {
        constexpr std::uint16_t coils_collect_num{static_cast<std::uint16_t>((Size % 8) ? (Size / 8 + 1) : (Size / 8))};
        std::array<std::uint8_t, coils_collect_num> coils_collect{};
        for (std::uint16_t i = 0; (i < modbus_base::max_read_bits) && (i < coils_collect_num); i++) {
            for (std::uint16_t j = 0; (j < 8) && (static_cast<std::size_t>(i * 8 + j) < vals.size()); j++) {
                if (vals[i * 8 + j]) {
                    coils_collect[i] |= 1 << j;
                }
            }
        }
        if (!msg_output_.template serialize<header, request_fields_wr_single, std::uint8_t, crc16ansi>(
                {{slave_, static_cast<std::uint8_t>(function::write_multiple_coils)},
                 {address_, static_cast<std::uint16_t>(vals.size()), static_cast<std::uint8_t>(coils_collect_num)},
                 static_cast<std::uint8_t>(coils_collect_num),
                 coils_collect.data()})) {
            error(exception::illegal_data_address);
        }
    }

    /**
     * @brief Returns an iterator to the beginning of the message.
     *
     * @return An iterator to the beginning of the message.
     */
    inline iterator
    begin() noexcept override
    {
        return msg_output_.storage().begin();
    }

    /**
     * @brief Returns an iterator to the beginning of the message.
     *
     * @return An iterator to the beginning of the message.
     */
    inline const_iterator
    begin() const noexcept override
    {
        return msg_output_.storage().begin();
    }

    /**
     * @brief Returns an iterator to the end of the message.
     *
     * @return An iterator to the end of the message.
     */
    inline iterator
    end() noexcept override
    {
        return msg_output_.storage().end();
    }

    /**
     * @brief Returns an iterator to the end of the message.
     *
     * @return An iterator to the end of the message.
     */
    inline const_iterator
    end() const noexcept override
    {
        return msg_output_.storage().end();
    }

    /**
     * @brief Returns the size of the message.
     *
     * @return The size of the message.
     */
    inline std::size_t
    size() noexcept override
    {
        return msg_output_.size();
    }

    /**
     * @brief Returns the size of the message.
     *
     * @return The size of the message.
     */
    inline std::size_t
    size() const noexcept override
    {
        return msg_output_.size();
    }

    /**
     * @brief Returns a reference to the message.
     *
     * @return A reference to the message.
     */
    inline msg_type&
    msg() noexcept
    {
        return msg_output_;
    }

    void
    no_answer() noexcept override
    {
        callback_(error(exception::bad_slave));
    }

    /**
     * @brief Clones the command.
     *
     * @param vault The command vault.
     * @return A pointer to the cloned command.
     */
    command*
    clone(command_vault_type& vault) const noexcept override
    {
        static_assert(command::command_buffer_max >= sizeof(write_bits),
                      "Command realization size exceeded storage area!");
        return new (&vault) write_bits(*this);
    }

    /**
     * @brief Clones the command.
     *
     * @return A shared pointer to the cloned command.
     */
    std::shared_ptr<command>
    clone() const noexcept override
    {
        return std::make_shared<write_bits>(*this);
    }

    /**
     * @brief Called when a response is received.
     *
     * @param message The received message.
     * @return The error code.
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
     * @brief Destroys the write bits object.
     */
    ~write_bits() noexcept override = default;

private:
    types::callback_function_type callback_;
    msg_type                      msg_output_{};
};

}    // namespace xitren::modbus::commands
