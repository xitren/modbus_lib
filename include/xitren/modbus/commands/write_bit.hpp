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
 * xitren::modbus::write_bit<10> cmd(1, 100, [](auto err) {
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
class write_bit : public command {
public:
    write_bit(std::uint8_t slave, std::uint16_t address, bool val, types::callback_function_type callback) noexcept
        : command{slave, address}, callback_{std::move(callback)}
    {
        std::uint32_t const max_address = address;
        if (max_address > static_cast<std::uint32_t>(std::numeric_limits<std::uint16_t>::max())) [[unlikely]] {
            error(exception::illegal_data_address);
            return;
        }
        value(val);
    }

    void
    value(bool val) noexcept
    {
        if (!msg_output_.template serialize<header, request_fields_read, std::uint8_t, crc16ansi>(
                {{slave_, static_cast<uint8_t>(function::write_single_coil)},
                 {address_, (val) ? (modbus_base::on_coil_value) : (modbus_base::off_coil_value)},
                 0,
                 nullptr})) {
            error(exception::illegal_data_address);
        }
    }

    void
    no_answer() noexcept override
    {
        callback_(error(exception::bad_slave));
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

    command*
    clone(command_vault_type& vault) const noexcept override
    {
        static_assert(command::command_buffer_max >= sizeof(write_bit),
                      "Command realization size exceeded storage area!");
        return new (&vault) write_bit(*this);
    }

    std::shared_ptr<command>
    clone() const noexcept override
    {
        return std::make_shared<write_bit>(*this);
    }

    exception
    receive(msg_type const& message) noexcept override
    {
        auto [pack, err] = input_msg<header, std::uint8_t, std::uint8_t>(slave(), message);
        if (error(err) != exception::no_error) [[unlikely]]
            return err;
        callback_(exception::no_error);
        return exception::no_error;
    }

    ~write_bit() noexcept override = default;

private:
    types::callback_function_type callback_;
    msg_type                      msg_output_{};
};

}    // namespace xitren::modbus::commands
