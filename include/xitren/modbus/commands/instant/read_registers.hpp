/*!
     _ _
__ _(_) |_ _ _ ___ _ _
\ \ / |  _| '_/ -_) ' \
/_\_\_|\__|_| \___|_||_|
* @date 15.02.2024
*/
#pragma once

#include "../command.hpp"

namespace xitren::modbus::commands::instant {

template <std::uint8_t Slave, std::uint16_t Address, std::size_t Size,
          std::invocable<exception, std::uint16_t*, std::uint16_t*> auto Callback>
class read_registers : public command {

public:
    static constexpr auto output_command = packet<header, request_fields_read, crc16ansi>::serialize(
        header{Slave, static_cast<uint8_t>(function::read_holding_registers)}, request_fields_read{Address, Size});

    consteval read_registers() noexcept : command{Slave, Address}
    {
        std::uint32_t const max_address = Address + Size - 1;
        if (max_address > static_cast<std::uint32_t>(std::numeric_limits<std::uint16_t>::max())) [[unlikely]] {
            error(exception::illegal_data_address);
            return;
        }
    }

    inline iterator
    begin() noexcept override
    {
        return const_cast<iterator>(output_command.begin());
    }

    inline constexpr const_iterator
    begin() const noexcept override
    {
        return output_command.begin();
    }

    inline iterator
    end() noexcept override
    {
        return const_cast<iterator>(output_command.end());
    }

    inline constexpr const_iterator
    end() const noexcept override
    {
        return output_command.end();
    }

    inline std::size_t
    size() noexcept override
    {
        return output_command.size();
    }

    inline constexpr std::size_t
    size() const noexcept override
    {
        return output_command.size();
    }

    void
    no_answer() noexcept override
    {
        Callback(error(exception::bad_slave), nullptr, nullptr);
    }

    command*
    clone(command_vault_type& vault) const noexcept override
    {
        static_assert(command::command_buffer_max >= sizeof(read_registers),
                      "Command realization size exceeded storage area!");
        return new (&vault) read_registers(*this);
    }

    std::shared_ptr<command>
    clone() const noexcept override
    {
        return std::make_shared<read_registers>(*this);
    }

    exception
    receive(msg_type const& message) noexcept override
    {
        static std::array<std::uint16_t, modbus_base::max_read_registers> values{};
        auto [pack, err] = input_msg<header, std::uint8_t, func::msb_t<std::uint16_t>>(slave(), message);
        if (error(err) != exception::no_error) [[unlikely]] {
            Callback(err, nullptr, nullptr);
            return err;
        }
        if (pack.size > modbus_base::max_read_registers) [[unlikely]] {
            Callback(err, nullptr, nullptr);
            return exception::illegal_data_value;
        }
        for (std::size_t i{}; (i < values.size()) && (i < pack.size); i++) {
            values[i] = pack.data[i].get();
        }
        Callback(exception::no_error, values.begin(), values.begin() + pack.size);
        return exception::no_error;
    }

    ~read_registers() noexcept override = default;
};

}    // namespace xitren::modbus::commands::instant
