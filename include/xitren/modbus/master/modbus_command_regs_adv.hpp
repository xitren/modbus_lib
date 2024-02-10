#pragma once

#include <xitren/modbus/master/modbus_command.hpp>

#include <concepts>
#include <utility>

namespace xitren::modbus {

template <typename T>
concept adv_command_concept = requires { typename T::output_command; };

template <typename T, size_t Size, typename AdvCommands>
utils::circular_buffer<T, Size>&
operator<<(utils::circular_buffer<T, Size>& buffer, AdvCommands const& in_data)
{
    for (auto& i : AdvCommands::output_command) {
        buffer.push(*i);
    }
    return buffer;
}

template <std::uint8_t Slave, std::uint16_t Address, std::size_t Size,
          std::invocable<exception, std::uint16_t*, std::uint16_t*> auto Callback>
class read_registers_static : public modbus_command {

public:
    static constexpr auto output_command = packet<header, request_fields_read, crc16ansi>::serialize(
        header{Slave, static_cast<uint8_t>(function::read_holding_registers)}, request_fields_read{Address, Size});

    consteval read_registers_static() noexcept : modbus_command{Slave, Address}
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

    modbus_command*
    clone(command_vault_type& vault) const noexcept override
    {
        static_assert(modbus_command::command_buffer_max >= sizeof(read_registers_static),
                      "Command realization size exceeded storage area!");
        return new (&vault) read_registers_static(*this);
    }

    std::shared_ptr<modbus_command>
    clone() const noexcept override
    {
        return std::make_shared<read_registers_static>(*this);
    }

    exception
    receive(msg_type const& message) noexcept override
    {
        static std::array<std::uint16_t, modbus_base::max_read_registers> values{};
        auto [pack, err] = input_msg<header, std::uint8_t, msb_t<std::uint16_t>>(slave(), message);
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

    ~read_registers_static() noexcept override = default;
};

template <std::uint8_t Slave, std::uint16_t Address, std::size_t Size, std::array<std::uint16_t, Size> Data,
          std::invocable<exception> auto Callback>
class write_registers_static : public modbus_command {
    using struct_type = struct __attribute__((__packed__)) tag_ {
        request_fields_wr_multi                fields;
        std::array<msb_t<std::uint16_t>, Size> data;
    };

    static consteval std::array<msb_t<std::uint16_t>, Size>
    swap(std::array<std::uint16_t, Size> const& val) noexcept
    {
        std::array<msb_t<std::uint16_t>, Size> data_r;
        for (decltype(Size) i = 0; i < Size; i++) {
            data_r[i] = val[i];
        }
        return data_r;
    }

public:
    static constexpr auto output_command = packet<header, struct_type, crc16ansi>::serialize(
        header{Slave, static_cast<uint8_t>(function::write_multiple_registers)},
        struct_type{request_fields_wr_multi{Address, Size, Size * 2}, swap(Data)});

    consteval write_registers_static() noexcept : modbus_command{Slave, Address}
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
        Callback(error(exception::bad_slave));
    }

    modbus_command*
    clone(command_vault_type& vault) const noexcept override
    {
        static_assert(modbus_command::command_buffer_max >= sizeof(write_registers_static),
                      "Command realization size exceeded storage area!");
        return new (&vault) write_registers_static(*this);
    }

    std::shared_ptr<modbus_command>
    clone() const noexcept override
    {
        return std::make_shared<write_registers_static>(*this);
    }

    exception
    receive(msg_type const& message) noexcept override
    {
        auto [pack, err] = input_msg<header, std::uint8_t, std::uint8_t>(slave(), message);
        if (error(err) != exception::no_error) [[unlikely]] {
            Callback(err);
            return err;
        }
        Callback(exception::no_error);
        return exception::no_error;
    }

    ~write_registers_static() noexcept override = default;
};

}    // namespace xitren::modbus
