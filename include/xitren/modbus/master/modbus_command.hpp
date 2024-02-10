#pragma once

#include <xitren/circular_buffer.hpp>
#include <xitren/modbus/modbus.hpp>

#include <functional>
#include <memory>
#include <optional>
#include <ranges>
#include <utility>
#include <variant>

namespace xitren::modbus {

namespace types {
using bits_array_type         = std::array<bool, modbus_base::max_read_bits>;
using array_type              = std::array<std::uint16_t, modbus_base::max_read_registers>;
using callback_function_type  = std::function<void(exception)>;
using callback_exception_type = std::function<void(exception, std::uint8_t)>;
using callback_bits_type      = std::function<void(exception, bool*, bool*)>;
using callback_regs_type      = std::function<void(exception, std::uint16_t*, std::uint16_t*)>;
using callback_types
    = std::variant<callback_function_type, callback_exception_type, callback_bits_type, callback_regs_type>;
};    // namespace types

class modbus_command {
    static constexpr std::size_t command_buffer_max = 325;

    template <typename T, size_t Size>
    friend containers::circular_buffer<T, Size>&
    operator<<(containers::circular_buffer<T, Size>&, modbus_command const&);

    template <typename T, size_t Size>
    friend containers::circular_buffer<T, Size>&
    operator>>(containers::circular_buffer<T, Size>&, modbus_command&);

public:
    using command_vault_type    = std::aligned_storage_t<command_buffer_max, 1>;
    using command_vault_pointer = command_vault_type*;
    using msg_type              = packet_accessor<modbus_base::max_adu_length>;

    virtual ~modbus_command() noexcept = default;

    virtual std::shared_ptr<modbus_command> clone(command_vault_pointer) noexcept = 0;

    virtual exception
    receive(msg_type const&) noexcept
    {
        return error_ = exception::no_error;
    }

    inline constexpr msg_type&
    msg() noexcept
    {
        return msg_;
    }

    [[nodiscard]] inline msg_type const&
    msg() const noexcept
    {
        return msg_;
    }

    inline std::uint8_t
    slave() noexcept
    {
        return slave_;
    }

    [[nodiscard]] inline std::uint8_t
    slave() const noexcept
    {
        return slave_;
    }

    inline exception
    error() noexcept
    {
        return error_;
    }

    [[nodiscard]] inline exception
    error() const noexcept
    {
        return error_;
    }

protected:
    modbus_command(std::uint8_t slave, std::uint16_t address) noexcept : slave_{slave}, address_{address} {}

    inline std::uint16_t
    address() noexcept
    {
        return address_;
    }

    [[nodiscard]] inline std::uint16_t
    address() const noexcept
    {
        return address_;
    }

    inline exception
    error(exception err) noexcept
    {
        return error_ = err;
    }

    template <typename Header, typename Fields, typename Type>
    inline constexpr std::pair<msg_type::fields_out<Header, Fields, Type>, exception>
    input_msg(std::uint8_t slave, msg_type const& message) noexcept
    {
        auto pack = message.template deserialize<Header, Fields, Type, crc16ansi>();
        if (pack.header.slave_id != slave) [[unlikely]] {
            return {{}, exception::bad_slave};
        }
        if (!pack.valid) [[unlikely]] {
            return {{}, exception::bad_crc};
        }
        if (pack.header.function_code & error_reply_mask) [[unlikely]] {
            return {{}, exception::illegal_function};
        }
        return {pack, exception::no_error};
    }

private:
    std::uint8_t  slave_;
    std::uint16_t address_;
    msg_type      msg_{};
    exception     error_{exception::no_error};
};

template <typename T, size_t Size>
containers::circular_buffer<T, Size>&
operator<<(containers::circular_buffer<T, Size>& buffer, modbus_command const& in_data)
{
    auto const begin{in_data.msg().storage().begin()};
    for (auto i{begin}; i != (begin + in_data.msg().size()); i++) {
        buffer.push(*i);
    }
    return buffer;
}

template <typename T, size_t Size>
containers::circular_buffer<T, Size>&
operator>>(containers::circular_buffer<T, Size>& buffer, modbus_command& out_data)
{
    std::size_t i{0};
    auto        it = out_data.msg().storage().begin();
    while (!buffer.empty() && (i < out_data.msg().storage().size())) {
        auto& item = buffer.front();
        *(it++)    = item;
        i++;
        buffer.pop();
    }
    out_data.msg().size(i);
    return buffer;
}

}    // namespace xitren::modbus
