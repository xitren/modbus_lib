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
using bits_array_type        = std::array<bool, modbus_base::max_read_bits>;
using array_type             = std::array<std::uint16_t, modbus_base::max_read_registers>;
using callback_function_type = std::function<void(exception)>;
using callback_logs_type
    = std::function<void(exception, std::uint16_t address, std::uint8_t* begin, std::uint8_t* end)>;
using callback_identification_type = std::function<void(exception, std::uint8_t address, char* begin, char* end)>;
using callback_bits_type           = std::function<void(exception, bool*, bool*)>;
using callback_regs_type           = std::function<void(exception, std::uint16_t*, std::uint16_t*)>;
}    // namespace types

class modbus_command {
    template <typename T, size_t Size>
    friend containers::circular_buffer<T, Size>&
    operator<<(containers::circular_buffer<T, Size>&, modbus_command const&);
    template <typename T, size_t Size>
    friend containers::circular_buffer<T, Size>&
    operator>>(containers::circular_buffer<T, Size>&, modbus_command&);

protected:
    using iterator       = std::uint8_t*;
    using const_iterator = std::uint8_t const*;

public:
    static constexpr std::size_t command_buffer_max = 370;
    using command_vault_type                        = std::aligned_storage_t<command_buffer_max, 1>;
    using msg_type                                  = packet_accessor<modbus_base::max_adu_length>;

    virtual ~modbus_command() noexcept = default;

    virtual modbus_command*
    clone(command_vault_type&) const noexcept
        = 0;

    virtual std::shared_ptr<modbus_command>
    clone() const noexcept = 0;

    virtual inline iterator
    begin() noexcept
        = 0;

    virtual inline const_iterator
    begin() const noexcept
        = 0;

    virtual inline iterator
    end() noexcept
        = 0;

    virtual inline const_iterator
    end() const noexcept
        = 0;

    virtual inline std::size_t
    size() noexcept
        = 0;

    virtual inline std::size_t
    size() const noexcept
        = 0;

    virtual exception
    receive(msg_type const&) noexcept
    {
        return error_ = exception::no_error;
    }

    virtual void
    no_answer() noexcept
    {
        error_ = exception::bad_slave;
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
    constexpr modbus_command(std::uint8_t slave, std::uint16_t address) noexcept : slave_{slave}, address_{address} {}

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
    inline constexpr std::pair<msg_type::fields_out_ptr<Header, Fields, Type>, exception>
    input_msg(std::uint8_t slave, msg_type const& message) const noexcept
    {
        auto pack = message.template deserialize_no_check<Header, Fields, Type, crc16ansi>();
        if (pack.header->slave_id != slave) [[unlikely]] {
            return {{}, exception::bad_slave};
        }
        if (pack.header->function_code & error_reply_mask) [[unlikely]] {
            return {{}, exception::illegal_function};
        }
        return {pack, exception::no_error};
    }

    std::uint8_t  slave_;
    std::uint16_t address_;

private:
    exception error_{exception::no_error};
};

template <typename T, size_t Size>
containers::circular_buffer<T, Size>&
operator<<(containers::circular_buffer<T, Size>& buffer, modbus_command const& in_data)
{
    auto const begin{in_data.begin()};
    for (auto i{begin}; i != (begin + in_data.size()); i++) {
        buffer.push(*i);
    }
    return buffer;
}

template <typename T, size_t Size>
containers::circular_buffer<T, Size>&
operator>>(containers::circular_buffer<T, Size>& buffer, modbus_command& out_data)
{
    modbus_command::msg_type buff{};
    std::size_t              i{0};
    auto                     it = buff.storage().begin();
    while (!buffer.empty() && (i < buff.storage().size())) {
        auto& item = buffer.front();
        *(it++)    = item;
        i++;
        buffer.pop();
    }
    buff.size(i);
    out_data.receive(buff);
    return buffer;
}

}    // namespace xitren::modbus
