#pragma once

#include <xitren/modbus/master/modbus_command.hpp>

namespace xitren::modbus {

class read_bits : public modbus_command {
public:
    read_bits(std::uint8_t slave, std::uint16_t address, std::size_t size, types::callback_bits_type callback) noexcept
        : modbus_command{slave, address}, size_{size}, callback_{std::move(callback)}
    {
        std::uint32_t const max_address = address + size_ - 1;
        if (max_address > static_cast<std::uint32_t>(std::numeric_limits<std::uint16_t>::max())) [[unlikely]] {
            error(exception::illegal_data_address);
            return;
        }
        if (!msg_output_.template serialize<header, request_fields_read, std::uint8_t, crc16ansi>(
                {{slave, static_cast<std::uint8_t>(function::read_coils)},
                 {address, static_cast<std::uint16_t>(size_)},
                 0,
                 nullptr})) {
            error(exception::illegal_data_address);
            return;
        }
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

    void
    no_answer() noexcept override
    {
        callback_(error(exception::bad_slave), nullptr, nullptr);
    }

    modbus_command*
    clone(command_vault_type& vault) const noexcept override
    {
        static_assert(modbus_command::command_buffer_max >= sizeof(read_bits),
                      "Command realization size exceeded storage area!");
        return new (&vault) read_bits(*this);
    }

    std::shared_ptr<modbus_command>
    clone() const noexcept override
    {
        return std::make_shared<read_bits>(*this);
    }

    exception
    receive(msg_type const& message) noexcept override
    {
        static types::bits_array_type values{};
        auto [pack, err] = input_msg<header, std::uint8_t, std::uint8_t>(slave(), message);
        if (error(err) != exception::no_error) [[unlikely]]
            return err;
        if (pack.size > modbus_base::max_read_bits) [[unlikely]]
            return exception::illegal_data_value;
        std::uint8_t const i_max = pack.size * 8;
        for (std::size_t i{}; (i < values.size()) && (i < i_max); i++) {
            std::uint8_t const i_bits = i / 8;
            std::uint8_t const ii     = 1 << (i % 8);
            values[i]                 = pack.data[i_bits] & ii;
        }
        callback_(exception::no_error, values.begin(), values.begin() + i_max);
        return exception::no_error;
    }

    ~read_bits() noexcept override = default;

private:
    std::size_t               size_;
    types::callback_bits_type callback_;
    msg_type                  msg_output_{};
};

class read_input_bits : public modbus_command {
public:
    read_input_bits(std::uint8_t slave, std::uint16_t address, std::size_t size,
                    types::callback_bits_type callback) noexcept
        : modbus_command{slave, address}, size_{size}, callback_{std::move(callback)}
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

    void
    no_answer() noexcept override
    {
        callback_(error(exception::bad_slave), nullptr, nullptr);
    }

    modbus_command*
    clone(command_vault_type& vault) const noexcept override
    {
        static_assert(modbus_command::command_buffer_max >= sizeof(read_input_bits),
                      "Command realization size exceeded storage area!");
        return new (&vault) read_input_bits(*this);
    }

    std::shared_ptr<modbus_command>
    clone() const noexcept override
    {
        return std::make_shared<read_input_bits>(*this);
    }

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

    ~read_input_bits() noexcept override = default;

private:
    std::size_t               size_;
    types::callback_bits_type callback_;
    msg_type                  msg_output_{};
};

class write_bit : public modbus_command {
public:
    write_bit(std::uint8_t slave, std::uint16_t address, bool val, types::callback_function_type callback) noexcept
        : modbus_command{slave, address}, callback_{std::move(callback)}
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

    modbus_command*
    clone(command_vault_type& vault) const noexcept override
    {
        static_assert(modbus_command::command_buffer_max >= sizeof(write_bit),
                      "Command realization size exceeded storage area!");
        return new (&vault) write_bit(*this);
    }

    std::shared_ptr<modbus_command>
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

class write_bits : public modbus_command {
public:
    template <std::size_t Size>
    write_bits(std::uint8_t slave, std::uint16_t address, std::array<bool, Size> const& vals,
               types::callback_function_type callback) noexcept
        : modbus_command{slave, address}, callback_{std::move(callback)}
    {
        static_assert(Size < modbus_base::max_write_bits, "Too much to write!");
        std::uint32_t const max_address = address + Size - 1;
        if (max_address > static_cast<std::uint32_t>(std::numeric_limits<std::uint16_t>::max())) [[unlikely]] {
            error(exception::illegal_data_address);
            return;
        }
        value(vals);
    }

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

    void
    no_answer() noexcept override
    {
        callback_(error(exception::bad_slave));
    }

    modbus_command*
    clone(command_vault_type& vault) const noexcept override
    {
        static_assert(modbus_command::command_buffer_max >= sizeof(write_bits),
                      "Command realization size exceeded storage area!");
        return new (&vault) write_bits(*this);
    }

    std::shared_ptr<modbus_command>
    clone() const noexcept override
    {
        return std::make_shared<write_bits>(*this);
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

    ~write_bits() noexcept override = default;

private:
    types::callback_function_type callback_;
    msg_type                      msg_output_{};
};

}    // namespace xitren::modbus
