#pragma once

#include <xitren/modbus/master/modbus_command.hpp>

#include <utility>

namespace xitren::modbus {

class read_registers : public modbus_command {
public:
    read_registers(std::uint8_t slave, std::uint16_t address, std::size_t size,
                   types::callback_regs_type callback) noexcept
        : modbus_command{slave, address}, size_{size}, callback_{std::move(callback)}
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
        static_assert(modbus_command::command_buffer_max >= sizeof(read_registers),
                      "Command realization size exceeded storage area!");
        return new (&vault) read_registers(*this);
    }

    std::shared_ptr<modbus_command>
    clone() const noexcept override
    {
        return std::make_shared<read_registers>(*this);
    }

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

    ~read_registers() noexcept override = default;

private:
    std::size_t               size_;
    types::callback_regs_type callback_;
    msg_type                  msg_output_{};
};

class write_register : public modbus_command {
public:
    write_register(std::uint8_t slave, std::uint16_t address, std::uint16_t val,
                   types::callback_function_type callback) noexcept
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
    value(std::uint16_t val) noexcept
    {
        if (!msg_output_.template serialize<header, request_fields_read, std::uint8_t, crc16ansi>(
                {{slave_, static_cast<uint8_t>(function::write_single_register)}, {address_, val}, 0, nullptr})) {
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
        callback_(error(exception::bad_slave));
    }

    modbus_command*
    clone(command_vault_type& vault) const noexcept override
    {
        static_assert(modbus_command::command_buffer_max >= sizeof(write_register),
                      "Command realization size exceeded storage area!");
        return new (&vault) write_register(*this);
    }

    std::shared_ptr<modbus_command>
    clone() const noexcept override
    {
        return std::make_shared<write_register>(*this);
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

    ~write_register() noexcept override = default;

private:
    types::callback_function_type callback_;
    msg_type                      msg_output_{};
};

class read_input_registers : public modbus_command {
public:
    read_input_registers(std::uint8_t slave, std::uint16_t address, std::size_t size,
                         types::callback_regs_type callback) noexcept
        : modbus_command{slave, address}, size_{size}, callback_{std::move(callback)}
    {
        std::uint32_t const max_address = address + size_ - 1;
        if (max_address > static_cast<std::uint32_t>(std::numeric_limits<std::uint16_t>::max())) [[unlikely]] {
            error(exception::illegal_data_address);
            return;
        }
        if (!msg_output_.template serialize<header, request_fields_read, func::msb_t<std::uint16_t>, crc16ansi>(
                {{slave, static_cast<uint8_t>(function::read_input_registers)}, {address, size_}, 0, nullptr})) {
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
        static_assert(modbus_command::command_buffer_max >= sizeof(read_input_registers),
                      "Command realization size exceeded storage area!");
        return new (&vault) read_input_registers(*this);
    }

    std::shared_ptr<modbus_command>
    clone() const noexcept override
    {
        return std::make_shared<read_input_registers>(*this);
    }

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

    ~read_input_registers() noexcept override = default;

private:
    std::size_t               size_;
    types::callback_regs_type callback_;
    msg_type                  msg_output_{};
};

class write_registers : public modbus_command {
public:
    template <std::size_t Size>
    write_registers(std::uint8_t slave, std::uint16_t address, std::array<std::uint16_t, Size> const& vals) noexcept
        : write_registers{slave, address, vals, nullptr}
    {}

    template <std::size_t Size>
    write_registers(std::uint8_t slave, std::uint16_t address, std::array<std::uint16_t, Size> const& vals,
                    types::callback_function_type callback) noexcept
        : modbus_command{slave, address}, callback_{std::move(callback)}
    {
        value(vals);
    }

    template <std::size_t Size>
    void
    value(std::array<std::uint16_t, Size> const& vals) noexcept
    {
        static_assert(Size < modbus_base::max_write_registers, "Too much to write!");
        static std::array<func::msb_t<std::uint16_t>, Size> data_formatted;
        auto                                                it1{vals.begin()};
        auto                                                it2{data_formatted.begin()};
        for (; (it1 != vals.end()) && (it2 != data_formatted.end()); it1++, it2++) {
            (*it2) = (*it1);
        }
        if (!msg_output_.template serialize<header, request_fields_wr_single, func::msb_t<std::uint16_t>, crc16ansi>(
                {{slave_, static_cast<std::uint8_t>(function::write_multiple_registers)},
                 {address_, static_cast<std::uint16_t>(vals.size()), static_cast<std::uint8_t>(Size * 2)},
                 Size,
                 data_formatted.data()})) {
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
        callback_(error(exception::bad_slave));
    }

    modbus_command*
    clone(command_vault_type& vault) const noexcept override
    {
        static_assert(modbus_command::command_buffer_max >= sizeof(write_registers),
                      "Command realization size exceeded storage area!");
        return new (&vault) write_registers(*this);
    }

    std::shared_ptr<modbus_command>
    clone() const noexcept override
    {
        return std::make_shared<write_registers>(*this);
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

    ~write_registers() noexcept override = default;

private:
    types::callback_function_type callback_;
    msg_type                      msg_output_{};
};

}    // namespace xitren::modbus
