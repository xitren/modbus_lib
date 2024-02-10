#pragma once

#include <xitren/modbus/master/modbus_command.hpp>

#include <utility>

namespace xitren::modbus {

class read_log : public modbus_command {
public:
    read_log(std::uint8_t slave, std::uint16_t address, std::size_t size, types::callback_logs_type callback) noexcept
        : modbus_command{slave, address}, size_{size}, callback_{std::move(callback)}
    {
        std::uint32_t const max_address = address + size_ - 1;
        if (max_address > static_cast<std::uint32_t>(std::numeric_limits<std::uint16_t>::max())) [[unlikely]] {
            error(exception::illegal_data_address);
            return;
        }
        if (!msg_output_.template serialize<header, request_fields_log, msb_t<std::uint16_t>, crc16ansi>(
                {{slave, static_cast<uint8_t>(function::read_log)}, {address, size_}, 0, nullptr})) {
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
        callback_(error(exception::bad_slave), 0, nullptr, nullptr);
    }

    modbus_command*
    clone(command_vault_type& vault) const noexcept override
    {
        static_assert(modbus_command::command_buffer_max >= sizeof(read_log),
                      "Command realization size exceeded storage area!");
        return new (&vault) read_log(*this);
    }

    std::shared_ptr<modbus_command>
    clone() const noexcept override
    {
        return std::make_shared<read_log>(*this);
    }

    exception
    receive(msg_type const& message) noexcept override
    {
        static std::array<std::uint8_t, modbus_base::max_read_log_bytes> values{};
        auto [pack, err] = input_msg<header, request_fields_log, std::uint8_t>(slave(), message);
        if (error(err) != exception::no_error) [[unlikely]]
            return err;
        if (pack.size > modbus_base::max_read_registers) [[unlikely]]
            return exception::illegal_data_value;
        std::copy(pack.data, pack.data + pack.size, values.begin());
        callback_(exception::no_error, pack.fields->address.get(), values.begin(), values.begin() + pack.size);
        return exception::no_error;
    }

    ~read_log() noexcept override = default;

private:
    std::size_t               size_;
    types::callback_logs_type callback_;
    msg_type                  msg_output_{};
};

class set_max_log_lvl : public modbus_command {
public:
    set_max_log_lvl(std::uint8_t slave, int lvl, types::callback_function_type callback) noexcept
        : modbus_command{slave, 0}, lvl_{static_cast<std::uint8_t>(lvl)}, callback_{std::move(callback)}
    {
        if ((LOG_LEVEL_TRACE > lvl) || (lvl > LOG_LEVEL_CRITICAL)) [[unlikely]] {
            error(exception::illegal_data_value);
            return;
        }
        if (!msg_output_.template serialize<header, std::uint8_t, std::uint8_t, crc16ansi>(
                {{slave, static_cast<uint8_t>(function::set_max_log_level)}, lvl_, 0, nullptr})) {
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
        static_assert(modbus_command::command_buffer_max >= sizeof(set_max_log_lvl),
                      "Command realization size exceeded storage area!");
        return new (&vault) set_max_log_lvl(*this);
    }

    std::shared_ptr<modbus_command>
    clone() const noexcept override
    {
        return std::make_shared<set_max_log_lvl>(*this);
    }

    exception
    receive(msg_type const& message) noexcept override
    {
        auto [pack, err] = input_msg<header, std::uint8_t, std::uint8_t>(slave(), message);
        if (error(err) != exception::no_error) [[unlikely]]
            return err;
        if (pack.size > modbus_base::max_read_registers) [[unlikely]]
            return exception::illegal_data_value;
        callback_(exception::no_error);
        return exception::no_error;
    }

    ~set_max_log_lvl() noexcept override = default;

private:
    std::uint8_t                  lvl_;
    types::callback_function_type callback_;
    msg_type                      msg_output_{};
};

class get_log_lvl : public modbus_command {
public:
    get_log_lvl(std::uint8_t slave, types::callback_function_type callback) noexcept
        : modbus_command{slave, 0}, callback_{std::move(callback)}
    {
        if (!msg_output_.template serialize<header, std::uint8_t, std::uint8_t, crc16ansi>(
                {{slave, static_cast<uint8_t>(function::get_current_log_level)}, {}, 0, nullptr})) {
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
        static_assert(modbus_command::command_buffer_max >= sizeof(get_log_lvl),
                      "Command realization size exceeded storage area!");
        return new (&vault) get_log_lvl(*this);
    }

    std::shared_ptr<modbus_command>
    clone() const noexcept override
    {
        return std::make_shared<get_log_lvl>(*this);
    }

    exception
    receive(msg_type const& message) noexcept override
    {
        auto [pack, err] = input_msg<header, std::uint8_t, std::uint8_t>(slave(), message);
        if (error(err) != exception::no_error) [[unlikely]]
            return err;
        if (pack.size > modbus_base::max_read_registers) [[unlikely]]
            return exception::illegal_data_value;
        callback_(exception::no_error);
        return exception::no_error;
    }

    ~get_log_lvl() noexcept override = default;

private:
    types::callback_function_type callback_;
    msg_type                      msg_output_{};
};

}    // namespace xitren::modbus
