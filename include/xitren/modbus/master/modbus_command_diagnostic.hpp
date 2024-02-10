#include <xitren/modbus/master/modbus_command_regs_adv.hpp>

namespace xitren::modbus {

template <std::uint8_t Slave, diagnostics_sub_function Sub, std::invocable<exception, std::uint16_t> auto Callback>
class read_diagnostics_cnt_static : public modbus_command {

public:
    static constexpr auto output_command = packet<header, func::msb_t<std::uint16_t>, crc16ansi>::serialize(
        header{Slave, static_cast<uint8_t>(function::diagnostic)}, static_cast<uint16_t>(Sub));

    consteval read_diagnostics_cnt_static() noexcept : modbus_command{Slave, static_cast<uint16_t>(Sub)}
    {
        if ((Sub != diagnostics_sub_function::return_bus_message_count)
            && (Sub != diagnostics_sub_function::return_bus_comm_error_count)
            && (Sub != diagnostics_sub_function::return_server_exception_error_count)
            && (Sub != diagnostics_sub_function::return_server_message_count)
            && (Sub != diagnostics_sub_function::return_server_no_response_count)
            && (Sub != diagnostics_sub_function::return_server_nak_count)
            && (Sub != diagnostics_sub_function::return_server_busy_count)
            && (Sub != diagnostics_sub_function::return_bus_char_overrun_count)) [[unlikely]] {
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
        Callback(error(exception::bad_slave), 0);
    }

    modbus_command*
    clone(command_vault_type& vault) const noexcept override
    {
        static_assert(modbus_command::command_buffer_max >= sizeof(read_diagnostics_cnt_static),
                      "Command realization size exceeded storage area!");
        return new (&vault) read_diagnostics_cnt_static(*this);
    }

    std::shared_ptr<modbus_command>
    clone() const noexcept override
    {
        return std::make_shared<read_diagnostics_cnt_static>(*this);
    }

    exception
    receive(msg_type const& message) noexcept override
    {
        static std::uint16_t values{};
        auto [pack, err] = input_msg<header, func::msb_t<std::uint16_t>, msb_t<std::uint16_t>>(slave(), message);
        if (error(err) != exception::no_error) [[unlikely]] {
            Callback(err, 0);
            return err;
        }
        if (pack.size > modbus_base::max_read_registers) [[unlikely]] {
            Callback(err, 0);
            return exception::illegal_data_value;
        }
        values = pack.data[0].get();
        Callback(exception::no_error, values);
        return exception::no_error;
    }

    ~read_diagnostics_cnt_static() noexcept override = default;
};

class read_diagnostics_cnt : public modbus_command {

public:
    read_diagnostics_cnt(std::uint8_t slave, diagnostics_sub_function sub, types::callback_regs_type callback) noexcept
        : modbus_command{slave, static_cast<uint16_t>(sub)}, callback_{std::move(callback)}
    {
        if ((sub != diagnostics_sub_function::return_bus_message_count)
            && (sub != diagnostics_sub_function::return_bus_comm_error_count)
            && (sub != diagnostics_sub_function::return_server_exception_error_count)
            && (sub != diagnostics_sub_function::return_server_message_count)
            && (sub != diagnostics_sub_function::return_server_no_response_count)
            && (sub != diagnostics_sub_function::return_server_nak_count)
            && (sub != diagnostics_sub_function::return_server_busy_count)
            && (sub != diagnostics_sub_function::return_bus_char_overrun_count)) [[unlikely]] {
            error(exception::illegal_data_address);
            return;
        }
        if (!msg_output_.template serialize<header, func::msb_t<std::uint16_t>, msb_t<std::uint16_t>, crc16ansi>(
                {{slave, static_cast<uint8_t>(function::diagnostic)}, static_cast<uint16_t>(sub), 0, nullptr})) {
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

    void
    no_answer() noexcept override
    {
        callback_(error(exception::bad_slave), nullptr, nullptr);
    }

    modbus_command*
    clone(command_vault_type& vault) const noexcept override
    {
        static_assert(modbus_command::command_buffer_max >= sizeof(read_diagnostics_cnt),
                      "Command realization size exceeded storage area!");
        return new (&vault) read_diagnostics_cnt(*this);
    }

    std::shared_ptr<modbus_command>
    clone() const noexcept override
    {
        return std::make_shared<read_diagnostics_cnt>(*this);
    }

    exception
    receive(msg_type const& message) noexcept override
    {
        static std::array<std::uint16_t, modbus_base::max_read_registers> values{};
        auto [pack, err] = input_msg<header, func::msb_t<std::uint16_t>, msb_t<std::uint16_t>>(slave(), message);
        if (error(err) != exception::no_error) [[unlikely]] {
            callback_(err, nullptr, nullptr);
            return err;
        }
        if (pack.size > modbus_base::max_read_registers) [[unlikely]] {
            callback_(err, nullptr, nullptr);
            return exception::illegal_data_value;
        }
        for (std::size_t i{}; (i < values.size()) && (i < pack.size); i++) {
            values[i] = pack.data[i].get();
        }
        callback_(exception::no_error, values.begin(), values.begin() + pack.size);
        return exception::no_error;
    }

    ~read_diagnostics_cnt() noexcept override = default;

private:
    types::callback_regs_type callback_;
    msg_type                  msg_output_{};
};

}    // namespace xitren::modbus
