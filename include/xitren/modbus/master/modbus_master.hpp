#pragma once
#include <xitren/modbus/master/modbus_command.hpp>
#include <xitren/modbus/modbus.hpp>

#include <optional>
#include <ranges>
#include <variant>

namespace xitren::modbus {

class modbus_master : public modbus_base {

    struct request_data {
        uint8_t  slave{};
        function code{};

        request_data&
        operator=(request_data&& other) noexcept
        {
            slave = other.slave;
            code  = other.code;
            return *this;
        }
    };

public:
    bool
    run_async(modbus_command const& in_data)
    {
        if (command_ != nullptr) {
            WARN(MODULE(modbus)) << "busy";
            return false;
        }

        auto const st{in_data.begin()};
        auto const fn{in_data.begin() + in_data.size()};
        std::copy(st, fn, output_msg_.storage().begin());
        output_msg_.size(in_data.size());
        command_ = in_data.clone(vault_);
        return push(output_msg_);
    }

    modbus_master&
    operator<<(modbus_command const& in_data)
    {
        run_async(in_data);
        return *this;
    }

    modbus_master&
    operator>>(modbus_command& out_data)
    {
        out_data.receive(input_msg_);
        return *this;
    }

    virtual bool
    timer_start()
        = 0;

    virtual bool
    timer_stop()
        = 0;

    virtual void
    wait()
    {}

    void
    timer_expired()
    {
        switch (state_) {
        case master_state::waiting_reply:
            TRACE(MODULE(modbus)) << "wait -> proc_err";
            state_ = master_state::processing_error;
            if (command_ != nullptr) {
                command_->no_answer();
                command_ = nullptr;
            }
            break;
        default:
            break;
        }
    }

    exception
    received() noexcept override
    {
        switch (state_) {
        case master_state::waiting_reply:
            if (command_ != nullptr) {
                return received_command();
            }
            TRACE(MODULE(modbus)) << "wait -> un_err";
            state_ = master_state::unrecoverable_error;
            return exception::unknown_exception;
            break;
        default:
            WARN(MODULE(modbus)) << "state undef: " << static_cast<int>(state_);
            break;
        }
        return exception::no_error;
    }

    inline bool
    idle() noexcept override
    {
        return (master_state::idle == state_) || (master_state::waiting_reply == state_);
    }

    exception
    processing() noexcept override
    {
        switch (state_) {
        case master_state::processing_reply:
        case master_state::processing_error:
            TRACE(MODULE(modbus)) << "proc -> idle";
            state_ = master_state::idle;
            break;
        case master_state::waiting_reply:
        case master_state::idle:
            break;
        default:
            WARN(MODULE(modbus)) << "state undef: " << static_cast<int>(state_);
            break;
        }
        return exception::no_error;
    }

    inline master_state
    state() noexcept
    {
        return state_;
    }

    [[nodiscard]] inline master_state
    state() const noexcept
    {
        return state_;
    }

    bool
    push(msg_type& msg)
    {
        state_ = master_state::waiting_reply;
        if (!send(msg.storage().begin(), msg.storage().begin() + msg.size())) {
            TRACE(MODULE(modbus)) << "wait -> un_err";
            state_ = master_state::unrecoverable_error;
            return false;
        }
        if (!timer_start()) {
            TRACE(MODULE(modbus)) << "wait -> un_err";
            state_ = master_state::unrecoverable_error;
            return false;
        }
        return true;
    }

    inline void
    reset() noexcept override
    {
        TRACE(MODULE(modbus)) << "-> idle";
        state_   = master_state::idle;
        error_   = exception::no_error;
        command_ = nullptr;
    }

    ~modbus_master() override = default;

protected:
    volatile master_state state_{
        master_state::
            idle};    // FIXME: See p.20 of
                      // https://wiki.yandex-team.ru/lavka/dev/robolab/programmirovanie/01-koncepcii-i-instrukcii/c-embedded-guidelines/?revision=149426615

private:
    request_data                       ask_{};
    modbus_command*                    command_{nullptr};
    modbus_command::command_vault_type vault_{};

    inline bool
    wait_input_msg()
    {
        while ((state_ == master_state::waiting_reply) || (state_ == master_state::unrecoverable_error)) {
            wait();
        }
        return state_ == master_state::processing_reply;
    }

    inline exception
    received_command() noexcept
    {
        if (command_ == nullptr) [[unlikely]] {
            return exception::no_error;
        }
        auto* cmd{command_};
        command_ = nullptr;
        (*this) >> (*(cmd));
        if (cmd->error() == exception::bad_slave) {
            state_   = master_state::waiting_reply;
            command_ = cmd;
        } else {
            state_ = master_state::processing_reply;
            if (!timer_stop()) [[unlikely]] {
                state_ = master_state::unrecoverable_error;
            }
        }
        if (state_ == master_state::unrecoverable_error) {
            return exception::unknown_exception;
        }
        return (*(cmd)).error();
    }

    template <typename Header, typename Fields, typename Type>
    inline std::pair<msg_type::fields_out_ptr<Header, Fields, Type>, exception>
    input_msg(std::uint8_t slave)
    {
        auto pack = input_msg_.template deserialize_no_check<Header, Fields, Type, crc16ansi>();
        if (pack.header->slave_id != slave) [[unlikely]] {
            state_ = master_state::processing_error;
            return {{}, exception::bad_slave};
        }
        if (pack.header->function_code & error_reply_mask) [[unlikely]] {
            state_ = master_state::processing_error;
            return {{}, exception::illegal_function};
        }
        return {pack, exception::no_error};
    }

    static inline exception
    request(modbus_master& master, std::uint8_t slave, diagnostics_sub_function sub, std::uint16_t& data)
    {
        master.ask_ = {slave, function::diagnostic};
        master.output_msg_.template serialize<header, msb_t<std::uint16_t>, msb_t<std::uint16_t>, crc16ansi>(
            {{slave, static_cast<uint8_t>(function::diagnostic)}, static_cast<uint16_t>(sub), 0, nullptr});

        if (!master.push(master.output_msg_)) {
            return exception::slave_or_server_failure;
        }
        if (!master.wait_input_msg()) [[unlikely]] {
            return exception::bad_slave;
        }
        auto [pack, err] = master.input_msg<header, msb_t<std::uint16_t>, msb_t<std::uint16_t>>(slave);
        if ((master.error_ = err) != exception::no_error) [[unlikely]]
            return err;
        if (pack.size != 1) [[unlikely]]
            return exception::bad_data;
        data = pack.data[0].get();
        return exception::no_error;
    }
};

}    // namespace xitren::modbus
