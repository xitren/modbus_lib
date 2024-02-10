#pragma once

#include <xitren/modbus/modbus.hpp>
#include <xitren/modbus/slave/modbus_diagnostics_functions.hpp>
#include <xitren/modbus/slave/modbus_fifo_functions.hpp>
#include <xitren/modbus/slave/modbus_read_functions.hpp>
#include <xitren/modbus/slave/modbus_write_functions.hpp>

#include <limits>

namespace xitren::modbus {

template <std::uint8_t Id, std::uint16_t Inputs, std::uint16_t Coils, std::uint16_t InputRegisters,
          std::uint16_t HoldingRegisters, std::uint16_t Fifo>
class modbus_slave : public modbus_base {
protected:
    using inputs_type       = std::array<bool, Inputs>;
    using coils_type        = std::array<bool, Coils>;
    using input_regs_type   = std::array<std::uint16_t, InputRegisters>;
    using holding_regs_type = std::array<std::uint16_t, HoldingRegisters>;

    static_assert(Id != 0, "Broadband ID can't be a slave ID!");
    static_assert(Inputs > 0, "Inputs must be more than 0!");
    static_assert(Coils > 0, "Coils must be more than 0!");
    static_assert(InputRegisters > 0, "InputRegisters must be more than 0!");
    static_assert(HoldingRegisters > 0, "HoldingRegisters must be more than 0!");
    static_assert(Fifo > 0, "FIFO length must be more than 0!");

public:
    using slave_type          = modbus_slave<Id, Inputs, Coils, InputRegisters, HoldingRegisters, Fifo>;
    using error_type          = packet<header, error_fields, crc16ansi>;
    using function_type       = exception (*)(modbus_slave<Id, Inputs, Coils, InputRegisters, HoldingRegisters, Fifo>&);
    using function_table_type = std::array<function_type, max_function_id + 1>;
    using fifo_type           = containers::circular_buffer<func::msb_t<std::uint16_t>, Fifo>;

    constexpr explicit modbus_slave(std::uint8_t unique_id)
        : slave_id_{static_cast<std::uint8_t>((unique_id & 0xf0) | (Id & 0x0f))}
    {
        register_function(function::read_coils, &read_coils);
        register_function(function::read_discrete_inputs, &read_inputs);
        register_function(function::read_holding_registers, &read_holding);
        register_function(function::read_input_registers, &read_input_regs);
        register_function(function::write_single_coil, &write_single_coil);
        register_function(function::write_single_register, &write_single_register);
        register_function(function::mask_write_register, &write_register_mask);
        register_function(function::write_multiple_registers, &write_registers);
        register_function(function::write_multiple_coils, &write_coils);
        register_function(function::read_exception_status, &read_exception_status);
        register_function(function::read_fifo, &read_fifo);
        register_function(function::diagnostic, &diagnostics);
    }

    void
    register_function(function const id, function_type const func) noexcept
    {
        defined_functions_table_[static_cast<std::uint8_t>(id)] = func;
    }

    void
    unregister_function(function const id) noexcept
    {
        defined_functions_table_[static_cast<std::uint8_t>(id)] = nullptr;
    }

    exception
    received() noexcept override
    {
        switch (state_) {
        case slave_state::idle:
            state_ = slave_state::checking_request;
            break;
        default:
            break;
        }
        return exception::no_error;
    }

    virtual void
    changed_coil(std::size_t, bool) noexcept
    {}

    virtual void
    changed_holding(std::size_t, std::uint16_t) noexcept
    {}

    virtual void
    restart_comm() noexcept
    {}

    exception
    processing() noexcept override
    {
        auto const head = func::data<header>::deserialize(input_msg_.storage().begin());
        switch (state_) {
        case slave_state::checking_request:
            if (input_msg_.size() == 0) [[unlikely]] {
                state_ = slave_state::idle;
                return exception::bad_data;
            }
            if ((head.slave_id != slave_id_) && (head.slave_id != 0)) [[likely]] {
                state_ = slave_state::idle;
                input_msg_.size(0);
                return exception::bad_slave;
            }
            if (!crc_valid(input_msg_)) [[unlikely]] {
                state_ = slave_state::idle;
                input_msg_.size(0);
                return exception::bad_crc;
            }

            increment_counter(diagnostics_sub_function::return_server_message_count);

            if (head.function_code >= max_function_id) [[unlikely]] {
                state_ = slave_state::idle;
                input_msg_.size(0);
                return exception::illegal_function;
            }

            if (defined_functions_table_[head.function_code] == nullptr) [[unlikely]] {
                state_ = slave_state::formatting_error_reply;
                error_ = exception::illegal_function;
                return error_;
            }
            state_ = slave_state::processing_action;
            break;
        case slave_state::processing_action:
            if (exception::no_error == (error_ = defined_functions_table_[head.function_code](*this))) [[likely]] {
                state_ = slave_state::formatting_reply;
            } else [[unlikely]] {
                increment_counter(diagnostics_sub_function::return_server_exception_error_count);
                state_ = slave_state::formatting_error_reply;
            }
            if (head.slave_id == broadcast_address) [[unlikely]] {
                increment_counter(diagnostics_sub_function::return_server_no_response_count);
                state_ = slave_state::idle;
            }
            input_msg_.size(0);
            break;
        case slave_state::formatting_reply:
            if (!silent_) {
                send(output_msg_.storage().begin(), output_msg_.storage().begin() + output_msg_.size());
            }
            output_msg_.size(0);
            state_ = slave_state::idle;
            break;
        case slave_state::formatting_error_reply:
            output_msg_.template serialize<header, error_fields, uint8_t, crc16ansi>(
                {{slave_id_, static_cast<uint8_t>(head.function_code | error_reply_mask)}, {error_}, 0, nullptr});
            if (!silent_) {
                if (!send(output_msg_.storage().begin(), output_msg_.storage().begin() + output_msg_.size()))
                    [[unlikely]] {
                    state_        = slave_state::unrecoverable_error;
                    return error_ = exception::unknown_exception;
                }
            }
            output_msg_.size(0);
            input_msg_.size(0);
            error_ = exception::no_error;
            state_ = slave_state::idle;
            break;
        default:
            break;
        }
        return exception::no_error;
    }

    inline bool
    idle() noexcept override
    {
        return slave_state::idle == state_;
    }

    inline std::uint8_t
    id() noexcept
    {
        return static_cast<std::uint8_t>(slave_id_);
    }

    [[nodiscard]] inline constexpr std::uint8_t
    id() const noexcept
    {
        return static_cast<std::uint8_t>(slave_id_);
    }

    inline inputs_type&
    inputs() noexcept
    {
        return inputs_;
    }

    [[nodiscard]] inline constexpr inputs_type&
    inputs() const noexcept
    {
        return inputs_;
    }

    inline coils_type&
    coils() noexcept
    {
        return coils_;
    }

    [[nodiscard]] inline constexpr coils_type&
    coils() const noexcept
    {
        return coils_;
    }

    inline input_regs_type&
    input_registers() noexcept
    {
        return input_registers_;
    }

    [[nodiscard]] inline constexpr input_regs_type&
    input_registers() const noexcept
    {
        return input_registers_;
    }

    inline holding_regs_type&
    holding_registers() noexcept
    {
        return holding_registers_;
    }

    [[nodiscard]] inline constexpr holding_regs_type&
    holding_registers() const noexcept
    {
        return holding_registers_;
    }

    inline void
    reset() noexcept override
    {
        state_ = slave_state::idle;
        error_ = exception::no_error;
    }

    inline slave_state
    state() noexcept
    {
        return state_;
    }

    [[nodiscard]] inline constexpr slave_state
    state() const noexcept
    {
        return state_;
    }

    inline bool
    silent() noexcept
    {
        return silent_;
    }

    [[nodiscard]] inline constexpr bool
    silent() const noexcept
    {
        return silent_;
    }

    inline void
    silent(bool val) noexcept
    {
        silent_ = val;
    }

    inline fifo_type&
    fifo() noexcept
    {
        return fifo_;
    }

    [[nodiscard]] inline constexpr fifo_type&
    fifo() const noexcept
    {
        return fifo_;
    }

    static bool
    crc_valid(msg_type const& inputs)
    {
        auto crc_conv = func::data<typename crc16ansi::value_type>::deserialize(inputs.storage().data() + inputs.size()
                                                                          - sizeof(typename crc16ansi::value_type));
        typename crc16ansi::value_type crc_calc = crc16ansi::calculate(
            inputs.storage().data(), inputs.storage().data() + inputs.size() - sizeof(typename crc16ansi::value_type));
        if (crc_conv.get() == crc_calc.get()) {
            return false;
        }
        return true;
    }

    static bool
    address_valid(std::uint16_t addr, std::uint16_t cnt, std::uint16_t size)
    {
        if (((std::numeric_limits<std::uint16_t>::max() - addr) < cnt) || ((addr + cnt) > size)) {
            return false;
        }
        return true;
    }

    template <std::ranges::common_range Array>
    modbus_slave&
    operator<<(Array const& in_data)
    {
        for (std::uint16_t const& item : in_data) {
            fifo_.push(item);
        }
        return *this;
    }

private:
    std::uint8_t const  slave_id_;
    bool                silent_{};
    slave_state         state_ = slave_state::idle;
    inputs_type         inputs_{};
    coils_type          coils_{};
    input_regs_type     input_registers_{};
    holding_regs_type   holding_registers_{};
    function_table_type defined_functions_table_{};
    fifo_type           fifo_{};
};

}    // namespace xitren::modbus
