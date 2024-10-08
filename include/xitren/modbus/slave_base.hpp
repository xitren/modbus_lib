/*!
_ _
__ _(_) |_ _ _ ___ _ _
\ \ / |  _| '_/ -_) ' \
/_\_\_|\__|_| \___|_||_|
* @date 15.02.2024
*/
#pragma once
#include <xitren/modbus/functions/diagnostics.hpp>
#include <xitren/modbus/functions/get_current_log_level.hpp>
#include <xitren/modbus/functions/identification.hpp>
#include <xitren/modbus/functions/read_coils.hpp>
#include <xitren/modbus/functions/read_exception_status.hpp>
#include <xitren/modbus/functions/read_fifo.hpp>
#include <xitren/modbus/functions/read_holding.hpp>
#include <xitren/modbus/functions/read_input_regs.hpp>
#include <xitren/modbus/functions/read_inputs.hpp>
#include <xitren/modbus/functions/read_log.hpp>
#include <xitren/modbus/functions/set_max_log_level.hpp>
#include <xitren/modbus/functions/write_coils.hpp>
#include <xitren/modbus/functions/write_register_mask.hpp>
#include <xitren/modbus/functions/write_registers.hpp>
#include <xitren/modbus/functions/write_single_coil.hpp>
#include <xitren/modbus/functions/write_single_register.hpp>
#include <xitren/modbus/modbus.hpp>

#include <concepts>
#include <limits>
#define STRINGIFY(x) #x

namespace xitren::modbus {

template <modbus_slave_container TInputs, modbus_slave_container TCoils, modbus_slave_container TInputRegisters,
          modbus_slave_container THoldingRegisters, std::uint16_t Fifo>
class slave_base : public modbus_base {
protected:
    using inputs_type       = TInputs;
    using coils_type        = TCoils;
    using input_regs_type   = TInputRegisters;
    using holding_regs_type = THoldingRegisters;

    static_assert(std::is_same_v<typename TInputs::value_type, bool>, "Inputs value_type must be bool!");
    static_assert(std::is_same_v<typename TCoils::value_type, bool>, "Coils value_type must be bool!");
    static_assert(std::is_same_v<typename TInputRegisters::value_type, std::uint16_t>,
                  "InputRegisters value_type must be std:uint16_t!");
    static_assert(std::is_same_v<typename THoldingRegisters::value_type, std::uint16_t>,
                  "HoldingRegisters value_type must be std:uint16_t!");

    static_assert(TInputs{}.size() > 0, "Inputs must be more than 0!");
    static_assert(TCoils{}.size() > 0, "Coils must be more than 0!");
    static_assert(TInputRegisters{}.size() > 0, "InputRegisters must be more than 0!");
    static_assert(THoldingRegisters{}.size() > 0, "HoldingRegisters must be more than 0!");

public:
    using slave_type          = slave_base<TInputs, TCoils, TInputRegisters, THoldingRegisters, Fifo>;
    using error_type          = packet<header, error_fields, crc16ansi>;
    using function_type       = exception (*)(slave_base<TInputs, TCoils, TInputRegisters, THoldingRegisters, Fifo>&);
    using function_table_type = std::array<function_type, max_function_id + 1>;
    using fifo_type           = containers::circular_buffer<func::msb_t<std::uint16_t>, Fifo>;
    using log_type            = containers::circular_buffer<std::uint8_t, xitren::modbus::log::log_size>;

    constexpr explicit slave_base(std::uint8_t slave_id, inputs_type const& inputs, coils_type& coils,
                                  input_regs_type const& input_regs, holding_regs_type& holding_regs)
        : slave_id_{slave_id},
          inputs_{inputs},
          coils_{coils},
          input_registers_{input_regs},
          holding_registers_{holding_regs}
    {
        register_function(function::read_coils, &functions::read_coils);
        register_function(function::read_discrete_inputs, &functions::read_inputs);
        register_function(function::read_holding_registers, &functions::read_holding);
        register_function(function::read_input_registers, &functions::read_input_regs);
        register_function(function::write_multiple_registers, &functions::write_registers);
        register_function(function::write_single_register, &functions::write_single_register);
        register_function(function::write_multiple_coils, &functions::write_coils);
        register_function(function::write_single_coil, &functions::write_single_coil);
        register_function(function::read_log, &functions::read_log);
        register_function(function::set_max_log_level, &functions::set_max_log_level);
        register_function(function::get_current_log_level, &functions::get_current_log_level);
        register_function(function::diagnostic, &functions::diagnostics);
        register_function(function::read_device_identification, &functions::identification);
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
            TRACE() << "idle -> check";
            state_ = slave_state::checking_request;
            break;
        default:
            WARN() << "state undef: " << static_cast<std::uint8_t>(state_);
            break;
        }
        return exception::no_error;
    }

    constexpr virtual std::string_view
    vendor_name() noexcept
    {
        return "Robolavka";
    }

    constexpr virtual std::string_view
    product_code() noexcept
    {
        return "General Modbus device";
    }

    constexpr virtual std::string_view
    major_minor_revision() noexcept
    {
        return "" STRINGIFY(MAJOR_VERSION) "." STRINGIFY(MINOR_VERSION) " " __DATE__ " " STRINGIFY(
            BUILD_NUMBER) " " STRINGIFY(COMMIT_ID);
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
        static header head{};

        switch (state_) {
        case slave_state::checking_request:
            head = func::data<header>::deserialize(input_msg_.storage().begin());

            if ((head.slave_id != slave_id_) && (head.slave_id != broadcast_address)) [[likely]] {
                TRACE() << "check -> idle";
                state_ = slave_state::idle;
                input_msg_.size(0);
                TRACE() << "bad_slave";
                return exception::bad_slave;
            }

            increment_counter(diagnostics_sub_function::return_bus_message_count);

            if ((head.function_code >= max_function_id) || (defined_functions_table_[head.function_code] == nullptr))
                [[unlikely]] {
                TRACE() << "check -> err_reply";
                state_ = slave_state::formatting_error_reply;
                input_msg_.size(0);
                increment_counter(diagnostics_sub_function::return_server_exception_error_count);
                WARN() << "illegal_function";
                return error_ = exception::illegal_function;
            }
            state_ = slave_state::processing_action;
            break;
        case slave_state::processing_action:
            if (exception::no_error == (error_ = defined_functions_table_[head.function_code](*this))) [[likely]] {
                TRACE() << "proc -> reply";
                state_ = slave_state::formatting_reply;
            } else [[unlikely]] {
                increment_counter(diagnostics_sub_function::return_server_exception_error_count);
                TRACE() << "proc -> err_reply";
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
            TRACE() << "reply -> idle";
            state_ = slave_state::idle;
            break;
        case slave_state::formatting_error_reply:
            output_msg_.template serialize<header, error_fields, uint8_t, crc16ansi>(
                {{slave_id_, static_cast<uint8_t>(head.function_code | error_reply_mask)}, {error_}, 0, nullptr});
            if (!silent_) {
                if (!send(output_msg_.storage().begin(), output_msg_.storage().begin() + output_msg_.size()))
                    [[unlikely]] {
                    TRACE() << "err_reply -> un_err";
                    state_ = slave_state::unrecoverable_error;
                    ERROR() << "unknown_exception";
                    return error_ = exception::unknown_exception;
                }
            }
            output_msg_.size(0);
            input_msg_.size(0);
            error_ = exception::no_error;
            TRACE() << "err_reply -> idle";
            state_ = slave_state::idle;
            break;
        default:
            WARN() << "state undef: " << static_cast<std::uint8_t>(state_);
        case slave_state::idle:
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

    [[nodiscard]] inline constexpr inputs_type const&
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

    [[nodiscard]] inline constexpr input_regs_type const&
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
        TRACE() << "-> idle";
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

    inline log_type&
    log() noexcept
    {
        return log_;
    }

    [[nodiscard]] inline constexpr log_type const&
    log() const noexcept
    {
        return log_;
    }

    static bool
    address_valid(std::uint16_t addr, std::uint16_t cnt, std::uint16_t size)
    {
        return ((std::numeric_limits<std::uint16_t>::max() - addr) >= cnt) && ((addr + cnt) <= size);
    }

    template <std::ranges::common_range Array>
    slave_base&
    to_log(Array const& in_data)
    {
        for (auto const& item : in_data) {
            log_.push(item);
        }
        return *this;
    }

private:
    std::uint8_t const     slave_id_;
    bool                   silent_{};
    volatile slave_state   state_ = slave_state::idle;
    inputs_type const&     inputs_;
    coils_type&            coils_;
    input_regs_type const& input_registers_;
    holding_regs_type&     holding_registers_;
    function_table_type    defined_functions_table_{};
    log_type               log_{};
};

}    // namespace xitren::modbus
