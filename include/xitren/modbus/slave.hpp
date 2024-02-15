/*!
_ _
__ _(_) |_ _ _ ___ _ _
\ \ / |  _| '_/ -_) ' \
/_\_\_|\__|_| \___|_||_|
* @date 15.02.2024
*/
#pragma once

#include <xitren/modbus/slave_base.hpp>

#include <concepts>
#include <limits>

namespace xitren::modbus {

template <modbus_slave_container TInputs, modbus_slave_container TCoils, modbus_slave_container TInputRegisters,
          modbus_slave_container THoldingRegisters, std::uint16_t Fifo>
class slave_ext : public slave_base<TInputs, TCoils, TInputRegisters, THoldingRegisters, Fifo> {
    static_assert(Fifo > 0, "FIFO length must be more than 0!");

public:
    using slave_type          = slave_base<TInputs, TCoils, TInputRegisters, THoldingRegisters, Fifo>;
    using error_type          = packet<header, error_fields, crc16ansi>;
    using function_type       = exception (*)(slave_base<TInputs, TCoils, TInputRegisters, THoldingRegisters, Fifo>&);
    using function_table_type = std::array<function_type, slave_type::max_function_id + 1>;
    using fifo_type           = containers::circular_buffer<func::msb_t<std::uint16_t>, Fifo>;
    using log_type            = containers::circular_buffer<std::uint8_t, slave_type::log_size>;

    constexpr explicit slave_ext(std::uint8_t slave_id, typename slave_type::inputs_type const& inputs,
                                 typename slave_type::coils_type&            coils,
                                 typename slave_type::input_regs_type const& input_regs,
                                 typename slave_type::holding_regs_type&     holding_regs)
        : slave_type{slave_id, inputs, coils, input_regs, holding_regs}
    {
        slave_type::register_function(function::write_single_coil, &functions::write_single_coil);
        slave_type::register_function(function::write_single_register, &functions::write_single_register);
        slave_type::register_function(function::mask_write_register, &functions::write_register_mask);
        slave_type::register_function(function::read_exception_status, &functions::read_exception_status);
        slave_type::register_function(function::read_fifo, &functions::read_fifo);
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

    template <std::ranges::common_range Array>
    slave_ext&
    operator<<(Array const& in_data)
    {
        for (std::uint16_t const& item : in_data) {
            fifo_.push(item);
        }
        return *this;
    }

private:
    fifo_type fifo_{};
};

template <std::uint16_t Inputs, std::uint16_t Coils, std::uint16_t InputRegisters, std::uint16_t HoldingRegisters,
          std::uint16_t Fifo>
class slave
    : public slave_base<std::array<bool, Inputs>, std::array<bool, Coils>, std::array<std::uint16_t, InputRegisters>,
                        std::array<std::uint16_t, HoldingRegisters>, Fifo> {
protected:
    using modbus_slave_base_type
        = slave_base<std::array<bool, Inputs>, std::array<bool, Coils>, std::array<std::uint16_t, InputRegisters>,
                     std::array<std::uint16_t, HoldingRegisters>, Fifo>;
    using inputs_type       = typename modbus_slave_base_type::inputs_type;
    using coils_type        = typename modbus_slave_base_type::coils_type;
    using input_regs_type   = typename modbus_slave_base_type::input_regs_type;
    using holding_regs_type = typename modbus_slave_base_type::holding_regs_type;

public:
    constexpr explicit slave(std::uint8_t slave_id)
        : modbus_slave_base_type::slave_base(slave_id, inputs_data_, coils_data_, input_registers_data_,
                                             holding_registers_data_)
    {}

    inline input_regs_type&
    input_registers() noexcept
    {
        return input_registers_data_;
    }

    inline inputs_type&
    inputs() noexcept
    {
        return inputs_data_;
    }

private:
    inputs_type       inputs_data_{};
    coils_type        coils_data_{};
    input_regs_type   input_registers_data_{};
    holding_regs_type holding_registers_data_{};
};

}    // namespace xitren::modbus
