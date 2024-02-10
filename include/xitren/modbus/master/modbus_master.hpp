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

    template <diagnostics_sub_function Sub>
    struct diagnostic_module;
    friend diagnostic_module<diagnostics_sub_function::return_query_data>;
    friend diagnostic_module<diagnostics_sub_function::restart_comm_option>;
    friend diagnostic_module<diagnostics_sub_function::return_diagnostic_register>;
    friend diagnostic_module<diagnostics_sub_function::force_listen_only_mode>;
    friend diagnostic_module<diagnostics_sub_function::clear_counters>;
    friend diagnostic_module<diagnostics_sub_function::return_bus_message_count>;

public:
    modbus_master&
    operator<<(modbus_command& in_data)
    {
        if (command_ == nullptr) {
            auto const st{in_data.msg().storage().begin()};
            auto const fn{in_data.msg().storage().begin() + in_data.msg().size()};
            std::copy(st, fn, output_msg_.storage().begin());
            output_msg_.size(in_data.msg().size());
            command_ = in_data.clone(nullptr);
            //            command_ = in_data.clone(&vault_);
            push(output_msg_);
        }
        return *this;
    }

    modbus_master&
    operator>>(modbus_command& out_data)
    {
        out_data.receive(input_msg_);
        return *this;
    }

    virtual bool
    timer_start(std::size_t microseconds)
        = 0;

    virtual bool
    timer_stop()
        = 0;

    void
    timer_expired()
    {
        switch (state_) {
        case master_state::waiting_reply:
            state_ = master_state::processing_error;
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
            } else {
                if (func::data<header>::deserialize(input_msg_.storage().begin()).slave_id != ask_.slave) {
                    state_ = master_state::waiting_reply;
                } else {
                    state_ = master_state::processing_reply;
                    if (!timer_stop()) [[unlikely]] {
                        state_ = master_state::unrecoverable_error;
                        return exception::unknown_exception;
                    }
                }
            }
            break;
        default:
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
            state_ = master_state::idle;
            break;
        default:
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
            state_ = master_state::unrecoverable_error;
            return false;
        }
        return true;
    }

    inline void
    reset() noexcept override
    {
        state_ = master_state::idle;
        error_ = exception::no_error;
    }

    template <diagnostics_sub_function Sub, std::size_t Size>
    [[deprecated("Will be deleted. You should use command instead.")]] constexpr exception
    diagnostics(std::uint8_t slave, std::array<std::uint16_t, Size>& data)
    {
        static_assert(Size < max_wr_write_registers, "Too much to write!");
        if (slave == broadcast_address) [[unlikely]] {
            return exception::bad_slave;
        }
        if constexpr ((Sub == diagnostics_sub_function::return_query_data)) {
            diagnostic_module<Sub>::request(*this, slave, data);
            return exception::no_error;
        } else if constexpr ((Sub == diagnostics_sub_function::restart_comm_option)) {
            diagnostic_module<Sub>::request(*this, slave, data[0]);
            return exception::no_error;
        } else if constexpr ((Sub == diagnostics_sub_function::return_diagnostic_register)) {
            diagnostic_module<Sub>::request(*this, slave, data[0]);
            return exception::no_error;
        } else if constexpr ((Sub == diagnostics_sub_function::force_listen_only_mode)) {
            diagnostic_module<Sub>::request(*this, slave);
            return exception::no_error;
        } else if constexpr ((Sub == diagnostics_sub_function::clear_counters)) {
            diagnostic_module<Sub>::request(*this, slave);
            return exception::no_error;
        } else if constexpr ((Sub == diagnostics_sub_function::return_bus_message_count)
                             || (Sub == diagnostics_sub_function::return_bus_comm_error_count)
                             || (Sub == diagnostics_sub_function::return_server_exception_error_count)
                             || (Sub == diagnostics_sub_function::return_server_message_count)
                             || (Sub == diagnostics_sub_function::return_server_no_response_count)
                             || (Sub == diagnostics_sub_function::return_server_nak_count)
                             || (Sub == diagnostics_sub_function::return_server_busy_count)
                             || (Sub == diagnostics_sub_function::return_bus_char_overrun_count)) {
            modbus_master::request(*this, slave, Sub, data[0]);
            return exception::no_error;
        }
        return exception::illegal_function;
    }

    template <std::size_t Size>
    [[deprecated("Will be deleted. You should use command instead.")]] exception
    read_bits(std::uint8_t slave, std::uint16_t address, std::array<bool, Size>& values)
    {
        static_assert(Size < max_read_bits, "Too much to read!");
        if (slave == broadcast_address) [[unlikely]] {
            return exception::bad_slave;
        }
        std::uint32_t const max_address = address + Size - 1;
        if (max_address > static_cast<std::uint32_t>(std::numeric_limits<std::uint16_t>::max())) [[unlikely]] {
            return exception::illegal_data_address;
        }
        ask_ = {slave, function::read_coils};
        if (!output_msg_.template serialize<header, request_fields_read, std::uint8_t, crc16ansi>(
                {{slave, static_cast<std::uint8_t>(function::read_coils)},
                 {address, static_cast<std::uint16_t>(Size)},
                 0,
                 nullptr})) {
            return exception::illegal_data_value;
        }

        if (!push(output_msg_)) {
            return exception::slave_or_server_failure;
        }
        if (!wait_prepare()) {
            return exception::slave_or_server_failure;
        }
        if (!wait_input_msg()) [[unlikely]] {
            return exception::bad_slave;
        }
        auto [pack, err] = input_msg<header, std::uint8_t, std::uint8_t>(slave);
        if ((error_ = err) != exception::no_error) [[unlikely]]
            return err;
        for (std::size_t i{}; i < values.size(); i++) {
            std::uint8_t const i_bits = i / 8;
            std::uint8_t const ii     = 1 << (i % 8);
            values[i]                 = pack.data[i_bits] & ii;
        }
        return exception::no_error;
    }

    template <std::size_t Size>
    [[deprecated("Will be deleted. You should use command instead.")]] exception
    read_input_bits(std::uint8_t slave, std::uint16_t address, std::array<bool, Size>& values)
    {
        static_assert(Size < max_read_bits, "Too much to read!");
        if (slave == broadcast_address) [[unlikely]] {
            return exception::bad_slave;
        }
        std::uint32_t const max_address = address + Size - 1;
        if (max_address > static_cast<std::uint32_t>(std::numeric_limits<std::uint16_t>::max())) [[unlikely]] {
            return exception::illegal_data_address;
        }
        ask_ = {slave, function::read_discrete_inputs};
        if (!output_msg_.template serialize<header, request_fields_read, std::uint8_t, crc16ansi>(
                {{slave, static_cast<uint8_t>(function::read_discrete_inputs)}, {address, Size}, 0, nullptr})) {
            return exception::illegal_data_value;
        }

        if (!push(output_msg_)) {
            return exception::slave_or_server_failure;
        }
        if (!wait_prepare()) {
            return exception::slave_or_server_failure;
        }
        if (!wait_input_msg()) [[unlikely]] {
            return exception::bad_slave;
        }
        auto [pack, err] = input_msg<header, std::uint8_t, std::uint8_t>(slave);
        if ((error_ = err) != exception::no_error) [[unlikely]]
            return err;
        for (std::size_t i{}; i < values.size(); i++) {
            std::uint8_t const i_bits = i / 8;
            std::uint8_t const ii     = 1 << (i % 8);
            values[i]                 = pack.data[i_bits] & ii;
        }
        return exception::no_error;
    }

    template <std::size_t Size>
    [[deprecated("Will be deleted. You should use command instead.")]] exception
    read_registers(std::uint8_t slave, std::uint16_t address, std::array<std::uint16_t, Size>& values)
    {
        static_assert(Size < max_read_registers, "Too much to read!");
        if (slave == broadcast_address) [[unlikely]] {
            return exception::bad_slave;
        }
        std::uint32_t const max_address = address + Size - 1;
        if (max_address > static_cast<std::uint32_t>(std::numeric_limits<std::uint16_t>::max())) [[unlikely]] {
            return exception::illegal_data_address;
        }
        ask_ = {slave, function::read_holding_registers};
        if (!output_msg_.template serialize<header, request_fields_read, func::msb_t<std::uint16_t>, crc16ansi>(
                {{slave, static_cast<uint8_t>(function::read_holding_registers)}, {address, Size}, 0, nullptr})) {
            return exception::illegal_data_value;
        }

        if (!push(output_msg_)) {
            return exception::slave_or_server_failure;
        }
        if (!wait_prepare()) {
            return exception::slave_or_server_failure;
        }
        if (!wait_input_msg()) [[unlikely]] {
            return exception::bad_slave;
        }
        auto [pack, err] = input_msg<header, std::uint8_t, func::msb_t<std::uint16_t>>(slave);
        if ((error_ = err) != exception::no_error) [[unlikely]]
            return err;
        for (std::size_t i{}; i < pack.size; i++) {
            values[i] = pack.data[i].get();
        }
        return exception::no_error;
    }

    template <std::size_t Size>
    [[deprecated("Will be deleted. You should use command instead.")]] exception
    read_input_registers(std::uint8_t slave, std::uint16_t address, std::array<std::uint16_t, Size>& values)
    {
        static_assert(Size < max_read_registers, "Too much to read!");
        if (slave == broadcast_address) [[unlikely]] {
            return exception::bad_slave;
        }
        std::uint32_t const max_address = address + Size - 1;
        if (max_address > static_cast<std::uint32_t>(std::numeric_limits<std::uint16_t>::max())) [[unlikely]] {
            return exception::illegal_data_address;
        }
        ask_ = {slave, function::read_input_registers};
        if (!output_msg_.template serialize<header, request_fields_read, func::msb_t<std::uint16_t>, crc16ansi>(
                {{slave, static_cast<uint8_t>(function::read_input_registers)}, {address, Size}, 0, nullptr})) {
            return exception::illegal_data_value;
        }

        if (!push(output_msg_)) {
            return exception::slave_or_server_failure;
        }
        if (!wait_prepare()) {
            return exception::slave_or_server_failure;
        }
        if (!wait_input_msg()) [[unlikely]] {
            return exception::bad_slave;
        }
        auto [pack, err] = input_msg<header, std::uint8_t, func::msb_t<std::uint16_t>>(slave);
        if ((error_ = err) != exception::no_error) [[unlikely]]
            return err;
        for (std::size_t i{}; i < pack.size; i++) {
            values[i] = pack.data[i].get();
        }
        return exception::no_error;
    }

    [[deprecated("Will be deleted. You should use command instead.")]] exception
    write_bit(std::uint8_t slave, std::uint16_t address, bool val)
    {
        if (slave == broadcast_address) [[unlikely]] {
            return exception::bad_slave;
        }
        std::uint32_t const max_address = address;
        if (max_address > static_cast<std::uint32_t>(std::numeric_limits<std::uint16_t>::max())) [[unlikely]] {
            return exception::illegal_data_address;
        }
        ask_ = {slave, function::write_single_coil};
        if (!output_msg_.template serialize<header, request_fields_read, std::uint8_t, crc16ansi>(
                {{slave, static_cast<uint8_t>(function::write_single_coil)},
                 {address, (val) ? (on_coil_value) : (off_coil_value)},
                 0,
                 nullptr})) {
            return exception::illegal_data_value;
        }
        if (!push(output_msg_)) {
            return exception::slave_or_server_failure;
        }
        if (!wait_prepare()) {
            return exception::slave_or_server_failure;
        }
        if (!wait_input_msg()) [[unlikely]] {
            return exception::bad_slave;
        }
        auto [pack, err] = input_msg<header, request_fields_read, std::uint16_t>(slave);
        if ((error_ = err) != exception::no_error) [[unlikely]]
            return err;
        return exception::no_error;
    }

    [[deprecated("Will be deleted. You should use command instead.")]] exception
    read_exception(std::uint8_t slave, std::uint8_t& val)
    {
        if (slave == broadcast_address) [[unlikely]] {
            return exception::bad_slave;
        }
        ask_ = {slave, function::write_single_coil};
        if (!output_msg_.template serialize<header, null_field, std::uint8_t, crc16ansi>(
                {{slave, static_cast<std::uint8_t>(function::read_exception_status)}, {}, 0, nullptr})) {
            return exception::illegal_data_value;
        }
        if (!push(output_msg_)) {
            return exception::slave_or_server_failure;
        }
        if (!wait_prepare()) {
            return exception::slave_or_server_failure;
        }
        if (!wait_input_msg()) [[unlikely]] {
            return exception::bad_slave;
        }
        auto [pack, err] = input_msg<header, std::uint8_t, std::uint8_t>(slave);
        val              = pack.fields;
        if ((error_ = err) != exception::no_error) [[unlikely]]
            return err;
        return exception::no_error;
    }

    [[deprecated("Will be deleted. You should use command instead.")]] exception
    write_register(std::uint8_t slave, std::uint16_t address, std::uint16_t val)
    {
        if (slave == broadcast_address) [[unlikely]] {
            return exception::bad_slave;
        }
        std::uint32_t const max_address = address;
        if (max_address > static_cast<std::uint32_t>(std::numeric_limits<std::uint16_t>::max())) [[unlikely]] {
            return exception::illegal_data_address;
        }
        ask_ = {slave, function::write_single_register};
        if (!output_msg_.template serialize<header, request_fields_read, std::uint8_t, crc16ansi>(
                {{slave, static_cast<uint8_t>(function::write_single_register)}, {address, val}, 0, nullptr})) {
            return exception::illegal_data_value;
        }

        if (!push(output_msg_)) {
            return exception::slave_or_server_failure;
        }
        if (!wait_prepare()) {
            return exception::slave_or_server_failure;
        }
        if (!wait_input_msg()) [[unlikely]] {
            return exception::bad_slave;
        }
        auto [pack, err] = input_msg<header, request_fields_read, std::uint16_t>(slave);
        if ((error_ = err) != exception::no_error) [[unlikely]]
            return err;
        return exception::no_error;
    }

    template <std::size_t Size>
    [[deprecated("Will be deleted. You should use command instead.")]] exception
    write_bits(std::uint8_t slave, std::uint16_t address, std::array<bool, Size> const& vals)
    {
        static_assert(Size < max_write_bits, "Too much to write!");
        if (slave == broadcast_address) [[unlikely]] {
            return exception::bad_slave;
        }
        std::uint32_t const max_address = address + Size - 1;
        if (max_address > static_cast<std::uint32_t>(std::numeric_limits<std::uint16_t>::max())) [[unlikely]] {
            return exception::illegal_data_address;
        }
        ask_ = {slave, function::write_multiple_coils};
        static std::array<std::uint8_t, max_read_bits / 8> coils_collect{};
        std::uint16_t const                                coils_collect_num{
            static_cast<std::uint16_t>((vals.size() % 8) ? (vals.size() / 8 + 1) : (vals.size() / 8))};
        for (std::uint16_t i = 0; (i < max_read_bits) && (i < coils_collect_num); i++) {
            for (std::uint16_t j = 0; (j < 8) && (static_cast<std::size_t>(i * 8 + j) < vals.size()); j++) {
                if (vals[i * 8 + j]) {
                    coils_collect[i] |= 1 << j;
                }
            }
        }
        if (!output_msg_.template serialize<header, request_fields_wr_single, std::uint8_t, crc16ansi>(
                {{slave, static_cast<std::uint8_t>(function::write_multiple_coils)},
                 {address, static_cast<std::uint16_t>(vals.size()), static_cast<std::uint8_t>(coils_collect_num)},
                 static_cast<std::uint8_t>(coils_collect_num),
                 coils_collect.data()})) {
            return exception::illegal_data_value;
        }

        if (!push(output_msg_)) {
            return exception::slave_or_server_failure;
        }
        if (!wait_prepare()) {
            return exception::slave_or_server_failure;
        }
        if (!wait_input_msg()) [[unlikely]] {
            return exception::bad_slave;
        }
        auto [pack, err] = input_msg<header, request_fields_read, std::uint16_t>(slave);
        if ((error_ = err) != exception::no_error) [[unlikely]]
            return err;
        return exception::no_error;
    }

    template <std::size_t Size>
    [[deprecated("Will be deleted. You should use command instead.")]] exception
    write_registers(std::uint8_t slave, std::uint16_t address, std::array<std::uint16_t, Size> const& vals)
    {
        static_assert(Size < max_write_registers, "Too much to write!");
        static std::array<func::msb_t<std::uint16_t>, Size> data_formatted;
        for (std::input_iterator auto it1{vals.begin()}, it2{data_formatted.begin()};
             (it1 != vals.end()) && (it2 != data_formatted.end()); it1++, it2++) {
            (*it2) = (*it1);
        }
        if (slave == broadcast_address) [[unlikely]] {
            return exception::bad_slave;
        }
        std::uint32_t const max_address = address + Size - 1;
        if (max_address > static_cast<std::uint32_t>(std::numeric_limits<std::uint16_t>::max())) [[unlikely]] {
            return exception::illegal_data_address;
        }
        ask_ = {slave, function::write_multiple_registers};
        if (!output_msg_.template serialize<header, request_fields_wr_single, func::msb_t<std::uint16_t>, crc16ansi>(
                {{slave, static_cast<std::uint8_t>(function::write_multiple_registers)},
                 {address, static_cast<std::uint16_t>(vals.size()), static_cast<std::uint8_t>(Size * 2)},
                 Size,
                 data_formatted.data()})) {
            return exception::illegal_data_value;
        }

        if (!push(output_msg_)) {
            return exception::slave_or_server_failure;
        }
        if (!wait_prepare()) {
            return exception::slave_or_server_failure;
        }
        if (!wait_input_msg()) [[unlikely]] {
            return exception::bad_slave;
        }
        auto [pack, err] = input_msg<header, request_fields_read, std::uint16_t>(slave);
        if ((error_ = err) != exception::no_error) [[unlikely]]
            return err;
        return exception::no_error;
    }

    [[deprecated("Will be deleted. You should use command instead.")]] exception
    mask_write_register(std::uint8_t slave, std::uint16_t address, std::uint16_t and_mask, std::uint16_t or_mask)
    {
        if (slave == broadcast_address) [[unlikely]] {
            return exception::bad_slave;
        }
        std::uint32_t const max_address = address;
        if (max_address > static_cast<std::uint32_t>(std::numeric_limits<std::uint16_t>::max())) [[unlikely]] {
            return exception::illegal_data_address;
        }
        ask_ = {slave, function::write_single_register};
        if (!output_msg_.template serialize<header, request_fields_wr_mask, std::uint8_t, crc16ansi>(
                {{slave, static_cast<uint8_t>(function::write_single_register)},
                 {address, and_mask, or_mask},
                 0,
                 nullptr})) {
            return exception::illegal_data_value;
        }

        if (!push(output_msg_)) {
            return exception::slave_or_server_failure;
        }
        if (!wait_prepare()) {
            return exception::slave_or_server_failure;
        }
        if (!wait_input_msg()) [[unlikely]] {
            return exception::bad_slave;
        }
        auto [pack, err] = input_msg<header, request_fields_wr_mask, std::uint8_t>(slave);
        if ((error_ = err) != exception::no_error) [[unlikely]]
            return err;
        return exception::no_error;
    }

    template <std::size_t WSize, std::size_t RSize>
    [[deprecated("Will be deleted. You should use command instead.")]] exception
    write_and_read_registers(std::uint8_t slave, std::uint16_t write_address,
                             std::array<std::uint16_t, WSize> const& write_vals, std::uint16_t read_address,
                             std::array<std::uint16_t, RSize>& read_vals)
    {
        static_assert(WSize < max_wr_write_registers, "Too much to write!");
        static_assert(RSize < max_wr_read_registers, "Too much to read!");
        static_assert((WSize * RSize) < max_wr_write_registers, "Size exceeded PDU length!");
        if (slave == broadcast_address) [[unlikely]] {
            return exception::bad_slave;
        }
        std::uint32_t const max_write_address = write_address + WSize - 1;
        std::uint32_t const max_read_address  = read_address + RSize - 1;
        if ((max_write_address > static_cast<std::uint32_t>(std::numeric_limits<std::uint16_t>::max()))
            || (max_read_address > static_cast<std::uint32_t>(std::numeric_limits<std::uint16_t>::max())))
            [[unlikely]] {
            return exception::illegal_data_address;
        }
        ask_ = {slave, function::write_and_read_registers};
        return exception::not_defined;
    }

    template <std::size_t Size>
    [[deprecated("Will be deleted. You should use command instead.")]] exception
    read_fifo_queue(std::uint8_t slave, std::uint16_t address, containers::circular_buffer<std::uint16_t, Size>& buffer)
    {
        if (slave == broadcast_address) [[unlikely]] {
            return exception::bad_slave;
        }
        std::uint32_t const max_address = address + Size - 1;
        if (max_address > static_cast<std::uint32_t>(std::numeric_limits<std::uint16_t>::max())) [[unlikely]] {
            return exception::illegal_data_address;
        }
        ask_ = {slave, function::read_fifo};
        if (!output_msg_.template serialize<header, func::msb_t<std::uint16_t>, std::uint16_t, crc16ansi>(
                {{slave, static_cast<uint8_t>(function::read_fifo)}, address, 0, nullptr})) {
            return exception::illegal_data_value;
        }

        if (!push(output_msg_)) {
            return exception::slave_or_server_failure;
        }
        if (!wait_prepare()) {
            return exception::slave_or_server_failure;
        }
        if (!wait_input_msg()) [[unlikely]] {
            return exception::bad_slave;
        }
        auto [pack, err] = input_msg<header, request_fields_fifo, func::msb_t<std::uint16_t>>(slave);
        if ((error_ = err) != exception::no_error) [[unlikely]]
            return err;
        for (std::size_t i{}; i < pack.size; i++) {
            buffer.push(pack.data[i].get());
        }
        return exception::no_error;
    }

protected:
    master_state state_{master_state::idle};

private:
    request_data                       ask_;
    std::shared_ptr<modbus_command>    command_{nullptr};
    modbus_command::command_vault_type vault_{};

    inline bool
    wait_prepare()
    {
        if (!timer_start(100)) [[unlikely]] {
            state_ = master_state::unrecoverable_error;
            return false;
        }
        return true;
    }

    inline bool
    wait_input_msg()
    {
        while ((state_ == master_state::waiting_reply) || (state_ == master_state::unrecoverable_error))
            ;
        return state_ == master_state::processing_reply;
    }

    inline exception
    received_command() noexcept
    {
        if (command_ == nullptr) [[unlikely]] {
            return exception::no_error;
        }
        (*this) >> (*(command_));
        if ((*(command_)).error() == exception::bad_slave) {
            state_ = master_state::waiting_reply;
        } else {
            state_ = master_state::processing_reply;
            if (!timer_stop()) [[unlikely]] {
                state_ = master_state::unrecoverable_error;
            }
            command_ = nullptr;
        }
        if (state_ == master_state::unrecoverable_error) {
            return exception::unknown_exception;
        }
        if (command_ == nullptr) [[unlikely]] {
            return exception::no_error;
        } else {
            return (*(command_)).error();
        }
    }

    template <typename Header, typename Fields, typename Type>
    inline std::pair<msg_type::fields_out<Header, Fields, Type>, exception>
    input_msg(std::uint8_t slave)
    {
        auto pack = input_msg_.template deserialize<Header, Fields, Type, crc16ansi>();
        if (pack.header.slave_id != slave) [[unlikely]] {
            state_ = master_state::processing_error;
            return {{}, exception::bad_slave};
        }
        if (pack.header.function_code & error_reply_mask) [[unlikely]] {
            state_ = master_state::processing_error;
            return {{}, exception::illegal_function};
        }
        if (!pack.valid) [[unlikely]] {
            state_ = master_state::processing_error;
            return {{}, exception::bad_crc};
        }
        return {pack, exception::no_error};
    }

    static inline exception
    request(modbus_master& master, std::uint8_t slave, diagnostics_sub_function sub, std::uint16_t& data)
    {
        master.ask_ = {slave, function::diagnostic};
        master.output_msg_
            .template serialize<header, func::msb_t<std::uint16_t>, func::msb_t<std::uint16_t>, crc16ansi>(
                {{slave, static_cast<uint8_t>(function::diagnostic)}, static_cast<uint16_t>(sub), 0, nullptr});

        if (!master.push(master.output_msg_)) {
            return exception::slave_or_server_failure;
        }
        if (!master.wait_prepare()) {
            return exception::slave_or_server_failure;
        }
        if (!master.wait_input_msg()) [[unlikely]] {
            return exception::bad_slave;
        }
        auto [pack, err] = master.input_msg<header, func::msb_t<std::uint16_t>, func::msb_t<std::uint16_t>>(slave);
        if ((master.error_ = err) != exception::no_error) [[unlikely]]
            return err;
        if (pack.size != 1) [[unlikely]]
            return exception::bad_data;
        data = pack.data[0].get();
        return exception::no_error;
    }
};

template <>
struct modbus_master::diagnostic_module<diagnostics_sub_function::return_query_data> {
    template <std::size_t Size>
    static inline exception
    request(modbus_master& master, std::uint8_t slave, std::array<std::uint16_t, Size>& data)
    {
        static std::array<func::msb_t<std::uint16_t>, Size> data_formatted;
        for (std::input_iterator auto it1{data.begin()}, it2{data_formatted.begin()};
             (it1 != data.end()) && (it2 != data_formatted.end()); it1++, it2++) {
            (*it2) = (*it1);
        }
        master.ask_ = {slave, function::diagnostic};
        master.output_msg_
            .template serialize<header, func::msb_t<std::uint16_t>, func::msb_t<std::uint16_t>, crc16ansi>(
                {{slave, static_cast<uint8_t>(function::diagnostic)},
                 static_cast<uint16_t>(diagnostics_sub_function::return_query_data),
                 data_formatted.size(),
                 data_formatted.data()});

        if (!master.push(master.output_msg_)) {
            return exception::slave_or_server_failure;
        }
        if (!master.wait_prepare()) {
            return exception::slave_or_server_failure;
        }
        if (!master.wait_input_msg()) [[unlikely]] {
            return exception::bad_slave;
        }
        auto [pack, err] = master.input_msg<header, func::msb_t<std::uint16_t>, func::msb_t<std::uint16_t>>(slave);
        if ((master.error_ = err) != exception::no_error) [[unlikely]]
            return err;
        if (pack.size != data.size()) [[unlikely]]
            return exception::bad_data;
        for (std::size_t i{}; i < pack.size; i++) {
            data[i] = pack.data[i].get();
        }
        return exception::no_error;
    }
};

template <>
struct modbus_master::diagnostic_module<diagnostics_sub_function::restart_comm_option> {
    static inline exception
    request(modbus_master& master, std::uint8_t slave, std::uint16_t const& data)
    {
        func::msb_t<std::uint16_t> data_formatted{data};
        master.ask_ = {slave, function::diagnostic};
        master.output_msg_
            .template serialize<header, func::msb_t<std::uint16_t>, func::msb_t<std::uint16_t>, crc16ansi>(
                {{slave, static_cast<uint8_t>(function::diagnostic)},
                 static_cast<uint16_t>(diagnostics_sub_function::restart_comm_option),
                 1,
                 &data_formatted});
        if (!master.push(master.output_msg_)) {
            return exception::slave_or_server_failure;
        }
        if (!master.wait_prepare()) {
            return exception::slave_or_server_failure;
        }
        if (!master.wait_input_msg()) [[unlikely]] {
            return exception::bad_slave;
        }
        auto [pack, err] = master.input_msg<header, func::msb_t<std::uint16_t>, func::msb_t<std::uint16_t>>(slave);
        if ((master.error_ = err) != exception::no_error) [[unlikely]]
            return err;
        if (pack.size != 1) [[unlikely]]
            return exception::bad_data;
        return exception::no_error;
    }
};

template <>
struct modbus_master::diagnostic_module<diagnostics_sub_function::return_diagnostic_register> {
    static inline exception
    request(modbus_master& master, std::uint8_t slave, std::uint16_t& data)
    {
        func::msb_t<std::uint16_t> data_formatted{data};
        master.ask_ = {slave, function::diagnostic};
        master.output_msg_
            .template serialize<header, func::msb_t<std::uint16_t>, func::msb_t<std::uint16_t>, crc16ansi>(
                {{slave, static_cast<uint8_t>(function::diagnostic)},
                 static_cast<uint16_t>(diagnostics_sub_function::return_diagnostic_register),
                 1,
                 &data_formatted});
        if (!master.push(master.output_msg_)) {
            return exception::slave_or_server_failure;
        }
        if (!master.wait_prepare()) {
            return exception::slave_or_server_failure;
        }
        if (!master.wait_input_msg()) [[unlikely]] {
            return exception::bad_slave;
        }
        auto [pack, err] = master.input_msg<header, func::msb_t<std::uint16_t>, func::msb_t<std::uint16_t>>(slave);
        if ((master.error_ = err) != exception::no_error) [[unlikely]]
            return err;
        if (pack.size != 1) [[unlikely]]
            return exception::bad_data;
        data = pack.data[0].get();
        return exception::no_error;
    }
};

template <>
struct modbus_master::diagnostic_module<diagnostics_sub_function::force_listen_only_mode> {
    static inline exception
    request(modbus_master& master, std::uint8_t slave)
    {
        master.ask_ = {slave, function::diagnostic};
        master.output_msg_.template serialize<header, func::msb_t<std::uint16_t>, std::uint16_t, crc16ansi>(
            {{slave, static_cast<uint8_t>(function::diagnostic)},
             static_cast<uint16_t>(diagnostics_sub_function::force_listen_only_mode),
             0,
             nullptr});
        if (!master.push(master.output_msg_)) {
            return exception::slave_or_server_failure;
        }
        if (!master.wait_prepare()) {
            return exception::slave_or_server_failure;
        }
        if (!master.wait_input_msg()) [[unlikely]] {
            return exception::bad_slave;
        }
        auto [pack, err] = master.input_msg<header, func::msb_t<std::uint16_t>, std::uint16_t>(slave);
        if ((master.error_ = err) != exception::no_error) [[unlikely]]
            return err;
        return exception::no_error;
    }
};

template <>
struct modbus_master::diagnostic_module<diagnostics_sub_function::clear_counters> {
    static inline exception
    request(modbus_master& master, std::uint8_t slave)
    {
        master.ask_ = {slave, function::diagnostic};
        master.output_msg_.template serialize<header, func::msb_t<std::uint16_t>, std::uint16_t, crc16ansi>(
            {{slave, static_cast<uint8_t>(function::diagnostic)},
             static_cast<uint16_t>(diagnostics_sub_function::clear_counters),
             0,
             nullptr});
        if (!master.push(master.output_msg_)) {
            return exception::slave_or_server_failure;
        }
        if (!master.wait_prepare()) {
            return exception::slave_or_server_failure;
        }
        if (!master.wait_input_msg()) [[unlikely]] {
            return exception::bad_slave;
        }
        auto [pack, err] = master.input_msg<header, func::msb_t<std::uint16_t>, std::uint16_t>(slave);
        if ((master.error_ = err) != exception::no_error) [[unlikely]]
            return err;
        return exception::no_error;
    }
};

}    // namespace xitren::modbus
