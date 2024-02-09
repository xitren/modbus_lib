#pragma once
#include <loveka/components/modbus/packet.hpp>

#include <limits>
#include <type_traits>

namespace loveka::components::modbus {

// MODBUS Application Protocol Specification V1.1b3
// http://www.modbus.orgv
constexpr std::string_view version{"1.1b3"};

enum class diagnostics_sub_function : std::uint16_t {
    return_query_data          = 0x00,
    restart_comm_option        = 0x01,
    return_diagnostic_register = 0x02,
    force_listen_only_mode     = 0x04,

    clear_counters                      = 0x0A,
    return_bus_message_count            = 0x0B,
    return_bus_comm_error_count         = 0x0C,
    return_server_exception_error_count = 0x0D,
    return_server_message_count         = 0x0E,
    return_server_no_response_count     = 0x0F,
    return_server_nak_count             = 0x10,
    return_server_busy_count            = 0x11,
    return_bus_char_overrun_count       = 0x12,
    clear_bus_char_overrun_count        = 0x14
};

enum class function : std::uint8_t {
    read_discrete_inputs = 0x02,
    read_coils           = 0x01,
    write_single_coil    = 0x05,
    write_multiple_coils = 0x0F,

    read_input_registers     = 0x04,
    read_holding_registers   = 0x03,
    write_single_register    = 0x06,
    write_multiple_registers = 0x10,
    write_and_read_registers = 0x17,
    mask_write_register      = 0x16,
    read_fifo                = 0x18,

    read_file_record  = 0x14,
    write_file_record = 0x15,

    read_exception_status      = 0x07,
    diagnostic                 = 0x08,
    get_com_event_counter      = 0x0B,
    get_com_event_log          = 0x0C,
    report_server_id           = 0x11,
    read_device_identification = 0x2B
};

constexpr std::uint8_t error_reply_mask = 0x80;

enum class exception : std::uint8_t {
    no_error         = 0x00,
    illegal_function = 0x01,
    illegal_data_address,
    illegal_data_value,
    slave_or_server_failure,
    acknowledge,
    slave_or_server_busy,
    negative_acknowledge,
    memory_parity,
    not_defined,
    gateway_path,
    gateway_target,
    bad_crc,
    bad_data,
    bad_exception,
    unknown_exception,
    missed_data,
    bad_slave,
    max
};

enum class slave_state {
    idle,
    checking_request,
    processing_action,
    formatting_reply,
    formatting_error_reply,
    unrecoverable_error
};

enum class master_state {
    idle,
    waiting_turnaround,
    waiting_reply,
    processing_reply,
    processing_error,
    unrecoverable_error
};

struct __attribute__((__packed__)) null_field {};

struct __attribute__((__packed__)) header {
    std::uint8_t slave_id{};
    std::uint8_t function_code{};
};

struct __attribute__((__packed__)) request_fields_read {
    msb_t<std::uint16_t> starting_address{};
    msb_t<std::uint16_t> quantity{};
};

struct __attribute__((__packed__)) request_fields_wr_single {
    msb_t<std::uint16_t> starting_address{};
    msb_t<std::uint16_t> quantity{};
    std::uint8_t         count{};
};

struct __attribute__((__packed__)) request_fields_wr_mask {
    msb_t<std::uint16_t> starting_address{};
    msb_t<std::uint16_t> and_mask{};
    msb_t<std::uint16_t> or_mask{};
};

struct __attribute__((__packed__)) request_fields_fifo {
    msb_t<std::uint16_t> quantity{};
    msb_t<std::uint16_t> count{};
};

struct __attribute__((__packed__)) error_fields {
    exception exception_code{};
};

class modbus_base {
public:
    constexpr static std::uint8_t  broadcast_address      = 0;
    constexpr static std::uint16_t max_read_bits          = 2000;
    constexpr static std::uint16_t max_write_bits         = 1968;
    constexpr static std::uint16_t max_read_registers     = 125;
    constexpr static std::uint16_t max_read_fifo          = 31;
    constexpr static std::uint16_t max_write_registers    = 123;
    constexpr static std::uint16_t max_wr_write_registers = 121;
    constexpr static std::uint16_t max_wr_read_registers  = 125;
    constexpr static std::uint16_t max_pdu_length         = 253;
    constexpr static std::uint16_t max_adu_length         = 256;
    constexpr static std::uint16_t min_adu_length         = 3;
    constexpr static std::size_t   max_function_id        = 0x7f;
    constexpr static std::uint16_t on_coil_value          = 0xff00;
    constexpr static std::uint16_t off_coil_value         = 0x0000;

    using request_type_read      = packet<header, request_fields_read, crc16ansi>;
    using request_type_wr_single = packet<header, request_fields_wr_single, crc16ansi>;
    using request_type_wr_mask   = packet<header, request_fields_wr_mask, crc16ansi>;
    using request_type_err       = packet<header, null_field, crc16ansi>;
    using request_type_fifo      = packet<header, msb_t<std::uint16_t>, crc16ansi>;
    using msg_type               = packet_accessor<max_adu_length>;

protected:
    exception                    error_{exception::no_error};
    msg_type                     input_msg_{};
    msg_type                     output_msg_{};
    std::uint8_t                 exception_status_{};
    std::uint16_t                diagnostic_register_{};
    std::array<std::uint16_t, 8> counters_{};

public:
    inline void
    increment_counter(diagnostics_sub_function counter)
    {
        const auto     val = static_cast<std::uint16_t>(counter);
        constexpr auto base
            = static_cast<std::uint16_t>(diagnostics_sub_function::return_bus_message_count);
        constexpr auto max
            = static_cast<std::uint16_t>(diagnostics_sub_function::return_bus_char_overrun_count);
        if ((base <= val) && (val <= max)) [[likely]] {
            counters_[val - base]++;
        }
    }

    inline std::uint16_t
    get_counter(std::uint16_t cnt)
    {
        const auto     val = cnt;
        constexpr auto base
            = static_cast<std::uint16_t>(diagnostics_sub_function::return_bus_message_count);
        constexpr auto max
            = static_cast<std::uint16_t>(diagnostics_sub_function::return_bus_char_overrun_count);
        if ((base <= val) && (val <= max)) [[likely]] {
            return counters_[val - base];
        }
        return 0;
    }

    inline std::uint16_t
    get_counter(diagnostics_sub_function cnt)
    {
        return get_counter(static_cast<std::uint16_t>(cnt));
    }

    inline void
    clear_counters()
    {
        std::fill(counters_.begin(), counters_.end(), 0);
    }

    virtual inline void
    reset() noexcept = 0;

    inline void
    diagnostic_register(std::uint16_t val) noexcept
    {
        diagnostic_register_ = val;
    }

    [[nodiscard]] inline std::uint16_t
    diagnostic_register() const noexcept
    {
        return diagnostic_register_;
    }

    [[nodiscard]] inline std::uint8_t
    exception_status() const noexcept
    {
        return exception_status_;
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

    virtual bool
    send(msg_type::array_type::iterator begin, msg_type::array_type::iterator end) noexcept = 0;

    template <class InputIterator>
    constexpr exception
    receive(InputIterator begin, InputIterator end) noexcept
    {
        if (!idle()) [[unlikely]] {
            increment_counter(diagnostics_sub_function::return_bus_char_overrun_count);
            increment_counter(diagnostics_sub_function::return_server_busy_count);
            return exception::slave_or_server_busy;
        }
        if ((end - begin) < min_adu_length) [[unlikely]] {
            increment_counter(diagnostics_sub_function::return_bus_comm_error_count);
        }
        if (static_cast<std::size_t>(end - begin) > input_msg_.storage().max_size()) [[unlikely]] {
            std::copy(begin, begin + input_msg_.storage().max_size(), input_msg_.storage().begin());
            input_msg_.size(input_msg_.storage().max_size());
        } else [[likely]] {
            std::copy(begin, end, input_msg_.storage().begin());
            input_msg_.size(end - begin);
        }
        return received();
    }

    exception
    receive() noexcept
    {
        auto begin = input_msg_.storage().begin();
        auto end   = input_msg_.storage().end();
        return receive(begin, end);
    }

    virtual inline bool
    idle() noexcept = 0;

    virtual exception
    received() noexcept = 0;

    virtual exception
    processing() noexcept = 0;

    inline msg_type&
    input() noexcept
    {
        return input_msg_;
    }

    [[nodiscard]] inline const msg_type&
    input() const noexcept
    {
        return input_msg_;
    }

    inline msg_type&
    output() noexcept
    {
        return output_msg_;
    }

    [[nodiscard]] inline const msg_type&
    output() const noexcept
    {
        return output_msg_;
    }
};

template <std::uint8_t Id, std::uint16_t Inputs, std::uint16_t Coils, std::uint16_t InputRegisters,
          std::uint16_t HoldingRegisters, std::uint16_t Fifo = 1>
class modbus_slave;
}    // namespace xitren::components::modbus
