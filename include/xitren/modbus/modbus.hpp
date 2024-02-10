#pragma once
#include <xitren/modbus/log/embedded.hpp>
#include <xitren/modbus/packet.hpp>

#include <limits>
#include <type_traits>

namespace xitren::modbus {

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
    read_log                 = 0x41,
    set_max_log_level        = 0x42,
    get_current_log_level    = 0x43,

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

enum class read_device_id_code : std::uint8_t {
    basic_identity_stream    = 0x01,
    regular_identity_stream  = 0x02,
    extended_identity_stream = 0x03,
    individual_access        = 0x04
};

enum class identification_id : std::uint8_t {
    basic_identity_stream    = 0x01,
    regular_identity_stream  = 0x02,
    extended_identity_stream = 0x03,
    individual_access        = 0x04
};

enum class object_id_code : std::uint8_t {
    vendor_name          = 0x00,
    product_code         = 0x01,
    major_minor_revision = 0x02,
    max                  = 0x03
};

enum class conformity_code : std::uint8_t {
    basic_identification        = 0x01,
    regular_identification      = 0x02,
    extended_identification     = 0x03,
    basic_identification_ind    = 0x81,
    regular_identification_ind  = 0x82,
    extended_identification_ind = 0x83
};

struct __attribute__((__packed__)) null_field {};

struct __attribute__((__packed__)) header {
    std::uint8_t slave_id{};
    std::uint8_t function_code{};
};

struct __attribute__((__packed__)) request_fields_read {
    func::msb_t<std::uint16_t> starting_address{};
    func::msb_t<std::uint16_t> quantity{};
};

struct __attribute__((__packed__)) request_fields_wr_single {
    func::msb_t<std::uint16_t> starting_address{};
    func::msb_t<std::uint16_t> quantity{};
    std::uint8_t               count{};
};

struct __attribute__((__packed__)) request_fields_wr_multi {
    func::msb_t<std::uint16_t> starting_address{};
    func::msb_t<std::uint16_t> quantity{};
    std::uint8_t               count{};
};

struct __attribute__((__packed__)) request_fields_wr_mask {
    func::msb_t<std::uint16_t> starting_address{};
    func::msb_t<std::uint16_t> and_mask{};
    func::msb_t<std::uint16_t> or_mask{};
};

struct __attribute__((__packed__)) request_fields_fifo {
    func::msb_t<std::uint16_t> quantity{};
    func::msb_t<std::uint16_t> count{};
};

struct __attribute__((__packed__)) request_fields_log {
    func::msb_t<std::uint16_t> address{};
    func::msb_t<std::uint16_t> quantity{};
};

struct __attribute__((__packed__)) request_identification {
    std::uint8_t mei_type{};
    std::uint8_t read_mode{};
    std::uint8_t object_id{};
};

struct __attribute__((__packed__)) response_identification {
    std::uint8_t mei_type{};
    std::uint8_t read_mode{};
    std::uint8_t conformity{};
    std::uint8_t more_follows{};
    std::uint8_t next_object_id{};
    std::uint8_t number_of_objects{};
    std::uint8_t object_id{};
    std::uint8_t object_len{};
};

struct __attribute__((__packed__)) error_fields {
    exception exception_code{};
};

class modbus_base {
public:
    static constexpr std::uint8_t  broadcast_address      = 0;
    static constexpr std::uint8_t  max_valid_address      = 247;
    static constexpr std::uint16_t max_read_bits          = 2000;
    static constexpr std::uint16_t max_write_bits         = 1968;
    static constexpr std::uint16_t max_read_registers     = 125;
    static constexpr std::uint16_t max_read_fifo          = 31;
    static constexpr std::uint16_t max_read_log_bytes     = 250;
    static constexpr std::uint16_t max_write_registers    = 123;
    static constexpr std::uint16_t max_wr_write_registers = 121;
    static constexpr std::uint16_t max_wr_read_registers  = 125;
    static constexpr std::uint16_t max_pdu_length         = 253;
    static constexpr std::uint16_t max_adu_length         = 256;
    static constexpr std::uint16_t min_adu_length         = 3;
    static constexpr std::size_t   max_function_id        = 0x7f;
    static constexpr std::uint16_t on_coil_value          = 0xff00;
    static constexpr std::uint16_t off_coil_value         = 0x0000;
    static constexpr std::uint8_t  more_follows           = 0xff;
    static constexpr std::uint8_t  no_more_follows        = 0x00;
    static constexpr std::uint8_t  mei_type               = 0x0e;

    using request_type_read      = packet<header, request_fields_read, crc16ansi>;
    using request_type_wr_single = packet<header, request_fields_wr_single, crc16ansi>;
    using request_type_wr_mask   = packet<header, request_fields_wr_mask, crc16ansi>;
    using request_type_err       = packet<header, null_field, crc16ansi>;
    using request_type_fifo      = packet<header, func::msb_t<std::uint16_t>, crc16ansi>;
    using request_type_log       = packet<header, request_fields_log, crc16ansi>;
    using request_type_log_level = packet<header, null_field, crc16ansi>;
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
        auto const     val  = static_cast<std::uint16_t>(counter);
        constexpr auto base = static_cast<std::uint16_t>(diagnostics_sub_function::return_bus_message_count);
        constexpr auto max  = static_cast<std::uint16_t>(diagnostics_sub_function::return_bus_char_overrun_count);
        if ((base <= val) && (val <= max)) [[likely]] {
            counters_[val - base]++;
        }
    }

    inline std::uint16_t
    get_counter(std::uint16_t cnt)
    {
        auto const     val  = cnt;
        constexpr auto base = static_cast<std::uint16_t>(diagnostics_sub_function::return_bus_message_count);
        constexpr auto max  = static_cast<std::uint16_t>(diagnostics_sub_function::return_bus_char_overrun_count);
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
    reset() noexcept
        = 0;

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
    send(msg_type::array_type::iterator begin, msg_type::array_type::iterator end) noexcept
        = 0;

    template <class InputIterator>
    constexpr exception
    receive(InputIterator begin, InputIterator end) noexcept
    {
        static_assert(sizeof(*begin) == 1);
        if (!idle()) [[unlikely]] {
            increment_counter(diagnostics_sub_function::return_bus_char_overrun_count);
            increment_counter(diagnostics_sub_function::return_server_busy_count);
            ERROR() << "busy";
            return exception::slave_or_server_busy;
        }
        if ((end - begin) < min_adu_length) [[unlikely]] {
            increment_counter(diagnostics_sub_function::return_bus_comm_error_count);
            ERROR() << "ADU < 3";
            return exception::bad_data;
        }
        if (static_cast<std::size_t>(end - begin) > max_adu_length) [[unlikely]] {
            increment_counter(diagnostics_sub_function::return_bus_comm_error_count);
            ERROR() << "ADU > MAX";
            return exception::bad_data;
        }
        auto const crc_ptr        = end - sizeof(crc16ansi::value_type);
        auto       crc            = func::data<crc16ansi::value_type>::deserialize(crc_ptr);
        auto       crc_calculated = crc16ansi::calculate(begin, crc_ptr);
        if (crc.get() != crc_calculated.get()) {
            increment_counter(diagnostics_sub_function::return_bus_comm_error_count);
            WARN() << "bad_crc";
            return exception::bad_crc;
        }
        std::copy(begin, end, input_msg_.storage().begin());
        input_msg_.size(end - begin);
        TRACE() << "recv msg";
        return received();
    }

    virtual inline bool
    idle() noexcept
        = 0;

    virtual exception
    received() noexcept
        = 0;

    virtual exception
    processing() noexcept
        = 0;

    inline msg_type&
    input() noexcept
    {
        return input_msg_;
    }

    [[nodiscard]] inline msg_type const&
    input() const noexcept
    {
        return input_msg_;
    }

    inline msg_type&
    output() noexcept
    {
        return output_msg_;
    }

    [[nodiscard]] inline msg_type const&
    output() const noexcept
    {
        return output_msg_;
    }
    virtual ~modbus_base() = default;
};

template <std::uint16_t Inputs, std::uint16_t Coils, std::uint16_t InputRegisters, std::uint16_t HoldingRegisters,
          std::uint16_t Fifo = 1>
class modbus_slave;

template <class T>
concept modbus_slave_container = requires(T a, std::size_t s) {
    {
        a.size()
    } -> std::same_as<typename T::size_type>;
    {
        a[s]
    } -> std::same_as<typename T::value_type&>;
};

template <modbus_slave_container TInputs, modbus_slave_container TCoils, modbus_slave_container TInputRegisters,
          modbus_slave_container THoldingRegisters, std::uint16_t Fifo>
class modbus_slave_base;
}    // namespace xitren::modbus
