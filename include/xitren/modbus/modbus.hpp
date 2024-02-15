/*!
_ _
__ _(_) |_ _ _ ___ _ _
\ \ / |  _| '_/ -_) ' \
/_\_\_|\__|_| \___|_||_|
* @date 15.02.2024
*/
#pragma once

#include <xitren/modbus/log/embedded.hpp>
#include <xitren/modbus/packet.hpp>

#include <limits>
#include <type_traits>

namespace xitren::modbus {

// MODBUS Application Protocol Specification V1.1b3
// http://www.modbus.orgv
constexpr std::string_view version{"1.1b3"};

/**
 * @brief Enum containing the different diagnostics sub-functions
 *
 * These are the different sub-functions that can be used in the diagnostic request.
 *
 * @see https://www.modbus.org/docs/Modbus_Application_Protocol_V11.pdf
 */
enum class diagnostics_sub_function : std::uint16_t {
    /**
     * @brief Return query data
     *
     * Returns the contents of the diagnostic register.
     */
    return_query_data = 0x00,
    /**
     * @brief Restart communication option
     *
     * Resets the communication options to the default values.
     */
    restart_comm_option = 0x01,
    /**
     * @brief Return diagnostic register
     *
     * Returns the contents of the diagnostic register.
     */
    return_diagnostic_register = 0x02,
    /**
     * @brief Force listen only mode
     *
     * Disables the server from responding to any requests.
     */
    force_listen_only_mode = 0x04,

    /**
     * @brief Clear counters
     *
     * Clears all of the counters in the slave.
     */
    clear_counters = 0x0A,
    /**
     * @brief Return bus message count
     *
     * Returns the number of messages received since the last time the counter was cleared.
     */
    return_bus_message_count = 0x0B,
    /**
     * @brief Return bus communication error count
     *
     * Returns the number of communication errors detected since the last time the counter was cleared.
     */
    return_bus_comm_error_count = 0x0C,
    /**
     * @brief Return server exception error count
     *
     * Returns the number of exception errors detected since the last time the counter was cleared.
     */
    return_server_exception_error_count = 0x0D,
    /**
     * @brief Return server message count
     *
     * Returns the number of messages sent since the last time the counter was cleared.
     */
    return_server_message_count = 0x0E,
    /**
     * @brief Return server no response count
     *
     * Returns the number of times the server did not respond to a request since the last time the counter was cleared.
     */
    return_server_no_response_count = 0x0F,
    /**
     * @brief Return server NAK count
     *
     * Returns the number of times the server sent a negative acknowledgement (NAK) since the last time the counter was
     * cleared.
     */
    return_server_nak_count = 0x10,
    /**
     * @brief Return server busy count
     *
     * Returns the number of times the server was busy processing a request since the last time the counter was cleared.
     */
    return_server_busy_count = 0x11,
    /**
     * @brief Return bus character overrun count
     *
     * Returns the number of times the input buffer overflowed since the last time the counter was cleared.
     */
    return_bus_char_overrun_count = 0x12,
    /**
     * @brief Clear bus character overrun count
     *
     * Clears the bus character overrun count.
     */
    clear_bus_char_overrun_count = 0x14
};

/**
 * @brief Enum containing the different functions that can be performed by a MODBUS device
 *
 * These are the different functions that can be performed by a MODBUS device. They are defined in the MODBUS
 * specification.
 *
 * @see https://www.modbus.org/docs/Modbus_Application_Protocol_V11.pdf
 */
enum class function : std::uint8_t {
    /**
     * @brief Read a set of discrete inputs
     *
     * Reads a set of discrete inputs from a device. The inputs are specified by a starting address and a quantity.
     *
     * @param starting_address The starting address of the first input to read
     * @param quantity The number of inputs to read
     * @return std::uint8_t A bit vector containing the input values
     */
    read_discrete_inputs = 0x02,

    /**
     * @brief Read a set of coils
     *
     * Reads a set of coils from a device. The coils are specified by a starting address and a quantity.
     *
     * @param starting_address The starting address of the first coil to read
     * @param quantity The number of coils to read
     * @return std::uint16_t A bit vector containing the coil values
     */
    read_coils = 0x01,

    /**
     * @brief Write a single coil
     *
     * Writes a single coil to a device. The coil is specified by a starting address and a value.
     *
     * @param starting_address The address of the coil to write
     * @param value The value to write to the coil
     * @return std::uint8_t Always returns 0x00
     */
    write_single_coil = 0x05,

    /**
     * @brief Write a set of coils
     *
     * Writes a set of coils to a device. The coils are specified by a starting address, a quantity, and a bit vector of
     * values.
     *
     * @param starting_address The starting address of the first coil to write
     * @param quantity The number of coils to write
     * @param values A bit vector containing the values to write to the coils
     * @return std::uint8_t Always returns 0x00
     */
    write_multiple_coils = 0x0F,

    /**
     * @brief Read a set of input registers
     *
     * Reads a set of input registers from a device. The registers are specified by a starting address and a quantity.
     *
     * @param starting_address The starting address of the first register to read
     * @param quantity The number of registers to read
     * @return std::uint16_t A bit vector containing the register values
     */
    read_input_registers = 0x04,

    /**
     * @brief Read a set of holding registers
     *
     * Reads a set of holding registers from a device. The registers are specified by a starting address and a quantity.
     *
     * @param starting_address The starting address of the first register to read
     * @param quantity The number of registers to read
     * @return std::uint16_t A bit vector containing the register values
     */
    read_holding_registers = 0x03,

    /**
     * @brief Write a single register
     *
     * Writes a single register to a device. The register is specified by a starting address and a value.
     *
     * @param starting_address The address of the register to write
     * @param value The value to write to the register
     * @return std::uint8_t Always returns 0x00
     */
    write_single_register = 0x06,

    /**
     * @brief Write a set of registers
     *
     * Writes a set of registers to a device. The registers are specified by a starting address, a quantity, and a bit
     * vector of values.
     *
     * @param starting_address The starting address of the first register to write
     * @param quantity The number of registers to write
     * @param values A bit vector containing the values to write to the registers
     * @return std::uint8_t Always returns 0x00
     */
    write_multiple_registers = 0x10,

    /**
     * @brief Write and read a set of registers
     *
     * Writes a set of registers to a device, then reads a set of registers from the device. The registers are specified
     * by a starting address, a quantity, and a bit vector of values.
     *
     * @param starting_address The starting address of the first register to write
     * @param quantity The number of registers to write and read
     * @param values A bit vector containing the values to write to the registers
     * @return std::uint16_t A bit vector containing the register values
     */
    write_and_read_registers = 0x17,

    /**
     * @brief Mask write a register
     *
     * Writes to a register using a mask. The register is specified by a starting address, an and mask, and an or mask.
     *
     * @param starting_address The address of the register to write
     * @param and_mask A bit vector containing the AND mask
     * @param or_mask A bit vector containing the OR mask
     * @return std::uint8_t Always returns 0x00
     */
    mask_write_register = 0x16,

    /**
     * @brief Read the FIFO queue
     *
     * Reads the contents of the FIFO queue from a device. The queue is specified by a quantity.
     *
     * @param quantity The number of registers to read from the FIFO queue
     * @return std::uint16_t A bit vector containing the register values
     */
    read_fifo = 0x18,

    /**
     * @brief Read the log
     *
     * Reads the contents of the log from a device. The log is specified by an address and a quantity.
     *
     * @param address The starting address of the log
     * @param quantity The number of registers to read from the log
     * @return std::uint16_t A bit vector containing the register values
     */
    read_log = 0x41,

    /**
     * @brief Set the maximum log level
     *
     * Sets the maximum log level for a device. The log level is specified by a value.
     *
     * @param value The maximum log level
     * @return std::uint8_t Always returns 0x00
     */
    set_max_log_level = 0x42,

    /**
     * @brief Get the current log level
     *
     * Gets the current log level for a device.
     *
     * @return std::uint8_t The current log level
     */
    get_current_log_level = 0x43,

    /**
     * @brief Read a file record
     *
     * Reads a file record from a device. The record is specified by an address.
     *
     * @param address The address of the file record
     * @return std::uint16_t A bit vector containing the register values
     */
    read_file_record = 0x14,

    /**
     * @brief Write a file record
     *
     * Writes a file record to a device. The record is specified by an address and a bit vector of values.
     *
     * @param address The address of the file record
     * @param values A bit vector containing the values to write to the file record
     * @return std::uint8_t Always returns 0x00
     */
    write_file_record = 0x15,

    /**
     * @brief Read the exception status
     *
     * Reads the exception status from a device.
     *
     * @return std::uint8_t A bit vector containing the exception status
     */
    read_exception_status = 0x07,

    /**
     * @brief Perform a diagnostic function
     *
     * Performs a diagnostic function on a device. The function is specified by a sub-function.
     *
     * @param sub_function The diagnostic sub-function to perform
     * @return std::uint16_t The value of the diagnostic register
     */
    diagnostic = 0x08,

    /**
     * @brief Get the communication event counter
     *
     * Gets the communication event counter for a device.
     *
     * @return std::uint16_t A bit vector containing the communication event counter values
     */
    get_com_event_counter = 0x0B,

    /**
     * @brief Get the communication event log
     *
     * Gets the communication event log for a device. The log is specified by an address and a quantity.
     *
     * @param address The starting address of the log
     * @param quantity The number of registers to read from the log
     * @return std::uint16_t A bit vector containing the register values
     */
    get_com_event_log = 0x0C,

    /**
     * @brief Report Server ID
     *
     * This diagnostic request returns the server ID of the device.
     *
     * @see https://www.modbus.org/docs/Modbus_Application_Protocol_V11.pdf#page-14
     */
    report_server_id = 0x11,

    /**
     * @brief Read Device Identification
     *
     * This diagnostic request returns information about the device, such as its vendor name, product code, and
     * version.
     *
     * @see https://www.modbus.org/docs/Modbus_Application_Protocol_V11.pdf#page-15
     */
    read_device_identification = 0x2B
};

constexpr std::uint8_t error_reply_mask = 0x80;

/**
 * @brief The modbus::exception enum defines the set of possible exceptions that may be returned by a MODBUS device.
 *
 * The exception codes are defined in the MODBUS Application Protocol Specification, and are used to indicate errors or
 * other conditions that occur during the processing of a MODBUS request.
 *
 * The exception codes are represented as an enum with each exception defined as a unique value. The exception codes are
 * defined as follows:
 *
 * | Name               | Code | Description                                                                         |
 * |--------------------|------|-------------------------------------------------------------------------------------|
 * | no_error           | 0x00 | No error                                                                            |
 * | illegal_function   | 0x01 | Illegal function code                                                               |
 * | illegal_data_address| 0x02 | Illegal data address                                                               |
 * | illegal_data_value | 0x03 | Illegal data value                                                                  |
 * | slave_or_server_failure | 0x04 | Slave or server failure | | acknowledge        | 0x05 | Acknowledge | |
 * slave_or_server_busy | 0x06 | Slave or server busy                                                                |
 * | negative_acknowledge | 0x07 | Negative acknowledge                                                                |
 * | memory_parity      | 0x08 | Memory parity error                                                                 |
 * | not_defined        | 0x0A | Function code not supported or implemented                                           |
 * | gateway_path       | 0x0B | Gateway path unavailable                                                            |
 * | gateway_target     | 0x0C | Gateway target device unavailable                                                   |
 * | bad_crc            | 0x0D | CRC check failed                                                                    |
 * | bad_data           | 0x0E | Data length is incorrect                                                            |
 * | bad_exception      | 0x0F | Exception occurred                                                                  |
 * | unknown_exception  | 0x10 | Unknown exception                                                                   |
 * | missed_data        | 0x11 | Data length is incorrect                                                            |
 * | bad_slave          | 0x12 | Slave address incorrect                                                             |
 * | max                | 0x13 | Maximum number of exceptions                                                        |
 *
 * The exception codes are divided into two categories: client-generated exceptions and server-generated exceptions.
 * Client-generated exceptions are generated by the MODBUS client (such as a software application) and are used to
 * indicate errors in the request. Server-generated exceptions are generated by the MODBUS server and are used to
 * indicate errors or other conditions that occur during the processing of the request.
 */
enum class exception : std::uint8_t {
    no_error         = 0x00,    ///< No error.
    illegal_function = 0x01,    ///< Illegal function code.
    illegal_data_address,       ///< Illegal data address.
    illegal_data_value,         ///< Illegal data value.
    slave_or_server_failure,    ///< Slave or server failure.
    acknowledge,                ///< Acknowledge.
    slave_or_server_busy,       ///< Slave or server busy.
    negative_acknowledge,       ///< Negative acknowledge.
    memory_parity,              ///< Memory parity error.
    not_defined,                ///< Function code not supported or implemented.
    gateway_path,               ///< Gateway path unavailable.
    gateway_target,             ///< Gateway target device unavailable.
    bad_crc,                    ///< CRC check failed.
    bad_data,                   ///< Data length is incorrect.
    bad_exception,              ///< Exception occurred.
    unknown_exception,          ///< Unknown exception.
    missed_data,                ///< Data length is incorrect.
    bad_slave,                  ///< Slave address incorrect.
    max                         ///< Maximum number of exceptions.
};

/**
 * @brief Enum containing the different slave states
 *
 * These are the different states that a MODBUS slave can be in.
 */
enum class slave_state {
    /**
     * @brief The slave is in the idle state
     *
     * In this state, the slave is waiting for a request from a master.
     */
    idle,
    /**
     * @brief The slave is in the checking request state
     *
     * In this state, the slave is checking the validity of the request received from the master.
     */
    checking_request,
    /**
     * @brief The slave is in the processing action state
     *
     * In this state, the slave is processing the action specified in the request received from the master.
     */
    processing_action,
    /**
     * @brief The slave is in the formatting reply state
     *
     * In this state, the slave is formatting the reply to be sent to the master.
     */
    formatting_reply,
    /**
     * @brief The slave is in the formatting error reply state
     *
     * In this state, the slave is formatting the error reply to be sent to the master in case of an error.
     */
    formatting_error_reply,
    /**
     * @brief The slave is in the unrecoverable error state
     *
     * In this state, the slave is in an unrecoverable error state and will not respond to any further requests.
     */
    unrecoverable_error
};

/**
 * @brief The master_state enum represents the possible states of a Modbus master.
 *
 * The possible states are:
 * - idle: The master is not currently processing a request or response.
 * - waiting_turnaround: The master is waiting for a response to a previous request before sending the next request.
 * - waiting_reply: The master is waiting for a response to a request it has sent.
 * - processing_reply: The master is processing a response to a request it has sent.
 * - processing_error: The master is processing an error response to a request it has sent.
 * - unrecoverable_error: The master is in an unrecoverable error state and cannot continue processing requests.
 */
enum class master_state {
    idle,
    waiting_turnaround,
    waiting_reply,
    processing_reply,
    processing_error,
    unrecoverable_error
};

/**
 * @brief Enum containing the different types of device identities that can be read.
 *
 * These are the different types of device identities that can be read using the Read Device Identification function.
 */
enum class read_device_id_code : std::uint8_t {
    /**
     * @brief The basic identity stream
     *
     * The basic identity stream contains the minimum set of information required to identify a device, including the
     * vendor name, product code, and version.
     */
    basic_identity_stream = 0x01,

    /**
     * @brief The regular identity stream
     *
     * The regular identity stream contains additional information about a device, including the vendor name, product
     * code, version, and a serial number.
     */
    regular_identity_stream = 0x02,

    /**
     * @brief The extended identity stream
     *
     * The extended identity stream contains even more information about a device, including the vendor name, product
     * code, version, serial number, and a lot number.
     */
    extended_identity_stream = 0x03,

    /**
     * @brief The individual access identity
     *
     * The individual access identity allows a device to provide its own unique identity, rather than using a standard
     * identity stream.
     */
    individual_access = 0x04
};

/**
 * @brief Enumerates the different types of Modbus identification
 *
 * The Modbus protocol supports several types of identification, each with different levels of information. These types
 * are defined by the `identification_id` enum.
 *
 * - `identification_id::basic_identity_stream`: This type of identification provides the minimum amount of information,
 * including the unit ID and function code.
 * - `identification_id::regular_identity_stream`: This type of identification provides more detailed information,
 * including the unit ID, function code, and the number of bytes in the message.
 * - `identification_id::extended_identity_stream`: This type of identification provides even more detailed information,
 * including the unit ID, function code, the number of bytes in the message, and the number of bytes in the preceding
 * message.
 * - `identification_id::individual_access`: This type of identification allows for individual objects to be accessed,
 * rather than a group of objects.
 *
 */
enum class identification_id : std::uint8_t {
    basic_identity_stream    = 0x01,    ///< Minimum amount of information
    regular_identity_stream  = 0x02,    ///< More detailed information
    extended_identity_stream = 0x03,    ///< Even more detailed information
    individual_access        = 0x04     ///< Access individual objects
};

/**
 * @brief Enumerates the object ID codes used in the Modbus protocol.
 *
 * These codes are used to identify the object being accessed in a request or response.
 *
 */
enum class object_id_code : std::uint8_t {
    /**
     * The vendor name object.
     */
    vendor_name = 0x00,

    /**
     * The product code object.
     */
    product_code = 0x01,

    /**
     * The major/minor revision object.
     */
    major_minor_revision = 0x02,

    /**
     * The maximum object ID code.
     */
    max = 0x03
};

/**
 * @brief An enum class representing the Modbus conformity codes.
 *
 * The Modbus protocol defines several types of conformity codes, which are used to indicate the level of identification
 * information that is provided in a request or response. These codes are represented by the `conformity_code` enum
 * class.
 *
 * The conformity codes are defined as follows:
 *
 * - `basic_identification`: Indicates that only the slave ID is present in the request or response.
 * - `regular_identification`: Indicates that the slave ID and function code are present in the request or response.
 * - `extended_identification`: Indicates that additional information, such as the object ID, is present in the request
 * or response.
 * - `basic_identification_ind`: Indicates that only the slave ID is present in the request or response, and that the
 * response contains more information.
 * - `regular_identification_ind`: Indicates that the slave ID and function code are present in the request or response,
 * and that the response contains more information.
 * - `extended_identification_ind`: Indicates that additional information, such as the object ID, is present in the
 * request or response, and that the response contains more information.
 */
enum class conformity_code : std::uint8_t {
    basic_identification        = 0x01,
    regular_identification      = 0x02,
    extended_identification     = 0x03,
    basic_identification_ind    = 0x81,
    regular_identification_ind  = 0x82,
    extended_identification_ind = 0x83
};

struct __attribute__((__packed__)) null_field {};

/*!
 * @brief The header struct contains the slave ID and function code fields of a Modbus message.
 *
 * @details The header struct is used to identify the destination device and the type of request or response being sent.
 *
 * @note The slave ID field is used to specify the device on the network that the message is intended for. The function
 * code field specifies the type of request or response, such as reading coils, writing registers, or an exception.
 */
struct __attribute__((__packed__)) header {
    std::uint8_t slave_id;      /*!< The slave ID field contains the device address on the network that the message is
                                   intended for. */
    std::uint8_t function_code; /*!< The function code field specifies the type of request or response being sent. */
};

/*!
 * @brief The request_fields_read struct contains the starting address and quantity fields of a Modbus request for
 * reading coils or input registers.
 *
 * @details The request_fields_read struct is used in requests for reading multiple coils or input registers, where the
 * starting address and quantity fields specify the starting register address and the number of registers to read.
 */
struct __attribute__((__packed__)) request_fields_read {
    /*!
     * @brief The starting_address field contains the starting register address of the coils or input registers to be
     * read.
     *
     * @details The starting_address field is a member of the request_fields_read struct and is used in requests for
     * reading multiple coils or input registers. It specifies the starting register address of the coils or input
     * registers to be read.
     */
    func::msb_t<std::uint16_t> starting_address{};
    /*!
     * @brief The quantity field contains the number of coils or input registers to be read.
     *
     * @details The quantity field is a member of the request_fields_read struct and is used in requests for reading
     * multiple coils or input registers. It specifies the number of coils or input registers to be read, starting from
     * the starting_address field.
     */
    func::msb_t<std::uint16_t> quantity{};
};

/*!
 * @brief The request_fields_wr_single struct contains the starting address, quantity, and count fields of a Modbus
 * request for writing a single register.
 *
 * @details The request_fields_wr_single struct is used in requests for writing a single register, where the starting
 * address, quantity, and count fields specify the starting register address, the number of registers to write, and the
 * value to be written.
 */
struct __attribute__((__packed__)) request_fields_wr_single {
    /*!
     * @brief The starting_address field contains the starting register address of the single register to be written.
     *
     * @details The starting_address field is a member of the request_fields_wr_single struct and is used in requests
     * for writing a single register. It specifies the starting register address of the single register to be written.
     */
    func::msb_t<std::uint16_t> starting_address{};
    /*!
     * @brief The quantity field contains the number of registers to be written.
     *
     * @details The quantity field is a member of the request_fields_wr_single struct and is used in requests for
     * writing multiple registers. It specifies the number of registers to be written, starting from the
     * starting_address field.
     */
    func::msb_t<std::uint16_t> quantity{};
    /*!
     * @brief The count field contains the value to be written to the single register.
     *
     * @details The count field is a member of the request_fields_wr_single struct and is used in requests for writing a
     * single register. It contains the value to be written to the single register specified by the starting_address
     * field.
     */
    std::uint8_t count{};
};

/*!
 * @brief The request_fields_wr_multi struct contains the starting address, quantity, and count fields of a Modbus
 * request for writing multiple registers.
 *
 * @details The request_fields_wr_multi struct is used in requests for writing multiple registers, where the starting
 * address, quantity, and count fields specify the starting register address, the number of registers to write, and the
 * value to be written.
 */
struct __attribute__((__packed__)) request_fields_wr_multi {
    /*!
     * @brief The starting_address field contains the starting register address of the first register to be written.
     *
     * @details The starting_address field is a member of the request_fields_wr_multi struct and is used in requests for
     * writing multiple registers. It specifies the starting register address of the first register to be written.
     */
    func::msb_t<std::uint16_t> starting_address{};
    /*!
     * @brief The quantity field contains the number of registers to be written.
     *
     * @details The quantity field is a member of the request_fields_wr_multi struct and is used in requests for writing
     * multiple registers. It specifies the number of registers to be written, starting from the starting_address field.
     */
    func::msb_t<std::uint16_t> quantity{};
    /*!
     * @brief The count field contains the value to be written to the first register.
     *
     * @details The count field is a member of the request_fields_wr_multi struct and is used in requests for writing
     * multiple registers. It contains the value to be written to the first register specified by the starting_address
     * field.
     */
    std::uint8_t count{};
};

/*!
 * @brief The request_fields_wr_mask struct contains the starting address, and_mask, and or_mask fields of a Modbus
 * request for writing a register with a mask.
 *
 * @details The request_fields_wr_mask struct is used in requests for writing a register with a mask, where the starting
 * address, and_mask, and or_mask fields specify the starting register address, a bitmask that determines which bits of
 * the register should be changed, and a value to be written to those changed bits.
 */
struct __attribute__((__packed__)) request_fields_wr_mask {
    /*!
     * @brief The starting_address field contains the starting register address of the single register to be written.
     *
     * @details The starting_address field is a member of the request_fields_wr_mask struct and is used in requests for
     * writing a register with a mask. It specifies the starting register address of the single register to be written.
     */
    func::msb_t<std::uint16_t> starting_address{};
    /*!
     * @brief The and_mask field contains a bitmask that determines which bits of the register should be changed.
     *
     * @details The and_mask field is a member of the request_fields_wr_mask struct and is used in requests for writing
     * a register with a mask. It contains a bitmask that determines which bits of the register should be changed,
     * starting from the starting_address field.
     */
    func::msb_t<std::uint16_t> and_mask{};
    /*!
     * @brief The or_mask field contains a value to be written to the changed bits of the register.
     *
     * @details The or_mask field is a member of the request_fields_wr_mask struct and is used in requests for writing a
     * register with a mask. It contains a value to be written to the changed bits of the register specified by the
     * starting_address and and_mask fields.
     */
    func::msb_t<std::uint16_t> or_mask{};
};

/*!
 * @brief The request_fields_fifo struct contains the quantity and count fields of a Modbus request for FIFO operations.
 *
 * @details The request_fields_fifo struct is used in requests for FIFO (First-In, First-Out) operations, where the
 * quantity and count fields specify the number of registers to transfer and the number of registers to be read or
 * written.
 */
struct __attribute__((__packed__)) request_fields_fifo {
    /*!
     * @brief The quantity field contains the number of registers to transfer.
     *
     * @details The quantity field is a member of the request_fields_fifo struct and is used in requests for FIFO
     * (First-In, First-Out) operations. It specifies the number of registers to transfer, either from the input queue
     * to the output queue or from the output queue to the input queue.
     */
    func::msb_t<std::uint16_t> quantity{};
    /*!
     * @brief The count field contains the number of registers to be read or written.
     *
     * @details The count field is a member of the request_fields_fifo struct and is used in requests for reading or
     * writing multiple registers from or to the FIFO. It specifies the number of registers to be read or written,
     * depending on the type of FIFO operation.
     */
    func::msb_t<std::uint16_t> count{};
};

/*!
 * @brief The request_fields_log struct contains the address and quantity fields of a Modbus request for logging
 * operations.
 *
 * @details The request_fields_log struct is used in requests for logging operations, where the address and quantity
 * fields specify the starting register address and the number of registers to be logged.
 */
struct __attribute__((__packed__)) request_fields_log {
    /*!
     * @brief The address field contains the starting register address of the first register to be logged.
     *
     * @details The address field is a member of the request_fields_log struct and is used in requests for logging
     * operations. It specifies the starting register address of the first register to be logged.
     */
    func::msb_t<std::uint16_t> address{};
    /*!
     * @brief The quantity field contains the number of registers to be logged.
     *
     * @details The quantity field is a member of the request_fields_log struct and is used in requests for logging
     * operations. It specifies the number of registers to be logged, starting from the address field.
     */
    func::msb_t<std::uint16_t> quantity{};
};

/*!
 * @brief The request_identification struct contains the MEI type, read mode, and object ID fields of a Modbus request
 * for identification.
 *
 * @details The request_identification struct is used in requests for identification, where the MEI type, read mode, and
 * object ID fields specify the type of identification, the read mode, and the object ID.
 */
struct __attribute__((__packed__)) request_identification {
    /*!
     * @brief The mei_type field contains the type of identification to be performed.
     *
     * @details The mei_type field is a member of the request_identification struct and is used in requests for
     * identification. It specifies the type of identification to be performed, such as manufacturer, product code, or
     * serial number.
     */
    std::uint8_t mei_type{};
    /*!
     * @brief The read_mode field contains the read mode to be used for the identification request.
     *
     * @details The read_mode field is a member of the request_identification struct and is used in requests for
     * identification. It specifies the read mode to be used for the identification request, such as read input
     * registers or read holding registers.
     */
    std::uint8_t read_mode{};
    /*!
     * @brief The object_id field contains the object ID to be identified.
     *
     * @details The object_id field is a member of the request_identification struct and is used in requests for
     * identification. It specifies the object ID to be identified, depending on the type of identification specified by
     * the mei_type field. For example, for a request for the manufacturer name, the object_id field would contain the
     * manufacturer code.
     */
    std::uint8_t object_id{};
};

/*!
 * @brief The response_identification struct contains the MEI type, read mode, and object ID fields of a Modbus response
 * for identification.
 *
 * @details The response_identification struct is used in responses for identification, where the MEI type, read mode,
 * and object ID fields specify the type of identification, the read mode, and the object ID.
 */
struct __attribute__((__packed__)) response_identification {
    /*!
     * @brief The mei_type field contains the type of identification performed.
     *
     * @details The mei_type field is a member of the response_identification struct and is used in responses for
     * identification. It specifies the type of identification performed, such as manufacturer, product code, or serial
     * number.
     */
    std::uint8_t mei_type{};
    /*!
     * @brief The read_mode field contains the read mode used for the identification request.
     *
     * @details The read_mode field is a member of the response_identification struct and is used in responses for
     * identification. It specifies the read mode used for the identification request, such as read input registers or
     * read holding registers.
     */
    std::uint8_t read_mode{};
    /*!
     * @brief The conformity field contains the conformity of the response.
     *
     * @details The conformity field is a member of the response_identification struct and is used to indicate whether
     * the response is conformant to the Modbus protocol. A value of 0 indicates that the response is not conformant,
     * while a value of 1 indicates that the response is conformant.
     */
    std::uint8_t conformity{};
    /*!
     * @brief The more_follows field indicates whether there are more objects to follow.
     *
     * @details The more_follows field is a member of the response_identification struct and is used to indicate whether
     * there are more objects to follow in the response. A value of 0 indicates that there are no more objects to
     * follow, while a value of 1 indicates that there are more objects to follow.
     */
    std::uint8_t more_follows{};
    /*!
     * @brief The next_object_id field contains the object ID of the next object to be identified.
     *
     * @details The next_object_id field is a member of the response_identification struct and is used in responses for
     * identification. It specifies the object ID of the next object to be identified, if there are more objects to be
     * identified.
     */
    std::uint8_t next_object_id{};
    /*!
     * @brief The number_of_objects field contains the number of objects identified.
     *
     * @details The number_of_objects field is a member of the response_identification struct and is used in responses
     * for identification. It specifies the number of objects identified in the response.
     */
    std::uint8_t number_of_objects{};
    /*!
     * @brief The object_id field contains the object ID of the first object identified.
     *
     * @details The object_id field is a member of the response_identification struct and is used in responses for
     * identification. It specifies the object ID of the first object identified in the response.
     */
    std::uint8_t object_id{};
    /*!
     * @brief The object_len field contains the length of the object data.
     *
     * @details The object_len field is a member of the response_identification struct and is used in responses for
     * identification. It specifies the length of the object data, in bytes.
     */
    std::uint8_t object_len{};
};

struct __attribute__((__packed__)) error_fields {
    exception exception_code{};
};

/**
 * @brief This class provides a base implementation of the Modbus protocol.
 *
 * This class provides a base implementation of the Modbus protocol. It defines
 * constants, data types, and functions that are common to all Modbus
 * implementations.
 */
class modbus_base {
public:
    /**
     * @brief The broadcast address
     *
     * This is the broadcast address used for sending messages to all devices on the
     * network.
     */
    static constexpr std::uint8_t broadcast_address = 0;

    /**
     * @brief The maximum valid address
     *
     * This is the maximum valid address that can be used for sending messages.
     */
    static constexpr std::uint8_t max_valid_address = 247;

    /**
     * @brief The maximum number of bits that can be read
     *
     * This is the maximum number of bits that can be read in a single request.
     */
    static constexpr std::uint16_t max_read_bits = 2000;

    /**
     * @brief The maximum number of bits that can be written
     *
     * This is the maximum number of bits that can be written in a single request.
     */
    static constexpr std::uint16_t max_write_bits = 1968;

    /**
     * @brief The maximum number of registers that can be read
     *
     * This is the maximum number of registers that can be read in a single
     * request.
     */
    static constexpr std::uint16_t max_read_registers = 125;

    /**
     * @brief The maximum size of the read FIFO
     *
     * This is the maximum size of the read FIFO, which is used for reading
     * multiple registers at once.
     */
    static constexpr std::uint16_t max_read_fifo = 31;

    /**
     * @brief The maximum number of bytes that can be read from the log
     *
     * This is the maximum number of bytes that can be read from the log.
     */
    static constexpr std::uint16_t max_read_log_bytes = 250;

    /**
     * @brief The maximum number of registers that can be written
     *
     * This is the maximum number of registers that can be written in a single
     * request.
     */
    static constexpr std::uint16_t max_write_registers = 123;

    /**
     * @brief The maximum number of registers that can be written and read
     *
     * This is the maximum number of registers that can be written and read in a
     * single request.
     */
    static constexpr std::uint16_t max_wr_write_registers = 121;

    /**
     * @brief The maximum number of registers that can be read and written
     *
     * This is the maximum number of registers that can be read and written in a
     * single request.
     */
    static constexpr std::uint16_t max_wr_read_registers = 125;

    /**
     * @brief The maximum length of the PDU
     *
     * This is the maximum length of the PDU, which is the entire Modbus message,
     * including the header and CRC.
     */
    static constexpr std::uint16_t max_pdu_length = 253;

    /**
     * @brief The maximum length of the ADU
     *
     * This is the maximum length of the ADU, which is the actual data portion of
     * the Modbus message, without the header and CRC.
     */
    static constexpr std::uint16_t max_adu_length = 256;

    /**
     * @brief The minimum length of the ADU
     *
     * This is the minimum length of the ADU, which is the actual data portion of
     * the Modbus message, without the header and CRC.
     */
    static constexpr std::uint16_t min_adu_length = 3;

    /**
     * @brief The maximum function ID
     *
     * This is the maximum function ID that can be used in a Modbus request.
     */
    static constexpr std::size_t max_function_id = 0x7f;

    /**
     * @brief The value for on coils
     *
     * This is the value for on coils, which is 0xff00.
     */
    static constexpr std::uint16_t on_coil_value = 0xff00;

    /**
     * @brief The value for off coils
     *
     * This is the value for off coils, which is 0x0000.
     */
    static constexpr std::uint16_t off_coil_value = 0x0000;

    /**
     * @brief The value for more follows
     *
     * This is the value for more follows, which is 0xff.
     */
    static constexpr std::uint8_t more_follows = 0xff;

    /**
     * @brief The value for no more follows
     *
     * This is the value for no more follows, which is 0x00.
     */
    static constexpr std::uint8_t no_more_follows = 0x00;

    /**
     * @brief The MEI type
     *
     * This is the MEI type, which is 0x0e.
     */
    static constexpr std::uint8_t mei_type = 0x0e;

    /**
     * @brief The request type for reading
     *
     * This is the request type for reading, which is a packet with a header,
     * request fields, and a CRC.
     */
    using request_type_read = packet<header, request_fields_read, crc16ansi>;

    /**
     * @brief The request type for writing a single register
     *
     * This is the request type for writing a single register, which is a packet
     * with a header, request fields, and a CRC.
     */
    using request_type_wr_single = packet<header, request_fields_wr_single, crc16ansi>;

    /**
     * @brief The request type for writing a mask of registers
     *
     * This is the request type for writing a mask of registers, which is a packet
     * with a header, request fields, and a CRC.
     */
    using request_type_wr_mask = packet<header, request_fields_wr_mask, crc16ansi>;

    /**
     * @brief The request type for errors
     *
     * This is the request type for errors, which is a packet with a header and a
     * CRC.
     */
    using request_type_err = packet<header, null_field, crc16ansi>;

    /**
     * @brief The request type for the FIFO
     *
     * This is the request type for the FIFO, which is a packet with a header and a
     * function code.
     */
    using request_type_fifo = packet<header, func::msb_t<std::uint16_t>, crc16ansi>;

    /**
     * @brief The request type for the log
     *
     * This is the request type for the log, which is a packet with a header,
     * request fields, and a CRC.
     */
    using request_type_log = packet<header, request_fields_log, crc16ansi>;

    /**
     * @brief The request type for the log level
     *
     * This is the request type for the log level, which is a packet with a header
     * and a CRC.
     */
    using request_type_log_level = packet<header, null_field, crc16ansi>;

    /**
     * @brief The message type
     *
     * This is the message type, which is a packet accessor with a maximum length.
     */
    using msg_type = packet_accessor<max_adu_length>;

protected:
    /**
     * @brief The last error
     */
    exception error_{exception::no_error};
    /**
     * @brief The input message
     */
    msg_type input_msg_{};
    /**
     * @brief The output message
     */
    msg_type output_msg_{};
    /**
     * @brief The exception status
     */
    std::uint8_t exception_status_{};
    /**
     * @brief The diagnostic register
     */
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

    /**
     * @brief Resets the state of the object
     */
    virtual inline void
    reset() noexcept
        = 0;

    /**
     * @brief Sets the diagnostic register
     *
     * @param val The value to set the diagnostic register to
     */
    inline void
    diagnostic_register(std::uint16_t val) noexcept
    {
        diagnostic_register_ = val;
    }

    /**
     * @brief Gets the diagnostic register
     *
     * @return std::uint16_t The value of the diagnostic register
     */
    [[nodiscard]] inline std::uint16_t
    diagnostic_register() const noexcept
    {
        return diagnostic_register_;
    }

    /**
     * @brief Gets the exception status
     *
     * @return std::uint8_t The exception status
     */
    [[nodiscard]] inline std::uint8_t
    exception_status() const noexcept
    {
        return exception_status_;
    }

    /**
     * @brief Gets the last error
     *
     * @return exception The last error
     */
    inline exception
    error() noexcept
    {
        return error_;
    }

    /**
     * @brief Gets the last error
     *
     * @return exception The last error
     */
    [[nodiscard]] inline exception
    error() const noexcept
    {
        return error_;
    }

    /**
     * @brief Sends a message
     *
     * @param begin An iterator to the beginning of the message
     * @param end An iterator to the end of the message
     * @return true If the message was sent successfully
     * @return false If the message could not be sent
     */
    virtual bool
    send(msg_type::array_type::iterator begin, msg_type::array_type::iterator end) noexcept
        = 0;

    /**
     * @brief Receives a message
     *
     * @param begin An iterator to the beginning of the message
     * @param end An iterator to the end of the message
     * @return exception::bad_data If the message is not long enough
     * @return exception::bad_crc If the CRC does not match
     * @return exception::slave_or_server_busy If the slave is busy
     */
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

    /**
     * @brief Checks if the slave is idle
     *
     * @return true If the slave is idle
     * @return false If the slave is not idle
     */
    virtual inline bool
    idle() noexcept
        = 0;

    virtual exception
    received() noexcept
        = 0;

    /**
     * @brief Processes the received message
     *
     * @return exception::illegal_function If the function code is not supported
     * @return exception::illegal_data_address If the data address is not valid
     * @return exception::illegal_data_value If the data value is not valid
     * @return exception::server_device_failure If an unexpected error occurred
     */
    virtual exception
    processing() noexcept
        = 0;

    /**
     * @brief Gets the input message
     *
     * @return msg_type& A reference to the input message
     */
    inline msg_type&
    input() noexcept
    {
        return input_msg_;
    }

    /**
     * @brief Gets the input message
     *
     * @return msg_type const& A const reference to the input message
     */
    [[nodiscard]] inline msg_type const&
    input() const noexcept
    {
        return input_msg_;
    }

    /**
     * @brief Gets the output message
     *
     * @return msg_type& A reference to the output message
     */
    inline msg_type&
    output() noexcept
    {
        return output_msg_;
    }

    /**
     * @brief Gets the output message
     *
     * @return msg_type const& A const reference to the output message
     */
    [[nodiscard]] inline msg_type const&
    output() const noexcept
    {
        return output_msg_;
    }
    /**
     * @brief Destroys the modbus_base object
     */
    virtual ~modbus_base() = default;
};

template <std::uint16_t Inputs, std::uint16_t Coils, std::uint16_t InputRegisters, std::uint16_t HoldingRegisters,
          std::uint16_t Fifo = 1>
class slave;

/**
 * @brief Concept to check if a type is a container
 *
 * A container is a class or struct that provides an ordered collection of elements,
 * allowing fast access to any element by its index.
 *
 * This concept defines the requirements for a type to be considered a container.
 * A type `T` satisfies the requirements of this concept if it provides two types:
 * - `size_type`, which is an unsigned integer type used to represent the size of the container; and
 * - `value_type`, which is the type of the elements stored in the container.
 *
 * In addition, the type must provide two non-member functions:
 * - `size(T const& c)`, which returns the size of the container `c` as a `size_type`; and
 * - `operator[](T const& c, size_type n)`, which returns a reference to the element at index `n` in the container `c`.
 *
 **/
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
class slave_base;
}    // namespace xitren::modbus
