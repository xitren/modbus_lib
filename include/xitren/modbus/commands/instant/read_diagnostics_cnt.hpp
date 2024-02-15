/*!
     _ _
__ _(_) |_ _ _ ___ _ _
\ \ / |  _| '_/ -_) ' \
/_\_\_|\__|_| \___|_||_|
* @date 15.02.2024
*/
#pragma once

#include "../command.hpp"

namespace xitren::modbus::commands::instant {

template <std::uint8_t Slave, diagnostics_sub_function Sub, std::invocable<exception, std::uint16_t> auto Callback>
class read_diagnostics_cnt : public command {

public:
    /**
     * @brief The statically initialized output command of the read diagnostics count function
     */
    static constexpr auto output_command = packet<header, func::msb_t<std::uint16_t>, crc16ansi>::serialize(
        header{Slave, static_cast<uint8_t>(function::diagnostic)}, static_cast<uint16_t>(Sub));

    /**
     * @brief Construct a new read diagnostics count static object
     *
     * Initializes the modbus_command object with the given slave address and sub function
     * and sets the error state to exception::no_error
     *
     * @param Slave The slave address of the device to read the diagnostics count from
     * @param Sub The sub function of the read diagnostics count to execute
     */
    consteval read_diagnostics_cnt() noexcept : command{Slave, static_cast<uint16_t>(Sub)}
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

    /**
     * @brief Returns an iterator to the beginning of the output command
     *
     * @return iterator An iterator to the beginning of the output command
     */
    inline iterator
    begin() noexcept override
    {
        return const_cast<iterator>(output_command.begin());
    }

    /**
     * @brief Returns an iterator to the beginning of the output command
     *
     * @return const_iterator An iterator to the beginning of the output command
     */
    inline constexpr const_iterator
    begin() const noexcept override
    {
        return output_command.begin();
    }

    /**
     * @brief Returns an iterator to the end of the output command
     *
     * @return iterator An iterator to the end of the output command
     */
    inline iterator
    end() noexcept override
    {
        return const_cast<iterator>(output_command.end());
    }

    /**
     * @brief Returns an iterator to the end of the output command
     *
     * @return const_iterator An iterator to the end of the output command
     */
    inline constexpr const_iterator
    end() const noexcept override
    {
        return output_command.end();
    }

    /**
     * @brief Returns the size of the output command
     *
     * @return std::size_t The size of the output command
     */
    inline std::size_t
    size() noexcept override
    {
        return output_command.size();
    }

    /**
     * @brief Returns the size of the output command
     *
     * @return std::size_t The size of the output command
     */
    inline constexpr std::size_t
    size() const noexcept override
    {
        return output_command.size();
    }

    /**
     * @brief Sets the error state of the command to exception::bad_slave
     *
     */
    void
    no_answer() noexcept override
    {
        Callback(error(exception::bad_slave), 0);
    }

    /**
     * @brief Creates a new modbus_command object that is a clone of the current object
     *
     * @param vault The command_vault_type object used to allocate memory for the clone
     * @return modbus_command* A pointer to the newly created clone
     */
    command*
    clone(command_vault_type& vault) const noexcept override
    {
        static_assert(command::command_buffer_max >= sizeof(read_diagnostics_cnt),
                      "Command realization size exceeded storage area!");
        return new (&vault) read_diagnostics_cnt(*this);
    }

    /**
     * @brief Creates a new shared_ptr to a modbus_command object that is a clone of the current object
     *
     * @return std::shared_ptr<modbus_command> A shared_ptr to the newly created clone
     */
    std::shared_ptr<command>
    clone() const noexcept override
    {
        return std::make_shared<read_diagnostics_cnt>(*this);
    }

    /**
     * @brief Processes an incoming modbus message and updates the error state and values of the command
     *
     * @param message The incoming modbus message
     * @return exception The error state of the command after processing the message
     */
    exception
    receive(msg_type const& message) noexcept override
    {
        static std::uint16_t values{};
        auto [pack, err] = input_msg<header, func::msb_t<std::uint16_t>, func::msb_t<std::uint16_t>>(slave(), message);
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

    /**
     * @brief Default destructor
     *
     */
    ~read_diagnostics_cnt() noexcept override = default;
};
}    // namespace xitren::modbus::commands::instant