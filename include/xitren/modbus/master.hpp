/*!
_ _
__ _(_) |_ _ _ ___ _ _
\ \ / |  _| '_/ -_) ' \
/_\_\_|\__|_| \___|_||_|
* @date 15.02.2024
*/
#pragma once

#include <xitren/modbus/commands/command.hpp>
#include <xitren/modbus/master.hpp>

#include <optional>
#include <ranges>
#include <variant>

namespace xitren::modbus {

/**
 * @brief The modbus master class is used to send requests to a modbus slave device.
 *
 * The modbus master class is responsible for sending requests to a modbus slave device, receiving responses, and
 *managing the communication with the slave device. The modbus master class provides a simple interface for sending
 *requests to the slave device, and then waiting for a response.
 *
 * The modbus master class uses the modbus protocol to communicate with the slave device. The modbus protocol is a
 *serial communication protocol that is used for communication between devices such as PLCs, HMIs, and other industrial
 *automation devices. The modbus protocol uses a master-slave architecture, where the master device sends requests to
 *the slave device, and the slave device responds with a response.
 *
 * The modbus master class provides a simple interface for sending requests to the slave device. The master device can
 *send a request to the slave device by calling the run_async() method, which will send the request to the slave device.
 *The master device can then wait for a response from the slave device by calling the wait() method.
 *
 * The modbus master class provides a number of methods for managing the communication with the slave device. These
 *methods include:
 *
 * - run_async() - This method is used to send a request to the slave device.
 * - << operator - This operator is used to send a request to the slave device.
 * - >> operator - This operator is used to receive a response from the slave device.
 * - timer_start() - This method is used to start a timer that will trigger if a response is not received from the slave
 *device within a specified time.
 * - timer_stop() - This method is used to stop the timer that is started by timer_start().
 * - wait() - This method is used to wait for a response from the slave device.
 *
 * The modbus master class also provides a number of methods for managing the state of the communication with the slave
 *device. These methods include:
 *
 * - state() - This method is used to get the current state of the communication with the slave device.
 * - idle() - This method is used to check if the master device is currently idle.
 * - processing() - This method is used to check if the master device is currently processing a request or response.
 * - reset() - This method is used to reset the state of the communication with the slave device.
 *
 * The modbus master class uses the modbus_command class to represent a request that is sent to the slave device. The
 *modbus_command class provides a simple interface for constructing and sending requests to the slave device. The
 *modbus_command class can be used to construct requests for a variety of different functions, including reading and
 *writing registers, and reading and writing coils.
 *
 * The modbus_master class uses the modbus_base class to provide a common implementation for a modbus master device. The
 *modbus_base class provides an implementation for the modbus protocol, including functions for sending and receiving
 *messages, and for managing the communication with the slave device.
 *
 * The modbus_master class is designed to be used in a polled mode of operation, where the master device sends a request
 *to the slave device, and then waits for a response. The modbus_master class does not provide an asynchronous interface
 *for sending requests to the slave device.
 *
 * The modbus_master class is designed to be used in a single-threaded environment. The modbus_master class is not
 *thread-safe, and should not be used in a multi-threaded environment.
 *
 * The modbus_master class is designed to be used in an embedded environment, where the master device is typically a
 *microcontroller or other low-powered device. The modbus_master class is designed to be efficient in terms of memory
 *usage and CPU usage.
 *
 * The modbus_master class is designed to be flexible and extensible. The modbus_master class provides a simple
 *interface for sending requests to the slave device, and provides a common implementation for the modbus protocol. The
 *modbus_master class can be extended to support additional features and functions, by providing additional methods and
 *functions for sending requests to the slave device.
 *
 * The modbus_master class is designed to be easy to use. The modbus_master class provides a simple interface for
 *sending requests to the slave device, and provides a common implementation for the modbus protocol. The modbus_master
 *class is designed to be easy to understand and use, even for developers who are new to the modbus protocol.
 *
 * The modbus_master class is designed to be reliable and robust. The modbus_master class provides a number of
 *mechanisms for ensuring that communication with the slave device is reliable and robust, including error detection and
 *correction, and timeout mechanisms. The modbus_master class is designed to handle errors gracefully, and to recover
 *from errors automatically.
 *
 * The modbus_master class is designed to be standards-compliant. The modbus_master class follows the specifications and
 *standards for the modbus protocol, including the Modbus Application Protocol Specification (MAP), and the Modbus
 *Message Structure and Communication Protocol (MBAP). The modbus_master class is designed to be compatible with other
 *devices that follow the modbus protocol.
 *
 * The modbus_master class is designed to be modular and scalable. The modbus_master class is designed to be modular, so
 *that different parts of the class can be used independently, or can be combined to create more complex master devices.
 *The modbus_master class is designed to be scalable, so that it can be used in a variety of different applications,
 *from small projects to large-scale industrial automation systems.
 *
 * The modbus_master class is designed to be efficient in terms of memory usage and CPU usage. The modbus_master class
 *is designed to use minimal memory, and to use minimal CPU resources while communicating with the slave device. The
 *modbus_master class is designed to be efficient in terms of both memory usage and CPU usage, so that it can be used in
 *a variety of different embedded environments.
 *
 * The modbus_master class is designed to be lightweight and fast. The modbus_master class is designed to be lightweight
 *and fast, so that it can be used in applications where speed is critical, such as in real-time control systems. The
 *modbus_master class is designed to be fast in terms of both execution time and response time.
 **/
class master : public modbus_base {

    /**
     * @brief A structure that contains the slave address and function code of a request.
     *
     * This structure is used to store the slave address and function code of a request that is sent to a Modbus slave
     *device. The slave address is used to identify the specific slave device that the request is intended for, and the
     *function code is used to identify the specific function that the request is intended to perform.
     *
     * The request_data structure is used by the modbus_master class to manage the communication with the slave device.
     *When a request is sent to the slave device, the request_data structure is used to store the slave address and
     *function code of the request. When a response is received from the slave device, the request_data structure is
     *used to identify the specific request that the response is associated with.
     *
     * The request_data structure is designed to be lightweight and efficient, and to be used in a variety of different
     *applications, including in embedded systems and in industrial automation systems.
     *
     * @note The request_data structure is designed to be used in a single-threaded environment. It is not thread-safe,
     *and should not be used in a multi-threaded environment.
     */
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
    /**
     * @brief Sends a request to the slave device asynchronously.
     *
     * This function is used to send a request to the slave device asynchronously. The request is sent as a
     *modbus_command object, which contains the request data. The modbus_command object is cloned, and the request data
     *is copied into the output message buffer. The output message buffer is then sent to the slave device.
     *
     * If the master device is currently processing a request, this function returns false. Otherwise, if the request
     *can be sent, the function returns true.
     *
     * @param in_data The modbus_command object that contains the request data.
     * @return true If the request was sent successfully.
     * @return false If the master device is currently processing a request.
     */
    bool
    run_async(command const& in_data)
    {
        if (command_ != nullptr) {
            // If the master device is currently processing a request, return false.
            WARN() << "busy";
            return false;
        }

        // Get a pointer to the beginning and end of the request data.
        auto const st{in_data.begin()};
        auto const fn{in_data.begin() + in_data.size()};

        // Copy the request data into the output message buffer.
        std::copy(st, fn, output_msg_.storage().begin());
        output_msg_.size(in_data.size());

        // Clone the modbus_command object.
        command_ = in_data.clone(vault_);

        // Send the output message to the slave device.
        return push(output_msg_);
    }

    /*!
     * @brief Overloaded input operator for the modbus_master class.
     *
     * This operator is used to send a modbus_command object to the slave device.
     * Once the command is sent, the master will wait for a response from the slave.
     *
     * @param in_data The modbus_command object to be sent to the slave.
     * @return modbus_master& A reference to the modbus_master object.
     */
    master&
    operator<<(command const& in_data)
    {
        run_async(in_data);
        return *this;
    }

    /*!
     * @brief Receive data from the input message
     *
     * @param out_data The modbus command object to store the received data
     * @return modbus_master& A reference to the modbus master object
     */
    master&
    operator>>(command& out_data)
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

    virtual void
    wait()
    {}

    /*!
     * @brief This function is called when the timer expires.
     *
     * This function handles the various states of the master and updates the state machine.
     * If the state is `master_state::waiting_reply`, the state is set to `master_state::processing_error` and the
     * `no_answer` function of the command is called. The `command_` pointer is set to `nullptr`. If the state is not
     * `master_state::waiting_reply`, a warning is printed.
     *
     * @param state_ The current state of the master.
     */
    void
    timer_expired()
    {
        switch (state_) {
        case master_state::waiting_reply:
            TRACE() << "wait -> proc_err";
            state_ = master_state::processing_error;
            if (command_ != nullptr) {
                command_->no_answer();
                command_ = nullptr;
            }
            break;
        default:
            WARN() << "state undefined: " << static_cast<int>(state_);
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
            TRACE() << "wait -> un_err";
            state_ = master_state::unrecoverable_error;
            return exception::unknown_exception;
            break;
        default:
            WARN() << "state undef: " << static_cast<int>(state_);
            break;
        }
        return exception::no_error;
    }

    /*!
     * @brief Returns whether the master is currently idle or waiting for a reply.
     *
     * @return `true` if the master is idle or waiting for a reply, `false` otherwise.
     */
    inline bool
    idle() noexcept override
    {
        return (master_state::idle == state_) || (master_state::waiting_reply == state_);
    }

    /*!
     * @brief This function is used to process the incoming data and update the state of the master.
     *
     * This function is called repeatedly by the main loop of the master. It processes the incoming data and updates the
     * state of the master accordingly.
     *
     * @return Returns an exception indicating the type of error that occurred, or no_error if no error occurred.
     */
    exception
    processing() noexcept override
    {
        switch (state_) {
        case master_state::processing_reply:
        case master_state::processing_error:
            // Trace that the state is being changed to idle.
            TRACE() << "proc -> idle";
            state_ = master_state::idle;
            break;
        case master_state::waiting_reply:
        case master_state::idle:
            // Do nothing if the state is waiting for a reply or if it is idle.
            break;
        default:
            // Print a warning message if an undefined state is encountered.
            WARN() << "state undef: " << static_cast<int>(state_);
            break;
        }
        return exception::no_error;
    }

    /*!
     * @brief Returns the current state of the master
     *
     * @return The current state of the master
     */
    inline master_state
    state() noexcept
    {
        return state_;
    }

    /*!
     * @brief Returns the current state of the master
     *
     * @return The current state of the master
     */
    inline master_state
    state() const noexcept
    {
        return state_;
    }

    /*!
     * @brief Sends a message to the slave and waits for a response.
     *
     * @param msg The message to send.
     * @return `true` if the message was sent successfully, `false` otherwise.
     */
    bool
    push(msg_type& msg)
    {
        state_ = master_state::waiting_reply;
        if (!send(msg.storage().begin(), msg.storage().begin() + msg.size())) {
            TRACE() << "wait -> un_err";
            state_ = master_state::unrecoverable_error;
            return false;
        }
        if (!timer_start(100)) {
            TRACE() << "wait -> un_err";
            state_ = master_state::unrecoverable_error;
            return false;
        }
        return true;
    }

    /*!
     * @brief Resets the master to its initial state.
     *
     * This function resets the master to its initial state, clearing any pending requests or responses, and resetting
     * the internal state machine to the idle state.
     */
    inline void
    reset() noexcept override
    {
        TRACE() << "-> idle";
        state_   = master_state::idle;
        error_   = exception::no_error;
        command_ = nullptr;
    }

    ~master() override = default;

protected:
    volatile master_state state_{
        master_state::
            idle};    // FIXME: See p.20 of
                      // https://wiki.yandex-team.ru/lavka/dev/robolab/programmirovanie/01-koncepcii-i-instrukcii/c-embedded-guidelines/?revision=149426615

private:
    request_data                ask_{};
    command*                    command_{nullptr};
    command::command_vault_type vault_{};

    inline bool
    wait_input_msg()
    {
        while ((state_ == master_state::waiting_reply) || (state_ == master_state::unrecoverable_error)) {
            wait();
        }
        return state_ == master_state::processing_reply;
    }

    /*!
     * @brief This function is used to handle incoming commands from the slave.
     *
     * If the command is from a different slave, it is stored in the command_vault_type vault_ and the master enters the
     * waiting_reply state. If the command is from the correct slave, it is processed and the master enters the
     * processing_reply state. If an error occurs during processing, the master enters the unrecoverable_error state.
     *
     * @return An exception indicating the type of error that occurred.
     */
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

    // This function deserializes a Modbus message from the input buffer.
    // It returns a std::pair containing the deserialized message and an exception object.
    // The exception object indicates any errors that occurred during deserialization.
    // The function throws an exception if an error occurs.
    template <typename Header, typename Fields, typename Type>
    inline std::pair<msg_type::fields_out_ptr<Header, Fields, Type>, exception>
    input_msg(std::uint8_t slave)
    {
        auto pack = input_msg_.template deserialize_no_check<Header, Fields, Type, crc16ansi>();

        // Check if the slave ID of the incoming message matches the expected slave ID.
        // If not, set the state to processing error and return an exception indicating a bad slave.
        if (pack.header->slave_id != slave) [[unlikely]] {
            state_ = master_state::processing_error;
            return {{}, exception::bad_slave};
        }

        // Check if the function code of the incoming message is an error reply.
        // If so, set the state to processing error and return an exception indicating an illegal function.
        if (pack.header->function_code & error_reply_mask) [[unlikely]] {
            state_ = master_state::processing_error;
            return {{}, exception::illegal_function};
        }

        // Return the deserialized message and a no-error exception.
        return {pack, exception::no_error};
    }

    /**
     * @brief Request a diagnostic function from a Modbus slave device
     *
     * This function sends a diagnostic request to a Modbus slave device, requesting a specific
     * diagnostic function. The function returns the result of the request, along with any data
     * returned by the slave.
     *
     * @param master The Modbus master device to use for the request
     * @param slave The slave device to request the diagnostic function from
     * @param sub The diagnostic function to request
     * @param data A reference to a variable to store the data returned by the slave
     * @return An exception indicating the result of the request
     */
    static inline exception
    request(master& master, std::uint8_t slave, diagnostics_sub_function sub, std::uint16_t& data)
    {
        master.ask_ = {slave, function::diagnostic};
        master.output_msg_
            .template serialize<header, func::msb_t<std::uint16_t>, func::msb_t<std::uint16_t>, crc16ansi>(
                {{slave, static_cast<uint8_t>(function::diagnostic)}, static_cast<uint16_t>(sub), 0, nullptr});

        if (!master.push(master.output_msg_)) {
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

}    // namespace xitren::modbus
