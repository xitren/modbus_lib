/*!
     _ _
__ _(_) |_ _ _ ___ _ _
\ \ / |  _| '_/ -_) ' \
/_\_\_|\__|_| \___|_||_|
* @date 15.02.2024
*/
#pragma once

#include "command.hpp"

namespace xitren::modbus::commands {

/**
 * @brief Command to read device identification (0x2B/0x0E).
 *
 * Requests a single identification object by object ID and returns the
 * payload as a byte span to the callback.
 *
 * Supported object IDs are defined by `object_id_code` and include vendor
 * name, product code, and major/minor revision.
 */
class read_identification : public command {
public:
    /**
     * @brief Construct a new read identification object
     *
     * @param slave The slave device address
     * @param address The object address
     * @param callback The function to be called with the response
     */
    read_identification(std::uint8_t slave, std::uint8_t address, types::callback_identification_type callback) noexcept
        : command{slave, address}, callback_{std::move(callback)}
    {
        if (address >= static_cast<std::uint8_t>(object_id_code::max)) [[unlikely]] {
            error(exception::illegal_data_address);
            return;
        }
        // GCOVR_EXCL_START
        if (!msg_output_.template serialize<header, request_identification, std::uint8_t, crc16ansi>(
                {{slave, static_cast<std::uint8_t>(function::read_device_identification)},
                 {modbus_base::mei_type, static_cast<std::uint8_t>(identification_id::individual_access), address},
                 0,
                 nullptr})) {
            error(exception::illegal_data_address);
            return;
        }
        // GCOVR_EXCL_STOP
    }

    /**
     * @brief Returns an iterator to the beginning of the message
     *
     * @return iterator
     */
    inline iterator
    begin() noexcept override
    {
        return msg_output_.storage().begin();
    }

    /**
     * @brief Returns an iterator to the beginning of the message
     *
     * @return const_iterator
     */
    inline const_iterator
    begin() const noexcept override
    {
        return msg_output_.storage().begin();
    }

    /**
     * @brief Returns an iterator to the end of the message
     *
     * @return iterator
     */
    inline iterator
    end() noexcept override
    {
        return msg_output_.storage().end();
    }

    /**
     * @brief Returns an iterator to the end of the message
     *
     * @return const_iterator
     */
    inline const_iterator
    end() const noexcept override
    {
        return msg_output_.storage().end();
    }

    /**
     * @brief Returns the size of the message
     *
     * @return std::size_t
     */
    inline std::size_t
    size() noexcept override
    {
        return msg_output_.size();
    }

    /**
     * @brief Returns the size of the message
     *
     * @return std::size_t
     */
    inline std::size_t
    size() const noexcept override
    {
        return msg_output_.size();
    }

    /**
     * @brief Returns a reference to the message object
     *
     * @return msg_type&
     */
    inline msg_type&
    msg() noexcept
    {
        return msg_output_;
    }

    /**
     * @brief Called when no response is received
     *
     */
    void
    no_answer() noexcept override
    {
        callback_(error(exception::bad_slave), 0, nullptr, nullptr);
    }

    /**
     * @brief Creates a new instance of the command
     *
     * @param vault The command vault
     * @return modbus_command*
     */
    command*
    clone(command_vault_type& vault) const noexcept override
    {
        static_assert(command::command_buffer_max >= sizeof(read_identification),
                      "Command realization size exceeded storage area!");
        return new (&vault) read_identification(*this);
    }

    /**
     * @brief Creates a new instance of the command
     *
     * @return std::shared_ptr<modbus_command>
     */
    std::shared_ptr<command>
    clone() const noexcept override
    {
        return std::make_shared<read_identification>(*this);
    }

    /**
     * @brief Processes a response message
     *
     * @param message The response message
     * @return exception
     */
    exception
    receive(msg_type const& message) noexcept override
    {
        auto [pack, err] = input_msg<header, response_identification, std::uint8_t>(slave(), message);
        if (error(err) != exception::no_error) [[unlikely]]
            return err;
        for (std::size_t i{}; (i < values_.size()) && (i < pack.size); i++) {}
        std::copy(pack.data, pack.data + pack.size, values_.begin());
        callback_(exception::no_error, pack.fields->object_id, values_.begin(), values_.begin() + pack.size);
        return exception::no_error;
    }

    /**
     * @brief Destroy the read identification object
     *
     */
    ~read_identification() noexcept override = default;

private:
    types::callback_identification_type callback_;
    msg_type                            msg_output_{};
    std::array<char, modbus_base::max_pdu_length> values_{};
};

}    // namespace xitren::modbus::commands
