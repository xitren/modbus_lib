#pragma once

#include <xitren/modbus/master/modbus_command.hpp>

namespace xitren::modbus {

class read_identification : public modbus_command {
public:
    read_identification(std::uint8_t slave, std::uint8_t address, types::callback_identification_type callback) noexcept
        : modbus_command{slave, address}, callback_{std::move(callback)}
    {
        if (address >= static_cast<std::uint8_t>(object_id_code::max)) [[unlikely]] {
            error(exception::illegal_data_address);
            return;
        }
        if (!msg_output_.template serialize<header, request_identification, std::uint8_t, crc16ansi>(
                {{slave, static_cast<std::uint8_t>(function::read_device_identification)},
                 {modbus_base::mei_type, static_cast<std::uint8_t>(identification_id::individual_access), address},
                 0,
                 nullptr})) {
            error(exception::illegal_data_address);
            return;
        }
    }

    inline iterator
    begin() noexcept override
    {
        return msg_output_.storage().begin();
    }

    inline const_iterator
    begin() const noexcept override
    {
        return msg_output_.storage().begin();
    }

    inline iterator
    end() noexcept override
    {
        return msg_output_.storage().end();
    }

    inline const_iterator
    end() const noexcept override
    {
        return msg_output_.storage().end();
    }

    inline std::size_t
    size() noexcept override
    {
        return msg_output_.size();
    }

    inline std::size_t
    size() const noexcept override
    {
        return msg_output_.size();
    }

    inline msg_type&
    msg() noexcept
    {
        return msg_output_;
    }

    void
    no_answer() noexcept override
    {
        callback_(error(exception::bad_slave), 0, nullptr, nullptr);
    }

    modbus_command*
    clone(command_vault_type& vault) const noexcept override
    {
        static_assert(modbus_command::command_buffer_max >= sizeof(read_identification),
                      "Command realization size exceeded storage area!");
        return new (&vault) read_identification(*this);
    }

    std::shared_ptr<modbus_command>
    clone() const noexcept override
    {
        return std::make_shared<read_identification>(*this);
    }

    exception
    receive(msg_type const& message) noexcept override
    {
        static std::array<char, modbus_base::max_pdu_length> values{};
        auto [pack, err] = input_msg<header, response_identification, std::uint8_t>(slave(), message);
        if (error(err) != exception::no_error) [[unlikely]]
            return err;
        for (std::size_t i{}; (i < values.size()) && (i < pack.size); i++) {}
        std::copy(pack.data, pack.data + pack.size, values.begin());
        callback_(exception::no_error, pack.fields->object_id, values.begin(), values.begin() + pack.size);
        return exception::no_error;
    }

    ~read_identification() noexcept override = default;

private:
    types::callback_identification_type callback_;
    msg_type                            msg_output_{};
};

}    // namespace xitren::modbus
