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

template <std::uint8_t Slave, std::uint16_t Address, std::size_t Size, std::array<std::uint16_t, Size> Data,
          std::invocable<exception> auto Callback>
class write_registers : public command {
    using struct_type = struct __attribute__((__packed__)) tag_ {
        request_fields_wr_multi                      fields;
        std::array<func::msb_t<std::uint16_t>, Size> data;
    };

    static constexpr std::array<func::msb_t<std::uint16_t>, Size>
    swap(std::array<std::uint16_t, Size> const& val) noexcept
    {
        std::array<func::msb_t<std::uint16_t>, Size> data_r;
        for (decltype(Size) i = 0; i < Size; i++) {
            data_r[i] = val[i];
        }
        return data_r;
    }

public:
    static constexpr auto output_command = packet<header, struct_type, crc16ansi>::serialize(
        header{Slave, static_cast<uint8_t>(function::write_multiple_registers)},
        struct_type{request_fields_wr_multi{Address, Size, Size * 2}, swap(Data)});

    consteval write_registers() noexcept : command{Slave, Address}
    {
        std::uint32_t const max_address = Address + Size - 1;
        if (max_address > static_cast<std::uint32_t>(std::numeric_limits<std::uint16_t>::max())) [[unlikely]] {
            error(exception::illegal_data_address);
            return;
        }
    }

    inline iterator
    begin() noexcept override
    {
        return const_cast<iterator>(output_command.begin());
    }

    inline constexpr const_iterator
    begin() const noexcept override
    {
        return output_command.begin();
    }

    inline iterator
    end() noexcept override
    {
        return const_cast<iterator>(output_command.end());
    }

    inline constexpr const_iterator
    end() const noexcept override
    {
        return output_command.end();
    }

    inline std::size_t
    size() noexcept override
    {
        return output_command.size();
    }

    inline constexpr std::size_t
    size() const noexcept override
    {
        return output_command.size();
    }

    void
    no_answer() noexcept override
    {
        Callback(error(exception::bad_slave));
    }

    command*
    clone(command_vault_type& vault) const noexcept override
    {
        static_assert(command::command_buffer_max >= sizeof(write_registers),
                      "Command realization size exceeded storage area!");
        return new (&vault) write_registers(*this);
    }

    std::shared_ptr<command>
    clone() const noexcept override
    {
        return std::make_shared<write_registers>(*this);
    }

    exception
    receive(msg_type const& message) noexcept override
    {
        auto [pack, err] = input_msg<header, std::uint8_t, std::uint8_t>(slave(), message);
        if (error(err) != exception::no_error) [[unlikely]] {
            Callback(err);
            return err;
        }
        Callback(exception::no_error);
        return exception::no_error;
    }

    ~write_registers() noexcept override = default;
};

}    // namespace xitren::modbus::commands::instant