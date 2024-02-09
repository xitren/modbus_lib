#pragma once

#include "modbus_command.hpp"

namespace loveka::components::modbus {

class read_bits : public modbus_command {
public:
    read_bits(std::uint8_t slave, std::uint16_t address, std::size_t size) noexcept
        : read_bits{slave, address, size, nullptr}
    {}

    read_bits(std::uint8_t slave, std::uint16_t address, std::size_t size,
              types::callback_bits_type callback) noexcept
        : size_{size}, callback_{std::move(callback)}, modbus_command{slave, address}
    {
        const std::uint32_t max_address = address + size_ - 1;
        if (max_address > static_cast<std::uint32_t>(std::numeric_limits<std::uint16_t>::max()))
            [[unlikely]] {
            error(exception::illegal_data_address);
            return;
        }
        if (!msg().template serialize<header, request_fields_read, std::uint8_t, crc16ansi>(
                {{slave, static_cast<std::uint8_t>(function::read_coils)},
                 {address, static_cast<std::uint16_t>(size_)},
                 0,
                 nullptr})) {
            error(exception::illegal_data_address);
            return;
        }
    }

    std::shared_ptr<modbus_command>
    clone(command_vault_pointer vault = nullptr) noexcept override
    {
        if (vault != nullptr) {
            return std::shared_ptr<modbus_command>{new (vault) read_bits(*this)};;
        } else {
            return std::make_shared<read_bits>(*this);
        }
    }

    exception
    receive(const msg_type& message) noexcept override
    {
        static types::bits_array_type values{};
        auto [pack, err] = input_msg<header, std::uint8_t, std::uint8_t>(slave(), message);
        if (error(err) != exception::no_error) [[unlikely]]
            return err;
        if (pack.size > modbus_base::max_read_bits) [[unlikely]]
            return exception::illegal_data_value;
        const std::uint8_t i_max = pack.size * 8;
        for (std::size_t i{}; (i < values.size()) && (i < i_max); i++) {
            const std::uint8_t i_bits = i / 8;
            const std::uint8_t ii     = 1 << (i % 8);
            values[i]                 = pack.data[i_bits] & ii;
        }
        callback_(exception::no_error, values.begin(), values.begin() + i_max);
        return exception::no_error;
    }

    ~read_bits() noexcept override{};

private:
    std::size_t               size_;
    types::callback_bits_type callback_;
};

class read_input_bits : public modbus_command {
public:
    read_input_bits(std::uint8_t slave, std::uint16_t address, std::size_t size) noexcept
        : read_input_bits{slave, address, size, nullptr}
    {}

    read_input_bits(std::uint8_t slave, std::uint16_t address, std::size_t size,
                    types::callback_bits_type callback) noexcept
        : size_{size}, callback_{std::move(callback)}, modbus_command{slave, address}
    {
        const std::uint32_t max_address = address + size_ - 1;
        if (max_address > static_cast<std::uint32_t>(std::numeric_limits<std::uint16_t>::max()))
            [[unlikely]] {
            error(exception::illegal_data_address);
            return;
        }
        if (!msg().template serialize<header, request_fields_read, std::uint8_t, crc16ansi>(
                {{slave, static_cast<std::uint8_t>(function::read_discrete_inputs)},
                 {address, static_cast<std::uint16_t>(size_)},
                 0,
                 nullptr})) {
            error(exception::illegal_data_address);
            return;
        }
    }

    std::shared_ptr<modbus_command>
    clone(command_vault_pointer vault = nullptr) noexcept override
    {
        if (vault != nullptr) {
            return std::shared_ptr<modbus_command>{new (vault) read_input_bits(*this)};
        } else {
            return std::make_shared<read_input_bits>(*this);
        }
    }

    exception
    receive(const msg_type& message) noexcept override
    {
        static types::bits_array_type values{};
        auto [pack, err] = input_msg<header, std::uint8_t, std::uint8_t>(slave(), message);
        if (error(err) != exception::no_error) [[unlikely]]
            return err;
        if (pack.size > modbus_base::max_read_bits) [[unlikely]]
            return exception::illegal_data_value;
        const std::size_t i_max = pack.size * 8;
        for (std::size_t i{}; (i < values.size()) && (i < i_max); i++) {
            const std::uint8_t i_bits = i / 8;
            const std::uint8_t ii     = 1 << (i % 8);
            values[i]                 = pack.data[i_bits] & ii;
        }
        callback_(exception::no_error, values.begin(), values.begin() + i_max);
        return exception::no_error;
    }

    ~read_input_bits() noexcept override = default;

private:
    std::size_t               size_;
    types::callback_bits_type callback_;
};

class write_bit : public modbus_command {
public:
    write_bit(std::uint8_t slave, std::uint16_t address, bool val) noexcept
        : write_bit{slave, address, val, nullptr}
    {}

    write_bit(std::uint8_t slave, std::uint16_t address, bool val,
              types::callback_function_type callback) noexcept
        : val_{val}, callback_{std::move(callback)}, modbus_command{slave, address}
    {
        const std::uint32_t max_address = address;
        if (max_address > static_cast<std::uint32_t>(std::numeric_limits<std::uint16_t>::max()))
            [[unlikely]] {
            error(exception::illegal_data_address);
            return;
        }
        if (!msg().template serialize<header, request_fields_read, std::uint8_t, crc16ansi>(
                {{slave, static_cast<uint8_t>(function::write_single_coil)},
                 {address, (val) ? (modbus_base::on_coil_value) : (modbus_base::off_coil_value)},
                 0,
                 nullptr})) {
            error(exception::illegal_data_address);
            return;
        }
    }

    std::shared_ptr<modbus_command>
    clone(command_vault_pointer vault = nullptr) noexcept override
    {
        if (vault != nullptr) {
            return std::shared_ptr<modbus_command>{new (vault) write_bit(*this)};
        } else {
            return std::make_shared<write_bit>(*this);
        }
    }

    exception
    receive(const msg_type& message) noexcept override
    {
        auto [pack, err] = input_msg<header, std::uint8_t, std::uint8_t>(slave(), message);
        if (error(err) != exception::no_error) [[unlikely]]
            return err;
        callback_(exception::no_error);
        return exception::no_error;
    }

    ~write_bit() noexcept override = default;

private:
    bool                          val_;
    types::callback_function_type callback_;
};

class write_bits : public modbus_command {
public:
    template <std::size_t Size>
    write_bits(std::uint8_t slave, std::uint16_t address,
               const std::array<bool, Size>& vals) noexcept
        : write_bits{slave, address, vals, nullptr}
    {}

    template <std::size_t Size>
    write_bits(std::uint8_t slave, std::uint16_t address, const std::array<bool, Size>& vals,
               types::callback_function_type callback) noexcept
        : callback_{std::move(callback)}, modbus_command{slave, address}
    {
        static_assert(Size < modbus_base::max_write_bits, "Too much to write!");
        const std::uint32_t max_address = address + Size - 1;
        if (max_address > static_cast<std::uint32_t>(std::numeric_limits<std::uint16_t>::max()))
            [[unlikely]] {
            error(exception::illegal_data_address);
            return;
        }
        constexpr std::uint16_t coils_collect_num{
            static_cast<std::uint16_t>((Size % 8) ? (Size / 8 + 1) : (Size / 8))};
        std::array<std::uint8_t, coils_collect_num> coils_collect{};
        for (std::uint16_t i = 0; (i < modbus_base::max_read_bits) && (i < coils_collect_num);
             i++) {
            for (std::uint16_t j = 0;
                 (j < 8) && (static_cast<std::size_t>(i * 8 + j) < vals.size()); j++) {
                if (vals[i * 8 + j]) {
                    coils_collect[i] |= 1 << j;
                }
            }
        }
        if (!msg().template serialize<header, request_fields_wr_single, std::uint8_t, crc16ansi>(
                {{slave, static_cast<std::uint8_t>(function::write_multiple_coils)},
                 {address, static_cast<std::uint16_t>(vals.size()),
                  static_cast<std::uint8_t>(coils_collect_num)},
                 static_cast<std::uint8_t>(coils_collect_num),
                 coils_collect.data()})) {
            error(exception::illegal_data_address);
            return;
        }
    }

    std::shared_ptr<modbus_command>
    clone(command_vault_pointer vault) noexcept override
    {
        if (vault != nullptr) {
            return std::shared_ptr<modbus_command>{new (vault) write_bits(*this)};
        } else {
            return std::make_shared<write_bits>(*this);
        }
    }

    exception
    receive(const msg_type& message) noexcept override
    {
        auto [pack, err] = input_msg<header, std::uint8_t, std::uint8_t>(slave(), message);
        if (error(err) != exception::no_error) [[unlikely]]
            return err;
        callback_(exception::no_error);
        return exception::no_error;
    }

    ~write_bits() noexcept override = default;

private:
    types::callback_function_type callback_;
};

}    // namespace xitren::components::modbus
