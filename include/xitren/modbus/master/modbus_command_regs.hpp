#pragma once

#include "modbus_command.hpp"

#include <utility>

namespace loveka::components::modbus {

class read_registers : public modbus_command {
public:
    read_registers(std::uint8_t slave, std::uint16_t address, std::size_t size) noexcept
        : read_registers{slave, address, size, nullptr}
    {}

    read_registers(std::uint8_t slave, std::uint16_t address, std::size_t size,
                   types::callback_regs_type callback) noexcept
        : size_{size}, callback_{std::move(callback)}, modbus_command{slave, address}
    {
        const std::uint32_t max_address = address + size_ - 1;
        if (max_address > static_cast<std::uint32_t>(std::numeric_limits<std::uint16_t>::max()))
            [[unlikely]] {
            error(exception::illegal_data_address);
            return;
        }
        if (!msg().template serialize<header, request_fields_read, msb_t<std::uint16_t>, crc16ansi>(
                {{slave, static_cast<uint8_t>(function::read_holding_registers)},
                 {address, size_},
                 0,
                 nullptr})) {
            error(exception::illegal_data_address);
            return;
        }
    }

    read_registers(std::uint8_t slave, types::callback_regs_type callback, std::uint8_t* begin,
                   std::uint8_t* end) noexcept
        : size_{static_cast<size_t>(end - begin)},
          callback_{std::move(callback)},
          modbus_command{slave, 0}
    {
        msg().size(end - begin);
        for (auto it{begin}, it_n{msg().storage().begin()};
             (it != end) && (it_n != msg().storage().end()); it++, it_n++) {
            (*it_n) = (*it);
        }
    }

    std::shared_ptr<modbus_command>
    clone(command_vault_pointer vault = nullptr) noexcept override
    {
        if (vault != nullptr) {
            return std::shared_ptr<modbus_command>{new (vault) read_registers(*this)};
        } else {
            return std::make_shared<read_registers>(*this);
        }
    }

    exception
    receive(const msg_type& message) noexcept override
    {
        static std::array<std::uint16_t, modbus_base::max_read_registers> values{};
        auto [pack, err] = input_msg<header, std::uint8_t, msb_t<std::uint16_t>>(slave(), message);
        if (error(err) != exception::no_error) [[unlikely]]
            return err;
        if (pack.size > modbus_base::max_read_registers) [[unlikely]]
            return exception::illegal_data_value;
        for (std::size_t i{}; (i < values.size()) && (i < pack.size); i++) {
            values[i] = pack.data[i].get();
        }
        callback_(exception::no_error, values.begin(), values.begin() + pack.size);
        return exception::no_error;
    }

    ~read_registers() noexcept override = default;

private:
    std::size_t               size_;
    types::callback_regs_type callback_;
};

class write_register : public modbus_command {
public:
    write_register(std::uint8_t slave, std::uint16_t address, std::uint16_t val) noexcept
        : write_register{slave, address, val, nullptr}
    {}

    write_register(std::uint8_t slave, std::uint16_t address, std::uint16_t val,
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
                {{slave, static_cast<uint8_t>(function::write_single_register)},
                 {address, val},
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
            return std::shared_ptr<modbus_command>{new (vault) write_register(*this)};
        } else {
            return std::make_shared<write_register>(*this);
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

    ~write_register() noexcept override = default;

private:
    std::uint16_t                 val_;
    types::callback_function_type callback_;
};

class read_input_registers : public modbus_command {
public:
    read_input_registers(std::uint8_t slave, std::uint16_t address, std::size_t size) noexcept
        : read_input_registers{slave, address, size, nullptr}
    {}

    read_input_registers(std::uint8_t slave, std::uint16_t address, std::size_t size,
                         types::callback_regs_type callback) noexcept
        : size_{size}, callback_{std::move(callback)}, modbus_command{slave, address}
    {
        const std::uint32_t max_address = address + size_ - 1;
        if (max_address > static_cast<std::uint32_t>(std::numeric_limits<std::uint16_t>::max()))
            [[unlikely]] {
            error(exception::illegal_data_address);
            return;
        }
        if (!msg().template serialize<header, request_fields_read, msb_t<std::uint16_t>, crc16ansi>(
                {{slave, static_cast<uint8_t>(function::read_input_registers)},
                 {address, size_},
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
            return std::shared_ptr<modbus_command>{new (vault) read_input_registers(*this)};
        } else {
            return std::make_shared<read_input_registers>(*this);
        }
    }

    exception
    receive(const msg_type& message) noexcept override
    {
        static std::array<std::uint16_t, modbus_base::max_read_registers> values{};
        auto [pack, err] = input_msg<header, std::uint8_t, msb_t<std::uint16_t>>(slave(), message);
        if (error(err) != exception::no_error) [[unlikely]]
            return err;
        if (pack.size > modbus_base::max_read_registers) [[unlikely]]
            return exception::illegal_data_value;
        for (std::size_t i{}; (i < values.size()) && (i < pack.size); i++) {
            values[i] = pack.data[i].get();
        }
        callback_(exception::no_error, values.begin(), values.end());
        return exception::no_error;
    }

    ~read_input_registers() noexcept override = default;

private:
    std::size_t               size_;
    types::callback_regs_type callback_;
};

class write_registers : public modbus_command {
public:
    template <std::size_t Size>
    write_registers(std::uint8_t slave, std::uint16_t address,
                    const std::array<std::uint16_t, Size>& vals) noexcept
        : write_registers{slave, address, vals, nullptr}
    {}

    template <std::size_t Size>
    write_registers(std::uint8_t slave, std::uint16_t address,
                    const std::array<std::uint16_t, Size>& vals,
                    types::callback_function_type          callback) noexcept
        : callback_{std::move(callback)}, modbus_command{slave, address}
    {
        const std::uint32_t max_address = address + Size - 1;
        static_assert(Size < modbus_base::max_write_registers, "Too much to write!");
        constexpr std::array<msb_t<std::uint16_t>, Size> data_formatted;
        for (std::input_iterator auto it1{vals.begin()}, it2{data_formatted.begin()};
             (it1 != vals.end()) && (it2 != data_formatted.end()); it1++, it2++) {
            (*it2) = (*it1);
        }
        if (!msg()
                 .template serialize<header, request_fields_wr_single, msb_t<std::uint16_t>,
                                     crc16ansi>(
                     {{slave, static_cast<std::uint8_t>(function::write_multiple_registers)},
                      {address, static_cast<std::uint16_t>(vals.size()),
                       static_cast<std::uint8_t>(Size * 2)},
                      Size,
                      data_formatted.data()})) {
            error(exception::illegal_data_address);
            return;
        }
    }

    std::shared_ptr<modbus_command>
    clone(command_vault_pointer vault = nullptr) noexcept override
    {
        if (vault != nullptr) {
            return std::shared_ptr<modbus_command>{new (vault) write_registers(*this)};
        } else {
            return std::make_shared<write_registers>(*this);
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

    ~write_registers() noexcept override = default;

private:
    types::callback_function_type callback_;
};

}    // namespace xitren::components::modbus
