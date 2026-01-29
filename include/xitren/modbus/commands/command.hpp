#pragma once

#include <xitren/circular_buffer.hpp>
#include <xitren/modbus/modbus.hpp>

#include <functional>
#include <memory>
#include <optional>
#include <ranges>
#include <utility>
#include <variant>

namespace xitren::modbus {

/**
 * @brief Common buffer and callback types used by Modbus commands.
 *
 * This namespace groups frequently used aliases for payload buffers and
 * callbacks passed to command objects. The aliases are intentionally sized to
 * Modbus limits defined in `modbus_base`, so command implementations can rely
 * on consistent limits across the library.
 */
namespace types {
using bits_array_type        = std::array<bool, modbus_base::max_read_bits>;
using array_type             = std::array<std::uint16_t, modbus_base::max_read_registers>;
using callback_function_type = std::function<void(exception)>;
using callback_logs_type
    = std::function<void(exception, std::uint16_t address, std::uint8_t* begin, std::uint8_t* end)>;
using callback_identification_type = std::function<void(exception, std::uint8_t address, char* begin, char* end)>;
using callback_bits_type           = std::function<void(exception, bool*, bool*)>;
using callback_regs_type           = std::function<void(exception, std::uint16_t*, std::uint16_t*)>;
}    // namespace types

/**
 * @brief Base class for all Modbus master commands.
 *
 * A command encapsulates the request payload (what the master sends) and the
 * logic for parsing the corresponding response. Concrete commands implement:
 *
 * - `begin()/end()/size()` to expose the request buffer;
 * - `receive()` to parse a response into their internal state;
 * - `clone()` to allow the master to store a copy while waiting for a reply.
 *
 * The class is intentionally minimal and stateless beyond the slave ID, start
 * address, and error code, so it can be safely allocated on the stack and
 * cloned into a fixed-size vault by the master.
 *
 * Typical lifecycle:
 * 1) Construct command (e.g. read registers);
 * 2) Send via `master::run_async()` or `operator<<`;
 * 3) Receive via `master::received()` + `operator>>`;
 * 4) Inspect parsed data and error.
 */
class command {
    template <typename T, size_t Size>
    friend xitren::circular_buffer<T, Size>&
    operator<<(xitren::circular_buffer<T, Size>&, command const&);
    template <typename T, size_t Size>
    friend xitren::circular_buffer<T, Size>&
    operator>>(xitren::circular_buffer<T, Size>&, command&);

protected:
    using iterator       = std::uint8_t*;
    using const_iterator = std::uint8_t const*;

public:
    /**
     * @brief Maximum size for a command clone vault.
     *
     * The master stores a pending command in a fixed-size memory buffer to
     * avoid dynamic allocation in embedded systems. A command implementation
     * must fit into this buffer when cloned.
     */
    static constexpr std::size_t command_buffer_max = 640;
    using command_vault_type                        = std::aligned_storage_t<command_buffer_max, 1>;
    using msg_type                                  = packet_accessor<modbus_base::max_adu_length>;

    /**
     * @brief Virtual destructor.
     */
    virtual ~command() noexcept = default;

    /**
     * @brief Clones the command into a fixed-size vault.
     *
     * This is used by the master to keep a pending request while waiting
     * for a response without using dynamic allocation.
     *
     * @param vault The memory buffer to clone into.
     * @return command* Pointer to the cloned command.
     */
    virtual command*
    clone(command_vault_type&) const noexcept = 0;

    /**
     * @brief Clones the command with dynamic storage.
     *
     * This overload is convenient for user code that needs ownership
     * semantics (e.g. queuing commands).
     *
     * @return std::shared_ptr<command> Shared pointer to a cloned command.
     */
    virtual std::shared_ptr<command>
    clone() const noexcept = 0;

    /**
     * @brief Returns a mutable iterator to the start of the request buffer.
     *
     * The buffer contents are the exact bytes to be sent to the slave.
     */
    virtual inline iterator
    begin() noexcept = 0;

    /**
     * @brief Returns a read-only iterator to the start of the request buffer.
     */
    virtual inline const_iterator
    begin() const noexcept = 0;

    /**
     * @brief Returns a mutable iterator past the end of the request buffer.
     */
    virtual inline iterator
    end() noexcept = 0;

    /**
     * @brief Returns a read-only iterator past the end of the request buffer.
     */
    virtual inline const_iterator
    end() const noexcept = 0;

    /**
     * @brief Returns the size of the request buffer in bytes.
     */
    virtual inline std::size_t
    size() noexcept = 0;

    /**
     * @brief Returns the size of the request buffer in bytes.
     */
    virtual inline std::size_t
    size() const noexcept = 0;

    /**
     * @brief Receives and parses a Modbus response.
     *
     * Default implementation just clears the error state. Derived commands
     * should parse `message` and set `error_` accordingly.
     *
     * @param message The Modbus response ADU.
     * @return exception The resulting error code.
     */
    virtual exception
    receive(msg_type const&) noexcept
    {
        return error_ = exception::no_error;
    }

    /**
     * @brief Called when a response timeout occurs.
     *
     * The default behavior marks the command as failed with `bad_slave`.
     * Derived commands may override to provide custom timeout handling.
     */
    virtual void
    no_answer() noexcept
    {
        error_ = exception::bad_slave;
    }

    /**
     * @brief Returns the slave ID this command targets.
     */
    inline std::uint8_t
    slave() noexcept
    {
        return slave_;
    }

    /**
     * @brief Returns the slave ID this command targets.
     */
    [[nodiscard]] inline std::uint8_t
    slave() const noexcept
    {
        return slave_;
    }

    /**
     * @brief Returns the error code of the last operation.
     */
    inline exception
    error() noexcept
    {
        return error_;
    }

    /**
     * @brief Returns the error code of the last operation.
     */
    [[nodiscard]] inline exception
    error() const noexcept
    {
        return error_;
    }

protected:
    /**
     * @brief Constructs a Modbus command.
     *
     * @param slave The target slave ID.
     * @param address The starting register/bit address.
     */
    constexpr command(std::uint8_t slave, std::uint16_t address) noexcept : slave_{slave}, address_{address} {}

    inline std::uint16_t
    address() noexcept
    {
        return address_;
    }

    [[nodiscard]] inline std::uint16_t
    address() const noexcept
    {
        return address_;
    }

    inline exception
    error(exception err) noexcept
    {
        return error_ = err;
    }

    /**
     * @brief Helper to deserialize a Modbus response into header/fields/data.
     *
     * This validates the slave ID and the error response bit. CRC validation
     * is handled by `packet_accessor`.
     *
     * @tparam Header Modbus header type.
     * @tparam Fields Modbus fields type.
     * @tparam Type Payload element type.
     * @param slave Expected slave ID.
     * @param message The input message buffer.
     * @return Pair of deserialized view and error code.
     */
    template <typename Header, typename Fields, typename Type>
    inline constexpr std::pair<msg_type::fields_out_ptr<Header, Fields, Type>, exception>
    input_msg(std::uint8_t slave, msg_type const& message) const noexcept
    {
        auto pack = message.template deserialize_no_check<Header, Fields, Type, crc16ansi>();
        if (pack.header->slave_id != slave) [[unlikely]] {
            return {{}, exception::bad_slave};
        }
        if (pack.header->function_code & error_reply_mask) [[unlikely]] {
            return {{}, exception::illegal_function};
        }
        return {pack, exception::no_error};
    }

    std::uint8_t  slave_;
    std::uint16_t address_;

private:
    exception error_{exception::no_error};
};

/**
 * @brief Appends a command's raw request bytes into a circular buffer.
 *
 * This is useful for queueing outgoing requests in a FIFO transport layer.
 */
template <typename T, size_t Size>
xitren::circular_buffer<T, Size>&
operator<<(xitren::circular_buffer<T, Size>& buffer, command const& in_data)
{
    auto const begin{in_data.begin()};
    for (auto i{begin}; i != (begin + in_data.size()); i++) {
        buffer.push(*i);
    }
    return buffer;
}

/**
 * @brief Parses a command response from a circular buffer.
 *
 * The function drains as many bytes as possible from the buffer into a
 * temporary message and then calls `command::receive()` to decode it.
 */
template <typename T, size_t Size>
xitren::circular_buffer<T, Size>&
operator>>(xitren::circular_buffer<T, Size>& buffer, command& out_data)
{
    command::msg_type buff{};
    std::size_t       i{0};
    auto              it = buff.storage().begin();
    while (!buffer.empty() && (i < buff.storage().size())) {
        auto& item = buffer.front();
        *(it++)    = item;
        i++;
        buffer.pop();
    }
    buff.size(i);
    out_data.receive(buff);
    return buffer;
}

}    // namespace xitren::modbus
