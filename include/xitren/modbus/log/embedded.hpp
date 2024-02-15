#pragma once

#include <xitren/circular_buffer.hpp>

#include <cstdint>
#include <string>

#define LOG_LEVEL_TRACE 0
#define LOG_LEVEL_DEBUG 1
#define LOG_LEVEL_INFO 2
#define LOG_LEVEL_WARN 3
#define LOG_LEVEL_ERROR 4
#define LOG_LEVEL_CRITICAL 5
#define LOG_LEVEL_OFF 6

#ifndef LOG_LEVEL
#    define LOG_LEVEL LOG_LEVEL_TRACE
#endif

namespace xitren::modbus::log {

static constexpr std::uint16_t log_size = 1024;

/**
 * @brief A class for logging messages.
 *
 * This class provides a simple interface for logging messages, with support for different log levels and a flexible
 * logging backend. The logging backend can be set to any data structure that supports the `push` method, such as a
 * `std::vector`.
 *
 * The logging levels are defined as follows:
 * - `TRACE`: A detailed trace of the program flow, typically used for debugging.
 * - `DEBUG`: A more detailed log of the program flow, typically used for debugging.
 * - `INFO`: A high-level summary of the program flow.
 * - `WARN`: A warning that something unusual has occurred.
 * - `ERROR`: An error that has occurred that may cause the program to fail.
 * - `CRITICAL`: A critical error that has occurred that will cause the program to fail.
 *
 * By default, only `WARN` and above messages are logged. To enable more detailed logging, you can set the log level
 * using the `set_current_lvl` method.
 *
 * The logging macros are designed to be easy to use and flexible. The macros accept a format string and optional
 * arguments, which are formatted and logged as a single message. The format string supports standard C formatting
 * specifiers, as well as some additional specifiers specific to the logging macros. For example, you can use the
 * `{func}` specifier to insert the name of the current function into the log message.
 *
 * The logging macros also support conditional logging, where the message is only logged if the specified log level is
 * enabled. For example, the `TRACE` macro is only compiled if the log level is set to `TRACE` or higher.
 *
 * The logging macros are designed to be lightweight and fast, so they should have minimal impact on the performance of
 * the program.
 */
class embedded {
    using name_type     = std::string_view const;
    using ptr_item_type = char const*;
    using log_type      = containers::circular_buffer<std::uint8_t, log_size>;
    using temp_type     = std::array<char, 10>;

public:
    /**
     * @brief Output a formatted message to the logging sink.
     *
     * @tparam T The type of the arguments to format and log.
     * @param fmt The format string for the message.
     * @param args The arguments to format and log.
     */
    template <typename... T>
    embedded(int lvl, [[maybe_unused]] char const* fmt, [[maybe_unused]] T&&... t) : lvl_{lvl}, silent_{true}
    {
        if (lvl_ >= current_lvl) {
            silent_ = true;
        }
        operator<<(fmt);
    }

    /**
     * @brief Construct a new log helper object with the specified log level.
     *
     * @param lvl The log level of the new log helper object.
     */
    embedded(int lvl) : lvl_{lvl}, silent_{false}
    {
        if (lvl_ >= current_lvl) {
            silent_ = true;
        }
    }

    /**
     * @brief Convert an unsigned 32-bit integer to a string in decimal format.
     *
     * This function is a naive implementation that is designed for performance and readability, rather than efficiency.
     * It should only be used for small integers.
     *
     * @param value The unsigned 32-bit integer to convert.
     * @param buffer A pointer to a buffer to store the resulting string.
     */
    static void
    u32toa_naive(std::uint32_t value, temp_type::iterator buffer)
    {
        constexpr std::uint32_t radix{10};
        temp_type               temp{};
        auto*                   p = temp.begin();
        do {
            *(p++) = static_cast<char>(value % radix) + '0';
            value /= radix;
        } while (value > 0);

        auto* b = buffer;
        do {
            *(b++) = *(--p);
        } while (p != temp.begin());

        *b = '\0';
    }

    /**
     * @brief Convert a signed 32-bit integer to a string in decimal format.
     *
     * This function is a naive implementation that is designed for performance and readability, rather than efficiency.
     * It should only be used for small integers.
     *
     * @param value The signed 32-bit integer to convert.
     * @param buffer A pointer to a buffer to store the resulting string.
     */
    static void
    i32toa_naive(std::int32_t value, temp_type::iterator buffer)
    {
        auto u = static_cast<std::uint32_t>(value);
        if (value < 0) {
            *(buffer++) = '-';
            u           = ~u + 1;
        }
        u32toa_naive(u, buffer);
    }

    /**
     * @brief Output a formatted message to the logging sink.
     *
     * @tparam T The type of the arguments to format and log.
     * @param v The value to format and log.
     */
    template <typename Type>
    inline embedded&
    operator<<(Type const& v) noexcept
    {
        if (!sink || silent_) {
            return *this;
        }
        if constexpr (std::same_as<Type, int>) {
            temp_type temp{};
            i32toa_naive(v, temp.begin());
            operator<<(temp.data());
        } else {
            static_assert("Not defined for this type!");
        }
        return *this;
    }

    /**
     * @brief Output a string to the logging sink.
     *
     * @param s The string to output.
     */
    inline embedded&
    operator<<(ptr_item_type s) noexcept
    {
        if (!sink || silent_) {
            return *this;
        }
        for (auto i{0}; (i < log_size) && (s != 0); i++, s++) {
            sink->push(*s);
        }
        return *this;
    }

    /**
     * @brief Destruct the log helper object.
     */
    ~embedded() = default;

    /**
     * @brief Set the current log level.
     *
     * @param val The new log level.
     */
    static inline void
    set_current_lvl(int val)
    {
        current_lvl = val;
    }

    /**
     * @brief Get the current log level.
     *
     * @return int The current log level.
     */
    static inline auto
    get_current_lvl()
    {
        return current_lvl;
    }

    /**
     * @brief Register a new logging sink.
     *
     * @param log
    static inline void
    register_sink(log_type& log_sink)
    {
        sink = &log_sink;
    }

    /**
     * @brief Unregister the current logging sink.
     */
    static inline void
    unregister_sink()
    {
        sink = nullptr;
    }

private:
    int                     lvl_;
    bool                    silent_;
    static inline log_type* sink{nullptr};
#ifdef DEBUG
    static inline int current_lvl{LOG_LEVEL_INFO};
#else
    static inline int current_lvl{LOG_LEVEL_WARN};
#endif
};

#define LEVEL(LVL) xitren::modbus::log::embedded::set_current_lvl(LVL)
#define GET_LEVEL() xitren::modbus::log::embedded::get_current_lvl()

#if LOG_LEVEL <= LOG_LEVEL_TRACE
#    define TRACE(...)                                  \
        xitren::modbus::log::embedded                   \
        {                                               \
            LOG_LEVEL_TRACE, __VA_OPT__(, ) __VA_ARGS__ \
        }
#else
#    define TRACE(...)                                     \
        if constexpr (true) {                              \
        } else                                             \
            xitren::modbus::log::embedded                  \
            {                                              \
                LOG_LEVEL_TRACE __VA_OPT__(, ) __VA_ARGS__ \
            }
#endif

#if LOG_LEVEL <= LOG_LEVEL_TRACE
#    define TRACE(...)                                  \
        xitren::modbus::log::embedded                   \
        {                                               \
            LOG_LEVEL_TRACE, __VA_OPT__(, ) __VA_ARGS__ \
        }
#else
#    define TRACE(...)                                     \
        if constexpr (true) {                              \
        } else                                             \
            xitren::modbus::log::embedded                  \
            {                                              \
                LOG_LEVEL_TRACE __VA_OPT__(, ) __VA_ARGS__ \
            }
#endif

#if LOG_LEVEL <= LOG_LEVEL_DEBUG
#    define DEBUG(...)                                 \
        xitren::modbus::log::embedded                  \
        {                                              \
            LOG_LEVEL_DEBUG __VA_OPT__(, ) __VA_ARGS__ \
        }
#else
#    define DEBUG(...)                                     \
        if constexpr (true) {                              \
        } else                                             \
            xitren::modbus::log::embedded                  \
            {                                              \
                LOG_LEVEL_DEBUG __VA_OPT__(, ) __VA_ARGS__ \
            }
#endif

#if LOG_LEVEL <= LOG_LEVEL_INFO
#    define INFO(...)                                 \
        xitren::modbus::log::embedded                 \
        {                                             \
            LOG_LEVEL_INFO __VA_OPT__(, ) __VA_ARGS__ \
        }
#else
#    define INFO(...)                                     \
        if constexpr (true) {                             \
        } else                                            \
            xitren::modbus::log::embedded                 \
            {                                             \
                LOG_LEVEL_INFO __VA_OPT__(, ) __VA_ARGS__ \
            }
#endif

#if LOG_LEVEL <= LOG_LEVEL_WARN
#    define WARN(...)                                 \
        xitren::modbus::log::embedded                 \
        {                                             \
            LOG_LEVEL_WARN __VA_OPT__(, ) __VA_ARGS__ \
        }
#else
#    define WARN(...)                                     \
        if constexpr (true) {                             \
        } else                                            \
            xitren::modbus::log::log_helper               \
            {                                             \
                LOG_LEVEL_WARN __VA_OPT__(, ) __VA_ARGS__ \
            }
#endif

#if LOG_LEVEL <= LOG_LEVEL_ERROR
#    define ERROR(...)                                 \
        xitren::modbus::log::embedded                  \
        {                                              \
            LOG_LEVEL_ERROR __VA_OPT__(, ) __VA_ARGS__ \
        }
#else
#    define ERROR(...)                                     \
        if constexpr (true) {                              \
        } else                                             \
            xitren::modbus::log::embedded                  \
            {                                              \
                LOG_LEVEL_ERROR __VA_OPT__(, ) __VA_ARGS__ \
            }
#endif

#if LOG_LEVEL <= LOG_LEVEL_CRITICAL
#    define CRITICAL(...)                                 \
        xitren::modbus::log::embedded                     \
        {                                                 \
            LOG_LEVEL_CRITICAL __VA_OPT__(, ) __VA_ARGS__ \
        }
#else
#    define CRITICAL(...)                                     \
        if constexpr (true) {                                 \
        } else                                                \
            xitren::modbus::log::embedded                     \
            {                                                 \
                LOG_LEVEL_CRITICAL __VA_OPT__(, ) __VA_ARGS__ \
            }
#endif
}    // namespace xitren::modbus::log
