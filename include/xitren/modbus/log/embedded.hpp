#pragma once

#include <xitren/circular_buffer.hpp>

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

class log_helper {
    using name_type     = std::string_view const;
    using ptr_item_type = char const*;
    using log_type      = containers::circular_buffer<std::uint8_t, log_size>;
    using temp_type     = std::array<char, 10>;

public:
    template <typename... T>
    log_helper(int lvl, [[maybe_unused]] char const* fmt, [[maybe_unused]] T&&... t) : lvl_{lvl}, silent_{true}
    {
        if (lvl_ >= current_lvl) {
            silent_ = true;
        }
        operator<<(fmt);
    }

    log_helper(int lvl) : lvl_{lvl}, silent_{false}
    {
        if (lvl_ >= current_lvl) {
            silent_ = true;
        }
    }

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

    template <typename Type>
    inline log_helper&
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

    inline log_helper&
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

    ~log_helper() = default;

    static inline void
    set_current_lvl(int val)
    {
        current_lvl = val;
    }

    static inline auto
    get_current_lvl()
    {
        return current_lvl;
    }

    static inline void
    register_sink(log_type& log_sink)
    {
        sink = &log_sink;
    }

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

#define LEVEL(LVL) xitren::modbus::log::log_helper::set_current_lvl(LVL)
#define GET_LEVEL() xitren::modbus::log::log_helper::get_current_lvl()

#if LOG_LEVEL <= LOG_LEVEL_TRACE
#    define TRACE(...)                                  \
        xitren::modbus::log::log_helper                 \
        {                                               \
            LOG_LEVEL_TRACE, __VA_OPT__(, ) __VA_ARGS__ \
        }
#else
#    define TRACE(...)                                     \
        if constexpr (true) {                              \
        } else                                             \
            xitren::modbus::log::log_helper                \
            {                                              \
                LOG_LEVEL_TRACE __VA_OPT__(, ) __VA_ARGS__ \
            }
#endif

#if LOG_LEVEL <= LOG_LEVEL_TRACE
#    define TRACE(...)                                  \
        xitren::modbus::log::log_helper                 \
        {                                               \
            LOG_LEVEL_TRACE, __VA_OPT__(, ) __VA_ARGS__ \
        }
#else
#    define TRACE(...)                                     \
        if constexpr (true) {                              \
        } else                                             \
            xitren::modbus::log::log_helper                \
            {                                              \
                LOG_LEVEL_TRACE __VA_OPT__(, ) __VA_ARGS__ \
            }
#endif

#if LOG_LEVEL <= LOG_LEVEL_DEBUG
#    define DEBUG(...)                                 \
        xitren::modbus::log::log_helper                \
        {                                              \
            LOG_LEVEL_DEBUG __VA_OPT__(, ) __VA_ARGS__ \
        }
#else
#    define DEBUG(...)                                     \
        if constexpr (true) {                              \
        } else                                             \
            xitren::modbus::log::log_helper                \
            {                                              \
                LOG_LEVEL_DEBUG __VA_OPT__(, ) __VA_ARGS__ \
            }
#endif

#if LOG_LEVEL <= LOG_LEVEL_INFO
#    define INFO(...)                                 \
        xitren::modbus::log::log_helper               \
        {                                             \
            LOG_LEVEL_INFO __VA_OPT__(, ) __VA_ARGS__ \
        }
#else
#    define INFO(...)                                     \
        if constexpr (true) {                             \
        } else                                            \
            xitren::modbus::log::log_helper               \
            {                                             \
                LOG_LEVEL_INFO __VA_OPT__(, ) __VA_ARGS__ \
            }
#endif

#if LOG_LEVEL <= LOG_LEVEL_WARN
#    define WARN(...)                                 \
        xitren::modbus::log::log_helper               \
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
        xitren::modbus::log::log_helper                \
        {                                              \
            LOG_LEVEL_ERROR __VA_OPT__(, ) __VA_ARGS__ \
        }
#else
#    define ERROR(...)                                     \
        if constexpr (true) {                              \
        } else                                             \
            xitren::modbus::log::log_helper                \
            {                                              \
                LOG_LEVEL_ERROR __VA_OPT__(, ) __VA_ARGS__ \
            }
#endif

#if LOG_LEVEL <= LOG_LEVEL_CRITICAL
#    define CRITICAL(...)                                 \
        xitren::modbus::log::log_helper                   \
        {                                                 \
            LOG_LEVEL_CRITICAL __VA_OPT__(, ) __VA_ARGS__ \
        }
#else
#    define CRITICAL(...)                                     \
        if constexpr (true) {                                 \
        } else                                                \
            xitren::modbus::log::log_helper                   \
            {                                                 \
                LOG_LEVEL_CRITICAL __VA_OPT__(, ) __VA_ARGS__ \
            }
#endif
}    // namespace xitren::modbus::log
