#pragma once

#include <array>
#include <concepts>
#include <cstdint>

namespace loveka::components::modbus {

static consteval bool
is_lsb()
{
    std::uint16_t               test{0x3322};
    std::array<std::uint8_t, 2> arr{};
    std::copy(&test, (&test) + 1, arr.begin());
    return arr[0] == 0x22;
}

template <class Type>
union data {
    Type                                     fields;
    std::array<std::uint8_t, sizeof(fields)> pure;

    template <std::input_iterator InputIterator>
    static constexpr Type
    deserialize(InputIterator begin) noexcept
    {
        data<Type> tt{};
        std::copy(begin, begin + sizeof(fields), tt.pure.begin());
        return tt.fields;
    }

    static constexpr std::array<std::uint8_t, sizeof(fields)>
    serialize(const Type& fields) noexcept
    {
        data<Type> tt{fields};
        return tt.pure;
    }

    template <std::input_iterator InputIterator>
    static constexpr void
    serialize(const Type& type, InputIterator begin) noexcept
    {
        data<Type> tt{type};
        std::copy(tt.pure.begin(), tt.pure.end(), begin);
    }
};

static constexpr std::uint16_t
swap(std::uint16_t val) noexcept
{
    data<std::uint16_t> l{val};
    std::swap(l.pure[0], l.pure[1]);
    return l.fields;
}

static constexpr std::uint32_t
swap(std::uint32_t val) noexcept
{
    data<std::uint32_t> l{val};
    std::swap(l.pure[0], l.pure[3]);
    std::swap(l.pure[1], l.pure[2]);
    return l.fields;
}

static constexpr std::uint64_t
swap(std::uint64_t val) noexcept
{
    data<std::uint64_t> l{val};
    std::swap(l.pure[0], l.pure[7]);
    std::swap(l.pure[1], l.pure[6]);
    std::swap(l.pure[2], l.pure[5]);
    std::swap(l.pure[3], l.pure[4]);
    return l.fields;
}

template <class T>
concept swappable
    = std::same_as<std::uint16_t,
                   T> || std::same_as<std::uint32_t, T> || std::same_as<std::uint64_t, T>;

template <swappable T>
class __attribute__((__packed__)) lsb_t {
public:
    using data_type = T;

    constexpr lsb_t() = default;
    constexpr lsb_t(const data_type& value)
    {
        if constexpr (is_lsb()) {
            value_ = value;
        } else {
            value_ = swap(value);
        }
    }
    constexpr lsb_t(const data_type&& value)
    {
        if constexpr (is_lsb()) {
            value_ = value;
        } else {
            value_ = swap(value);
        }
    }

    [[nodiscard]] constexpr data_type
    get() const
    {
        if constexpr (is_lsb()) {
            return value_;
        } else {
            return swap(value_);
        }
    }

    constexpr lsb_t&
    operator=(data_type value)
    {
        if constexpr (is_lsb()) {
            value_ = value;
        } else {
            value_ = swap(value);
        }
        return *this;
    }

    constexpr bool
    operator==(const lsb_t& other)
    {
        return this->get() == other.get();
    }

    constexpr bool
    operator<(const lsb_t& other)
    {
        return this->get() < other.get();
    }

    constexpr bool
    operator>(const lsb_t& other)
    {
        return this->get() > other.get();
    }

    constexpr bool
    operator<=(const lsb_t& other)
    {
        return this->get() <= other.get();
    }

    constexpr bool
    operator>=(const lsb_t& other)
    {
        return this->get() >= other.get();
    }

private:
    data_type value_{};
};

template <swappable T>
class __attribute__((__packed__)) msb_t {
public:
    using data_type = T;

    constexpr msb_t() = default;
    constexpr msb_t(const data_type& value)
    {
        if constexpr (is_lsb()) {
            value_ = swap(value);
        } else {
            value_ = value;
        }
    }
    constexpr msb_t(const data_type&& value)
    {
        if constexpr (is_lsb()) {
            value_ = swap(value);
        } else {
            value_ = value;
        }
    }

    [[nodiscard]] constexpr data_type
    get() const
    {
        if constexpr (is_lsb()) {
            return swap(value_);
        } else {
            return value_;
        }
    }

    constexpr msb_t&
    operator=(data_type value)
    {
        if constexpr (is_lsb()) {
            value_ = swap(value);
        } else {
            value_ = value;
        }
        return *this;
    }

    constexpr bool
    operator==(const msb_t& other)
    {
        return this->get() == other.get();
    }

    constexpr bool
    operator<(const msb_t& other)
    {
        return this->get() < other.get();
    }

    constexpr bool
    operator>(const msb_t& other)
    {
        return this->get() > other.get();
    }

    constexpr bool
    operator<=(const msb_t& other)
    {
        return this->get() <= other.get();
    }

    constexpr bool
    operator>=(const msb_t& other)
    {
        return this->get() >= other.get();
    }

private:
    data_type value_{};
};

}    // namespace xitren::components::modbus
