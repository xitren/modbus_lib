#pragma once

#include <loveka/components/modbus/crc16ansi.hpp>
#include <loveka/components/modbus/data.hpp>
#include <loveka/components/utils/circular_buffer.hpp>

#include <concepts>
#include <cstdint>

namespace loveka::components::modbus {

template <typename Header, typename Fields, crc_concept Crc>
union packet {
    using size_type = std::size_t;
    static constexpr size_type length
        = (sizeof(Header) + sizeof(Fields) + sizeof(typename Crc::value_type));

public:
    explicit constexpr packet(const std::array<uint8_t, length>&& array) noexcept
        : pure_{std::move(array)}
    {}

    template <std::input_iterator InputIterator>
    explicit constexpr packet(InputIterator begin) noexcept : pure_{}
    {
        std::copy(begin, begin + length, pure_.begin());
    }

    constexpr packet() noexcept : pure_{} {}

    constexpr packet(const Header& header, const Fields& fields) noexcept
        : fields_{header, fields, {}}
    {
        fields_.crc = Crc::calculate(pure_.begin(), pure_.end() - sizeof(typename Crc::value_type));
    }

    Header&
    header() noexcept
    {
        return fields_.header;
    }

    constexpr Header&
    header() const noexcept
    {
        return fields_.header;
    }

    constexpr Fields&
    fields() const noexcept
    {
        return fields_.fields;
    }

    Fields&
    fields() noexcept
    {
        return fields_.fields;
    }

    Crc&
    crc() noexcept
    {
        return fields_.crc;
    }

    constexpr Crc&
    crc() const noexcept
    {
        return fields_.crc;
    }

    constexpr bool
    valid() noexcept
    {
        return (fields_.crc
                == Crc::calculate(pure_.begin(), pure_.end() - sizeof(typename Crc::value_type)));
    }

    constexpr std::array<std::uint8_t, length>
    to_array() const noexcept
    {
        return pure_;
    }

    template <typename InputIterator>
    static constexpr packet
    deserialize(InputIterator begin) noexcept
    {
        data<packet> tt{};
        std::copy(begin, begin + length, tt.pure.begin());
        return tt.fields;
    }

    static constexpr std::array<std::uint8_t, length>
    serialize(const packet& fields) noexcept
    {
        data<packet> tt{fields};
        return tt.pure;
    }

    template <std::output_iterator<std::uint8_t> InputIterator>
    static constexpr void
    serialize(const packet& type, InputIterator begin) noexcept
    {
        data<packet> tt{type};
        std::copy(tt.pure.begin(), tt.pure.end(), begin);
    }

private:
    struct __attribute__((__packed__)) tag_fields {
        Header                   header;
        Fields                   fields;
        typename Crc::value_type crc;
    } fields_;
    std::array<std::uint8_t, sizeof(fields_)> pure_;
};

template <std::size_t Max>
class packet_accessor {
    using size_type = std::size_t;

public:
    using array_type = std::array<std::uint8_t, Max>;

    template <typename Header, typename Fields, typename Type>
    struct __attribute__((__packed__)) fields_out {
        Header      header;
        Fields      fields;
        bool        valid;
        size_t      size;
        const Type* data;
    };

    template <typename Header, typename Fields, typename Type>
    struct __attribute__((__packed__)) fields_in {
        Header      header;
        Fields      fields;
        size_t      size;
        const Type* data;
    };

    template <typename Header, typename Fields, typename Type, crc_concept Crc>
    constexpr auto
    deserialize() const
    {
        using return_type = fields_out<Header, Fields, Type>;
        constexpr size_type length
            = (sizeof(Header) + sizeof(Fields) + sizeof(typename Crc::value_type));
        static_assert(sizeof(Type) > 0);
        static_assert(Max >= length);
        const size_type variable_part = (size_ - length) / sizeof(Type);
        if (((size_ - length) % sizeof(Type))) {
            return return_type{{}, {}, false, 0, nullptr};
        }
        auto header_conv = data<Header>::deserialize(storage_.begin());
        auto fields_conv = data<Fields>::deserialize(storage_.begin() + sizeof(Header));
        auto crc_conv    = data<typename Crc::value_type>::deserialize(
            storage_.begin() + size_ - sizeof(typename Crc::value_type));
        typename Crc::value_type crc_calc = Crc::calculate(
            storage_.begin(), storage_.begin() + size_ - sizeof(typename Crc::value_type));
        return return_type{
            header_conv, fields_conv, crc_conv.get() == crc_calc.get(), variable_part,
            reinterpret_cast<const Type*>(storage_.begin() + sizeof(Header) + sizeof(Fields))};
    }

    template <typename Header, typename Fields, typename Type, crc_concept Crc>
    constexpr bool
    serialize(const fields_in<Header, Fields, Type>& input)
    {
        constexpr size_type length
            = (sizeof(Header) + sizeof(Fields) + sizeof(typename Crc::value_type));
        static_assert(sizeof(Type) > 0);
        static_assert(Max >= length);
        if ((input.size * sizeof(Type) + length) > Max) {
            return false;
        }
        data<Header>::serialize(input.header, storage_.begin());
        data<Fields>::serialize(input.fields, storage_.begin() + sizeof(Header));
        if ((input.size > 0) && (input.data != nullptr)) {
            std::copy(
                reinterpret_cast<const uint8_t*>(input.data),
                reinterpret_cast<const uint8_t*>(input.data + input.size),
                reinterpret_cast<uint8_t*>(storage_.begin() + sizeof(Header) + sizeof(Fields)));
        }
        const auto crc_ptr
            = storage_.begin() + sizeof(Header) + sizeof(Fields) + input.size * sizeof(Type);
        const typename Crc::value_type crc{Crc::calculate(storage_.begin(), crc_ptr)};
        data<typename Crc::value_type>::serialize(crc, crc_ptr);
        size_ = length + input.size * sizeof(Type);
        return true;
    }

    [[nodiscard]] size_type
    size() const
    {
        return size_;
    }

    void
    size(size_type size)
    {
        size_ = size;
    }

    inline array_type&
    storage() noexcept
    {
        return storage_;
    }

    inline const array_type&
    storage() const noexcept
    {
        return storage_;
    }

private:
    array_type storage_{};
    size_type  size_{Max};
};

}    // namespace xitren::components::modbus
