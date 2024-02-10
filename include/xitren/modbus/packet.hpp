#pragma once

#include <xitren/circular_buffer.hpp>
#include <xitren/func/data.hpp>
#include <xitren/modbus/crc16ansi.hpp>

#include <concepts>
#include <cstdint>

namespace xitren::modbus {

template <typename Header, typename Fields, crc::crc_concept Crc>
union packet {
    using size_type                   = std::size_t;
    static constexpr size_type length = (sizeof(Header) + sizeof(Fields) + sizeof(typename Crc::value_type));
    // clang-format off
    using struct_type                 = struct __attribute__((__packed__)) {
        Header                   header;
        Fields                   fields;
        typename Crc::value_type crc;
    };
    // clang-format on
    using struct_nocrc_type = struct __attribute__((__packed__)) {
        Header header;
        Fields fields;
    };
    using array_type = std::array<std::uint8_t, sizeof(struct_type)>;

public:
    explicit constexpr packet(std::array<uint8_t, length> const&& array) noexcept : pure_{std::move(array)} {}

    template <std::input_iterator InputIterator>
    explicit constexpr packet(InputIterator begin) noexcept : pure_{}
    {
        std::copy(begin, begin + length, pure_.begin());
    }

    constexpr packet() noexcept : pure_{} {}

    constexpr packet(Header const& header, Fields const& fields) noexcept : fields_{header, fields, {}}
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
        return (fields_.crc == Crc::calculate(pure_.begin(), pure_.end() - sizeof(typename Crc::value_type)));
    }

    constexpr array_type
    to_array() const noexcept
    {
        return pure_;
    }

    static consteval array_type
    serialize(Header const& header, Fields const& fields) noexcept
    {
        auto data_tr = func::data<struct_nocrc_type>::serialize({header, fields});
        auto crc     = Crc::calculate(data_tr.begin(), data_tr.end());
        return func::data<struct_type>::serialize({header, fields, crc});
    }

    template <std::size_t Size>
    static consteval std::tuple<bool, Header, Fields>
    deserialize(std::array<std::uint8_t, Size> const& array) noexcept
    {
        auto [header, fields, crc] = func::data<struct_type>::serialize(array);
        std::array<std::uint8_t, Size - sizeof(Crc::value_type)> checker;
        auto                                                     calc_crc = Crc::calculate(checker);
        return {crc == calc_crc, header, fields};
    }

    template <typename InputIterator>
    static constexpr packet
    deserialize(InputIterator begin) noexcept
    {
        func::data<packet> tt{};
        std::copy(begin, begin + length, tt.pure.begin());
        return tt.fields;
    }

    template <std::output_iterator<std::uint8_t> InputIterator>
    static constexpr void
    serialize(packet const& type, InputIterator begin) noexcept
    {
        func::data<packet> tt{type};
        std::copy(tt.pure.begin(), tt.pure.end(), begin);
    }

private:
    struct_type fields_;
    array_type  pure_;
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
        size_type   size;
        Type const* data;
    };

    template <typename Header, typename Fields, typename Type>
    struct __attribute__((__packed__)) fields_out_ptr {
        Header const* const header;
        Fields const* const fields;
        size_type           size;
        Type const*         data;
    };

    template <typename Header, typename Fields, typename Type>
    struct __attribute__((__packed__)) fields_in {
        Header      header;
        Fields      fields;
        size_type   size;
        Type const* data;
    };

    template <typename Header, typename Fields, typename Type, crc::crc_concept Crc>
    auto
    deserialize_no_check() const noexcept
    {
        using return_type          = fields_out_ptr<Header, Fields, Type>;
        constexpr size_type length = (sizeof(Header) + sizeof(Fields) + sizeof(typename Crc::value_type));
        static_assert(sizeof(Type) != 0);
        static_assert(Max >= length);
        size_type const variable_part = (size_ - length) / sizeof(Type);
        auto            header_conv   = reinterpret_cast<Header const*>(storage_.begin());
        auto            fields_conv   = reinterpret_cast<Fields const*>(storage_.begin() + sizeof(Header));
        auto            data_conv = reinterpret_cast<Type const*>(storage_.begin() + sizeof(Header) + sizeof(Fields));
        return return_type{header_conv, fields_conv, variable_part, data_conv};
    }

    template <typename Header, typename Fields, typename Type, crc::crc_concept Crc>
    constexpr auto
    deserialize() const
    {
        using return_type          = fields_out<Header, Fields, Type>;
        constexpr size_type length = (sizeof(Header) + sizeof(Fields) + sizeof(typename Crc::value_type));
        static_assert(sizeof(Type) != 0);
        static_assert(Max >= length);
        size_type const variable_part = (size_ - length) / sizeof(Type);
        if (((size_ - length) % sizeof(Type))) {
            return return_type{{}, {}, false, 0, nullptr};
        }
        auto                     header_conv = func::data<Header>::deserialize(storage_.begin());
        auto                     fields_conv = func::data<Fields>::deserialize(storage_.begin() + sizeof(Header));
        auto                     crc_conv = func::data<typename Crc::value_type>::deserialize(storage_.begin() + size_
                                                                                              - sizeof(typename Crc::value_type));
        typename Crc::value_type crc_calc
            = Crc::calculate(storage_.begin(), storage_.begin() + size_ - sizeof(typename Crc::value_type));
        return return_type{header_conv, fields_conv, crc_conv.get() == crc_calc.get(), variable_part,
                           reinterpret_cast<Type const*>(storage_.begin() + sizeof(Header) + sizeof(Fields))};
    }

    template <typename Header, typename Fields, typename Type, crc::crc_concept Crc>
    constexpr bool
    serialize(fields_in<Header, Fields, Type> const& input)
    {
        constexpr size_type length = (sizeof(Header) + sizeof(Fields) + sizeof(typename Crc::value_type));
        static_assert(sizeof(Type) != 0);
        static_assert(Max >= length);
        if ((input.size * sizeof(Type) + length) > Max) {
            return false;
        }
        func::data<Header>::serialize(input.header, storage_.begin());
        func::data<Fields>::serialize(input.fields, storage_.begin() + sizeof(Header));
        if ((input.size > 0) && (input.data != nullptr)) {
            std::copy(reinterpret_cast<uint8_t const*>(input.data),
                      reinterpret_cast<uint8_t const*>(input.data + input.size),
                      reinterpret_cast<uint8_t*>(storage_.begin() + sizeof(Header) + sizeof(Fields)));
        }
        auto const crc_ptr = storage_.begin() + sizeof(Header) + sizeof(Fields) + input.size * sizeof(Type);
        typename Crc::value_type const crc{Crc::calculate(storage_.begin(), crc_ptr)};
        func::data<typename Crc::value_type>::serialize(crc, crc_ptr);
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

    inline array_type const&
    storage() const noexcept
    {
        return storage_;
    }

private:
    array_type storage_{};
    size_type  size_{};
};

}    // namespace xitren::modbus
