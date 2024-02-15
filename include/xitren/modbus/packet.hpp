/*!
_ _
__ _(_) |_ _ _ ___ _ _
\ \ / |  _| '_/ -_) ' \
/_\_\_|\__|_| \___|_||_|
* @date 15.02.2024
*/
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
        /**
         * The packet header.
         */
        Header                   header;
        /**
         * The packet fields.
         */
        Fields                   fields;
        /**
         * The packet CRC.
         */
        typename Crc::value_type crc;
    };
    // clang-format on
    using struct_nocrc_type = struct __attribute__((__packed__)) {
        /**
         * The packet header.
         */
        Header header;
        /**
         * The packet fields.
         */
        Fields fields;
    };
    using array_type = std::array<std::uint8_t, sizeof(struct_type)>;

public:
    /**
     * Constructs a packet from an array of bytes.
     *
     * @param array The array of bytes to construct the packet from.
     */
    explicit constexpr packet(std::array<uint8_t, length> const&& array) noexcept : pure_{std::move(array)} {}

    /**
     * Constructs a packet from an input iterator.
     *
     * @param begin The input iterator pointing to the first byte of the packet.
     */
    template <std::input_iterator InputIterator>
    explicit constexpr packet(InputIterator begin) noexcept : pure_{}
    {
        std::copy(begin, begin + length, pure_.begin());
    }

    /**
     * Default constructor.
     */
    constexpr packet() noexcept : pure_{} {}

    /**
     * Constructs a packet from a header and fields.
     *
     * @param header The packet header.
     * @param fields The packet fields.
     */
    constexpr packet(Header const& header, Fields const& fields) noexcept : fields_{header, fields, {}}
    {
        fields_.crc = Crc::calculate(pure_.begin(), pure_.end() - sizeof(typename Crc::value_type));
    }

    /**
     * Returns a reference to the packet header.
     *
     * @return A reference to the packet header.
     */
    Header&
    header() noexcept
    {
        return fields_.header;
    }

    /**
     * Returns a constant reference to the packet header.
     *
     * @return A constant reference to the packet header.
     */
    constexpr Header&
    header() const noexcept
    {
        return fields_.header;
    }

    /**
     * Returns a constant reference to the packet fields.
     *
     * @return A constant reference to the packet fields.
     */
    constexpr Fields&
    fields() const noexcept
    {
        return fields_.fields;
    }

    /**
     * Returns a reference to the packet fields.
     *
     * @return A reference to the packet fields.
     */
    Fields&
    fields() noexcept
    {
        return fields_.fields;
    }

    /**
     * Returns a reference to the packet CRC.
     *
     * @return A reference to the packet CRC.
     */
    Crc&
    crc() noexcept
    {
        return fields_.crc;
    }

    /**
     * Returns a constant reference to the packet CRC.
     *
     * @return A constant reference to the packet CRC.
     */
    constexpr Crc&
    crc() const noexcept
    {
        return fields_.crc;
    }

    /**
     * Checks if the packet is valid.
     *
     * @return `true` if the packet is valid, `false` otherwise.
     */
    constexpr bool
    valid() noexcept
    {
        return (fields_.crc == Crc::calculate(pure_.begin(), pure_.end() - sizeof(typename Crc::value_type)));
    }

    /**
     * Returns the packet as an array of bytes.
     *
     * @return The packet as an array of bytes.
     */
    constexpr array_type
    to_array() const noexcept
    {
        return pure_;
    }

    /**
     * Serializes a packet into an array of bytes.
     *
     * @param header The packet header.
     * @param fields The packet fields.
     * @return The serialized packet as an array of bytes.
     */
    static constexpr array_type
    serialize(Header const& header, Fields const& fields) noexcept
    {
        auto data_tr = func::data<struct_nocrc_type>::serialize({header, fields});
        auto crc     = Crc::calculate(data_tr.begin(), data_tr.end());
        return func::data<struct_type>::serialize({header, fields, crc});
    }

    /**
     * Deserializes a packet from an array of bytes.
     *
     * @param array The array of bytes to deserialize the packet from.
     * @return A tuple containing the packet validity, header, and fields.
     */
    template <std::size_t Size>
    static constexpr std::tuple<bool, Header, Fields>
    deserialize(std::array<std::uint8_t, Size> const& array) noexcept
    {
        auto [header, fields, crc] = func::data<struct_type>::serialize(array);
        std::array<std::uint8_t, Size - sizeof(Crc::value_type)> checker;
        auto                                                     calc_crc = Crc::calculate(checker);
        return {crc == calc_crc, header, fields};
    }

    /**
     * Deserializes a packet from an input iterator.
     *
     * @param begin The input iterator pointing to the first byte of the packet.
     * @return The deserialized packet.
     */
    template <typename InputIterator>
    static constexpr packet
    deserialize(InputIterator begin) noexcept
    {
        func::data<packet> tt{};
        std::copy(begin, begin + length, tt.pure.begin());
        return tt.fields;
    }

    /**
     * Serializes a packet into an output iterator.
     *
     * @param type The packet to serialize.
     * @param begin The output iterator pointing to the first byte of the destination buffer.
     */
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

    /**
     * A structure used to return packet fields.
     *
     * @tparam Header The packet header type.
     * @tparam Fields The packet fields type.
     * @tparam Type The packet data type.
     */
    template <typename Header, typename Fields, typename Type>
    struct __attribute__((__packed__)) fields_out {
        /**
         * The packet header.
         */
        Header header;
        /**
         * The packet fields.
         */
        Fields fields;
        /**
         * A flag indicating if the packet is valid.
         */
        bool valid;
        /**
         * The size of the packet data.
         */
        size_type size;
        /**
         * A pointer to the packet data.
         */
        Type const* data;
    };

    /**
     * A structure used to return packet fields as a pointer.
     *
     * @tparam Header The packet header type.
     * @tparam Fields The packet fields type.
     * @tparam Type The packet data type.
     */
    template <typename Header, typename Fields, typename Type>
    struct __attribute__((__packed__)) fields_out_ptr {
        /**
         * The packet header.
         */
        Header const* const header;
        /**
         * The packet fields.
         */
        Fields const* const fields;
        /**
         * The size of the packet data.
         */
        size_type size;
        /**
         * A pointer to the packet data.
         */
        Type const* data;
    };

    /**
     * A structure used to deserialize packet fields.
     *
     * @tparam Header The packet header type.
     * @tparam Fields The packet fields type.
     * @tparam Type The packet data type.
     */
    template <typename Header, typename Fields, typename Type>
    struct __attribute__((__packed__)) fields_in {
        /**
         * The packet header.
         */
        Header header;
        /**
         * The packet fields.
         */
        Fields fields;
        /**
         * The size of the packet data.
         */
        size_type size;
        /**
         * A pointer to the packet data.
         */
        Type const* data;
    };

    /**
     * Deserializes packet fields without checking the CRC.
     *
     * @tparam Header The packet header type.
     * @tparam Fields The packet fields type.
     * @tparam Type The packet data type.
     * @tparam Crc The CRC type.
     * @return A structure containing the packet fields.
     */
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

    /**
     * Deserializes packet fields.
     *
     * @tparam Header The packet header type.
     * @tparam Fields The packet fields type.
     * @tparam Type The packet data type.
     * @tparam Crc The CRC type.
     * @return A structure containing the packet fields.
     */
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

    /**
     * Serializes packet fields.
     *
     * @tparam Header The packet header type.
     * @tparam Fields The packet fields type.
     * @tparam Type The packet data type.
     * @tparam Crc The CRC type.
     * @param input The packet fields to serialize.
     * @return `true` if the serialization was successful, `false` otherwise.
     */
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

    /**
     * Returns the size of the packet.
     *
     * @return The size of the packet.
     */
    [[nodiscard]] size_type
    size() const
    {
        return size_;
    }

    /**
     * Sets the size of the packet.
     *
     * @param size The new size of the packet.
     */
    void
    size(size_type size)
    {
        size_ = size;
    }

    /**
     * Returns a reference to the packet storage.
     *
     * @return A reference to the packet storage.
     */
    inline array_type&
    storage() noexcept
    {
        return storage_;
    }

    /**
     * Returns a constant reference to the packet storage.
     *
     * @return A constant reference to the packet storage.
     */
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
