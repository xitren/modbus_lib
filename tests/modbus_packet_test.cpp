#include <xitren/circular_buffer.hpp>
#include <xitren/modbus/crc16ansi.hpp>
#include <xitren/modbus/modbus.hpp>
#include <xitren/modbus/packet.hpp>

#include <gtest/gtest.h>

using namespace xitren::modbus;

struct header_ext {
    uint8_t magic_header[2];
};

struct noise_frame {
    static inline header_ext header = {'N', 'O'};
    uint16_t                 data;
};

struct adc_frame {
    static inline header_ext header = {'L', 'N'};
    uint16_t                 data[11];
};

TEST(modbus_packet_test, packet)
{
    noise_frame adf = {};

    for (auto i{0}; i < 1000; i++) {
        adf.data = static_cast<uint16_t>(std::rand());
        packet<header_ext, noise_frame, crc16ansi> pack(noise_frame::header, adf);
        packet<header_ext, noise_frame, crc16ansi> pack_recv(pack.to_array());
        ASSERT_EQ(noise_frame::header.magic_header[0], pack_recv.header().magic_header[0]);
        ASSERT_EQ(noise_frame::header.magic_header[1], pack_recv.header().magic_header[1]);
    }
}

TEST(modbus_packet_test, deserialize)
{
    std::array<std::uint16_t, 3> a{1, 2, 3};

    packet_accessor<255> arr2{};
    auto                 pack = packet_accessor<255>::fields_in<header, std::uint8_t, std::uint16_t>{
        {0x10, static_cast<uint8_t>(function::read_input_registers)}, {}, a.size(), a.begin()};
    arr2.template serialize<header, std::uint8_t, std::uint16_t, crc16ansi>(pack);
    auto [header_f, field, valid, size, data] = arr2.deserialize<header, std::uint8_t, std::uint16_t, crc16ansi>();
    ASSERT_EQ(size, a.size());
    for (std::size_t i = 0; i < a.size(); i++)
        ASSERT_EQ(data[i], a[i]);
}

TEST(modbus_packet_test, deserialize_no_check)
{
    std::array<std::uint16_t, 3> a{1, 2, 3};
    header                       par1{0x10, static_cast<uint8_t>(function::read_input_registers)};
    std::uint8_t                 par2{};

    packet_accessor<255> arr2{};
    auto pack = packet_accessor<255>::fields_in<header, std::uint8_t, std::uint16_t>{par1, par2, a.size(), a.begin()};
    arr2.template serialize<header, std::uint8_t, std::uint16_t, crc16ansi>(pack);
    auto [header_f, field, size, data] = arr2.deserialize_no_check<header, std::uint8_t, std::uint16_t, crc16ansi>();
    ASSERT_EQ(header_f->function_code, par1.function_code);
    ASSERT_EQ(header_f->slave_id, par1.slave_id);
    ASSERT_EQ(*field, par2);
    ASSERT_EQ(size, a.size());
    for (std::size_t i = 0; i < a.size(); i++)
        ASSERT_EQ(data[i], a[i]);
}

TEST(modbus_packet_test, deserialize_no_check_coils)
{
    header        par1{176, static_cast<uint8_t>(function::read_coils)};
    std::uint16_t par2{16};
    std::uint16_t par3{8};

    packet_accessor<255> arr2{};
    arr2.storage()[0] = 176;
    arr2.storage()[1] = 1;
    arr2.storage()[2] = 0;
    arr2.storage()[3] = 16;
    arr2.storage()[4] = 0;
    arr2.storage()[5] = 8;
    arr2.storage()[6] = 39;
    arr2.storage()[7] = 232;
    arr2.size(8);
    auto [header_f, field, size, data]
        = arr2.deserialize_no_check<header, request_fields_read, xitren::func::msb_t<std::uint16_t>, crc16ansi>();
    ASSERT_EQ(header_f->function_code, par1.function_code);
    ASSERT_EQ(header_f->slave_id, par1.slave_id);
    ASSERT_EQ(field->starting_address.get(), par2);
    ASSERT_EQ(field->quantity.get(), par3);
}
