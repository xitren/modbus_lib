#include <xitren/circular_buffer.hpp>
#include <xitren/modbus/crc16ansi.hpp>
#include <xitren/modbus/modbus.hpp>
#include <xitren/modbus/packet.hpp>

#include <gtest/gtest.h>

using namespace xitren::modbus;

TEST(modbus_packet_test, crc16ansi)
{
    std::array<std::uint8_t, 5> data = {1, 2, 3, 4, 5};
    crc16ansi::value_type       crc(crc16ansi::calculate(data.begin(), data.end()));
    EXPECT_EQ(crc.get(), 47914);
}

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
