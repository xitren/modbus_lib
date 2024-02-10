#include <xitren/circular_buffer.hpp>
#include <xitren/comm/observer.hpp>
#include <xitren/modbus/crc16ansi.hpp>
#include <xitren/modbus/master/modbus_master.hpp>
#include <xitren/modbus/packet.hpp>
#include <xitren/modbus/slave/modbus_slave.hpp>

#include <gtest/gtest.h>

using namespace xitren::modbus;

class test_master : public modbus_master {

    bool
    send(msg_type::array_type::iterator begin, msg_type::array_type::iterator end) noexcept override
    {
        std::cout << std::endl;
        std::cout << ">>>>>>>>>>>>>>Master output msg " << std::endl;
        std::cout << std::noshowbase << std::internal << std::setfill('0');
        last_.clear();
        for (auto i{begin}; i < end; i++) {
            last_.push_back(*i);
            std::cout << std::hex << std::setw(2) << static_cast<int>(*i) << std::dec << " ";
        }
        std::cout << std::endl;
        return true;
    }

public:
    test_master() {}

    void
    add(msg_type::array_type::iterator begin, msg_type::array_type::iterator end) noexcept
    {
        receive(begin, end);
    }

    bool
    timer_start(std::size_t) override
    {
        return false;
    }

    bool
    timer_stop() override
    {
        return true;
    }

    inline std::vector<std::uint8_t>&
    last()
    {
        return last_;
    }

private:
    std::vector<std::uint8_t> last_;
};

template <typename T, size_t Size>
bool
arrays_match(std::array<T, Size> const& expected, std::vector<T> const& actual)
{
    std::cout << "===========Expected " << std::endl;
    std::cout << std::noshowbase << std::internal << std::setfill('0');
    for (auto i : expected) {
        std::cout << std::hex << std::setw(2) << static_cast<int>(i) << std::dec << " ";
    }
    std::cout << std::endl << "CRC:";
    std::cout << std::hex << std::setw(4) << crc16ansi::calculate(expected.begin(), expected.end() - 2).get()
              << std::dec << " ";
    std::cout << std::endl;
    for (size_t i{0}; i < Size; ++i) {
        if (expected[i] != actual[i]) {
            std::cout << "array[" << i << "] (" << actual[i] << ") != expected[" << i << "] (" << expected[i] << ")"
                      << std::endl;
            return false;
        }
    }
    return true;
}

TEST(modbus_master_test, modbus_master_read_coils)
{
    constexpr auto              address = 0x22;
    test_master                 ms;
    std::array<bool, 1>         val{};
    std::array<bool, 0x30>      val2{};
    std::array<std::uint8_t, 8> array{0x22, 0x01, 0x00, 0x00, 0x00, 0x01, 0xFA, 0x99};
    std::array<std::uint8_t, 8> array2{0x22, 0x01, 0x00, 0x25, 0x00, 0x30, 0x2A, 0x86};

    ms.read_bits(address, 0, val);
    EXPECT_TRUE(arrays_match(array, ms.last()));

    ms.read_bits(address, 0x25, val2);
    EXPECT_TRUE(arrays_match(array2, ms.last()));
}

TEST(modbus_master_test, modbus_master_read_inputs)
{
    constexpr auto              address = 0x22;
    test_master                 ms;
    std::array<bool, 1>         val{};
    std::array<bool, 0x30>      val2{};
    std::array<std::uint8_t, 8> array{0x22, 0x02, 0x00, 0x00, 0x00, 0x01, 0xBE, 0x99};
    std::array<std::uint8_t, 8> array2{0x22, 0x02, 0x00, 0x25, 0x00, 0x30, 0x6E, 0x86};

    ms.read_input_bits(address, 0, val);
    EXPECT_TRUE(arrays_match(array, ms.last()));

    ms.read_input_bits(address, 0x25, val2);
    EXPECT_TRUE(arrays_match(array2, ms.last()));
}

TEST(modbus_master_test, modbus_master_read_holdings)
{
    constexpr auto                  address = 0x22;
    test_master                     ms;
    std::array<std::uint16_t, 1>    val{};
    std::array<std::uint16_t, 0x30> val2{};
    std::array<std::uint8_t, 8>     array{0x22, 0x03, 0x00, 0x00, 0x00, 0x01, 0x83, 0x59};
    std::array<std::uint8_t, 8>     array2{0x22, 0x03, 0x00, 0x25, 0x00, 0x30, 0x53, 0x46};

    ms.read_registers(address, 0, val);
    EXPECT_TRUE(arrays_match(array, ms.last()));

    ms.read_registers(address, 0x25, val2);
    EXPECT_TRUE(arrays_match(array2, ms.last()));
}

TEST(modbus_master_test, modbus_master_read_input_registers)
{
    constexpr auto                  address = 0x22;
    test_master                     ms;
    std::array<std::uint16_t, 1>    val{};
    std::array<std::uint16_t, 0x30> val2{};
    std::array<std::uint8_t, 8>     array{0x22, 0x04, 0x00, 0x00, 0x00, 0x01, 0x36, 0x99};
    std::array<std::uint8_t, 8>     array2{0x22, 0x04, 0x00, 0x25, 0x00, 0x30, 0xE6, 0x86};

    ms.read_input_registers(address, 0, val);
    EXPECT_TRUE(arrays_match(array, ms.last()));

    ms.read_input_registers(address, 0x25, val2);
    EXPECT_TRUE(arrays_match(array2, ms.last()));
}

TEST(modbus_master_test, modbus_master_write_coil)
{
    constexpr auto              address = 0x22;
    test_master                 ms;
    std::array<std::uint8_t, 8> array{0x22, 0x05, 0x00, 0x00, 0xFF, 0x00, 0x8B, 0x69};
    std::array<std::uint8_t, 8> array2{0x22, 0x05, 0x00, 0x25, 0x00, 0x00, 0xDB, 0x52};

    ms.write_bit(address, 0, true);
    EXPECT_TRUE(arrays_match(array, ms.last()));

    ms.write_bit(address, 0x25, false);
    EXPECT_TRUE(arrays_match(array2, ms.last()));
}

TEST(modbus_master_test, modbus_master_write_registers)
{
    constexpr auto               address = 0x22;
    test_master                  ms;
    std::array<std::uint16_t, 1> val{};
    std::array<std::uint16_t, 6> val2{};
    std::array<std::uint8_t, 11> array{0x22, 0x10, 0x00, 0x00, 0x00, 0x01, 0x02, 0x00, 0x0A, 0xAB, 0x66};
    std::array<std::uint8_t, 21> array2{0x22, 0x10, 0x00, 0x25, 0x00, 0x06, 0x0C, 0x01, 0x0A, 0x02, 0x0A,
                                        0x03, 0x0A, 0x04, 0x0A, 0x05, 0x0A, 0x06, 0x0A, 0xCC, 0x57};

    val[0] = 0x000A;
    ms.write_registers(address, 0, val);
    EXPECT_TRUE(arrays_match(array, ms.last()));

    for (std::uint16_t i{}; i < 6; i++) {
        val2[i] = ((i + 1) << 8) | 0x000A;
    }
    ms.write_registers(address, 0x25, val2);
    EXPECT_TRUE(arrays_match(array2, ms.last()));
}

TEST(modbus_master_test, modbus_master_write_register)
{
    constexpr auto               address = 0x22;
    test_master                  ms;
    std::array<std::uint16_t, 1> val{0x0006};
    std::array<std::uint8_t, 8>  array{0x22, 0x06, 0x00, 0x00, 0x00, 0x06, 0x0E, 0x9B};
    std::array<std::uint8_t, 8>  array2{0x22, 0x06, 0x00, 0x25, 0x00, 0x06, 0x1F, 0x50};

    ms.write_register(address, 0, val[0]);
    EXPECT_TRUE(arrays_match(array, ms.last()));

    ms.write_register(address, 0x25, val[0]);
    EXPECT_TRUE(arrays_match(array2, ms.last()));
}

TEST(modbus_master_test, modbus_master_fifo)
{
    using buffer_type = xitren::containers::circular_buffer<std::uint16_t, 1024>;

    constexpr auto              address = 0x22;
    test_master                 ms;
    std::array<std::uint8_t, 6> array{0x22, 0x18, 0x00, 0x00, 0x8A, 0x5B};
    std::array<std::uint8_t, 6> array2{0x22, 0x18, 0x00, 0x25, 0x4B, 0x80};
    buffer_type                 buffer{};

    auto err = ms.read_fifo_queue(address, 0, buffer);
    // EXPECT_TRUE(err == exception::no_error);
    EXPECT_TRUE(arrays_match(array, ms.last()));

    ms.read_fifo_queue(address, 0x25, buffer);
    EXPECT_TRUE(arrays_match(array2, ms.last()));
}
