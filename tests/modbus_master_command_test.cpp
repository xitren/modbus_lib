#include <xitren/circular_buffer.hpp>
#include <xitren/comm/observer.hpp>
#include <xitren/modbus/crc16ansi.hpp>
#include <xitren/modbus/master/modbus_command_bits.hpp>
#include <xitren/modbus/master/modbus_command_regs.hpp>
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
        for (auto i{begin}; i < end; i++) {
            std::cout << std::hex << std::setw(2) << static_cast<int>(*i) << std::dec << " ";
        }
        std::cout << std::endl;
        return true;
    }

public:
    test_master() = default;

    void
    add(msg_type::array_type::iterator begin, msg_type::array_type::iterator end) noexcept
    {
        receive(begin, end);
    }

    bool
    timer_start(std::size_t microseconds) override
    {
        return true;
    }

    bool
    timer_stop() override
    {
        return true;
    }
};

template <typename T, size_t Size, size_t Size1>
bool
arrays_match(std::array<T, Size> const& expected, std::array<T, Size1> const& actual, std::size_t size)
{
    std::cout << "===========Expected " << std::endl;
    std::cout << std::noshowbase << std::internal << std::setfill('0');
    for (size_t i{0}; i < size; ++i) {
        std::cout << std::hex << std::setw(2) << static_cast<int>(expected[i]) << std::dec << " ";
    }
    for (size_t i{0}; i < size; ++i) {
        if (expected[i] != actual[i]) {
            std::cout << "array[" << i << "] (" << actual[i] << ") != expected[" << i << "] (" << expected[i] << ")"
                      << std::endl;
            return false;
        }
    }
    return true;
}

using slave_type = modbus_slave<0x02, 10, 10, 10, 10, 64>;

class test_slave : public slave_type {

    bool
    send(msg_type::array_type::iterator begin, msg_type::array_type::iterator end) noexcept override
    {
        begin_last_ = begin;
        end_last_   = end;
        std::cout << std::endl;
        std::cout << "<<<<<<<<<<<<<<Slave output msg " << std::endl;
        std::cout << std::noshowbase << std::internal << std::setfill('0');
        for (auto i{begin}; i < end; i++) {
            std::cout << std::hex << std::setw(2) << static_cast<int>(*i) << std::dec << " ";
        }
        std::cout << std::endl;
        return true;
    }

    msg_type::array_type::iterator begin_last_{nullptr};
    msg_type::array_type::iterator end_last_{nullptr};

public:
    test_slave() : modbus_slave(0x20) { exception_status_ = 0x55; }

    inline msg_type::array_type::iterator
    begin_last() noexcept
    {
        return begin_last_;
    }

    [[nodiscard]] inline msg_type::array_type::iterator
    begin_last() const noexcept
    {
        return begin_last_;
    }

    inline msg_type::array_type::iterator
    end_last() noexcept
    {
        return end_last_;
    }

    [[nodiscard]] inline msg_type::array_type::iterator
    end_last() const noexcept
    {
        return end_last_;
    }

    template <std::size_t Size>
    void
    data(std::array<std::uint8_t, Size>& nd, std::size_t size)
    {
        receive(nd.begin(), nd.begin() + size);
        processing();
        processing();
        processing();
    }

    void
    changed_coil(std::size_t address, bool value) noexcept override
    {
        inputs()[address] = value;
    }

    void
    changed_holding(std::size_t address, std::uint16_t value) noexcept override
    {
        input_registers()[address] = value;
    }
};

TEST(modbus_master_command_test, modbus_master_read_coils)
{
    constexpr auto              address = 0x22;
    test_master                 master{};
    test_slave                  slave{};
    std::array<std::uint8_t, 8> array{0x22, 0x01, 0x00, 0x00, 0x00, 0x08, 0x3A, 0x9F};
    volatile bool                        result_modbus_master_read_coils = false;

    read_bits t1(address, 0, 8,
                 [&](exception, types::bits_array_type::iterator begin, types::bits_array_type::iterator end) mutable {
                     std::cout << "read_bits callback" << std::endl;
                     EXPECT_TRUE(std::distance(begin, end) == 8);
                     result_modbus_master_read_coils = true;
                 });
    EXPECT_TRUE(t1.error() == exception::no_error);
    EXPECT_TRUE(arrays_match(t1.msg().storage(), array, t1.msg().size()));

    master << t1;
    slave.data(t1.msg().storage(), t1.msg().size());
    master.receive(slave.begin_last(), slave.end_last());
    EXPECT_TRUE(t1.error() == exception::no_error);
    EXPECT_TRUE(result_modbus_master_read_coils);
}

TEST(modbus_master_command_test, modbus_master_read_inputs)
{
    constexpr auto              address = 0x22;
    test_master                 master{};
    test_slave                  slave{};
    std::array<std::uint8_t, 8> array{0x22, 0x02, 0x00, 0x00, 0x00, 0x08, 0x7E, 0x9F};
    bool                        result_modbus_master_read_inputs = false;

    read_input_bits t1(
        address, 0, 8,
        [&](exception, types::bits_array_type::iterator begin, types::bits_array_type::iterator end) mutable {
            std::cout << "read_inputs callback" << std::endl;
            EXPECT_TRUE(std::distance(begin, end) == 8);
            result_modbus_master_read_inputs = true;
        });
    EXPECT_TRUE(t1.error() == exception::no_error);
    EXPECT_TRUE(arrays_match(t1.msg().storage(), array, t1.msg().size()));

    master << t1;
    slave.data(t1.msg().storage(), t1.msg().size());
    master.receive(slave.begin_last(), slave.end_last());
    EXPECT_TRUE(t1.error() == exception::no_error);
    EXPECT_TRUE(result_modbus_master_read_inputs);
}

TEST(modbus_master_command_test, modbus_master_read_holdings)
{
    constexpr auto              address = 0x22;
    test_master                 master{};
    test_slave                  slave{};
    std::array<std::uint8_t, 8> array{0x22, 0x03, 0x00, 0x00, 0x00, 0x02, 0xC3, 0x58};
    bool                        result_modbus_master_read_holdings = false;

    read_registers t1(address, 0, 2,
                      [&](exception, types::array_type::iterator begin, types::array_type::iterator end) mutable {
                          std::cout << "read_holdings callback" << std::endl;
                          EXPECT_TRUE(std::distance(begin, end) == 2);
                          result_modbus_master_read_holdings = true;
                      });
    EXPECT_TRUE(t1.error() == exception::no_error);
    EXPECT_TRUE(arrays_match(t1.msg().storage(), array, t1.msg().size()));

    master << t1;
    slave.data(t1.msg().storage(), t1.msg().size());
    master.receive(slave.begin_last(), slave.end_last());
    EXPECT_TRUE(t1.error() == exception::no_error);
    EXPECT_TRUE(result_modbus_master_read_holdings);
}
