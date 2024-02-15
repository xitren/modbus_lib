#include <xitren/modbus/master.hpp>
#include <xitren/modbus/slave.hpp>
#include <xitren/circular_buffer.hpp>
#include <xitren/comm/observer.hpp>
#include <xitren/modbus/crc16ansi.hpp>
#include <xitren/modbus/commands/get_log_lvl.hpp>
#include <xitren/modbus/commands/instant/read_diagnostics_cnt.hpp>
#include <xitren/modbus/commands/instant/read_registers.hpp>
#include <xitren/modbus/commands/instant/write_registers.hpp>
#include <xitren/modbus/commands/read_bits.hpp>
#include <xitren/modbus/commands/read_diagnostics_cnt.hpp>
#include <xitren/modbus/commands/read_identification.hpp>
#include <xitren/modbus/commands/read_input_bits.hpp>
#include <xitren/modbus/commands/read_input_registers.hpp>
#include <xitren/modbus/commands/read_log.hpp>
#include <xitren/modbus/commands/read_registers.hpp>
#include <xitren/modbus/commands/set_max_log_lvl.hpp>
#include <xitren/modbus/commands/write_bit.hpp>
#include <xitren/modbus/commands/write_bits.hpp>
#include <xitren/modbus/commands/write_register.hpp>
#include <xitren/modbus/commands/write_registers.hpp>
#include <xitren/modbus/packet.hpp>

#include <gtest/gtest.h>

using namespace xitren::modbus;
using namespace xitren::modbus::commands;

class test_master : public master {

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

using slave_type = slave<10, 10, 10, 10, 64>;

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
    test_slave() : slave(0x22) { exception_status_ = 0x55; }

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
    data(std::array<std::uint8_t, Size> const& nd, std::size_t size)
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
    bool                        result_modbus_master_read_coils = false;

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

template <typename _Traits, typename AdvCommands>
inline std::basic_ostream<char, _Traits>&
operator<<(std::basic_ostream<char, _Traits>& __out, [[maybe_unused]] AdvCommands const& in_data)
{
    for (auto& i : AdvCommands::output_command) {
        __out << std::hex << static_cast<int>(i) << ' ';
    }
    return __out;
}

template <size_t Size>
bool
arrays_match(std::array<std::uint8_t, Size> const& expected, std::array<std::uint8_t, Size> const& actual)
{
    for (size_t i{0}; i < Size; ++i) {
        if (expected[i] != actual[i]) {
            std::cout << "array[" << i << "] (" << actual[i] << ") != expected[" << i << "] (" << expected[i] << ")"
                      << std::endl;
            return false;
        }
    }
    return true;
}

//TEST(modbus_master_command_test, modbus_master_read_registers_adv)
//{
//    GTEST_SKIP();
//    constexpr std::array<std::uint8_t, 8> test{0x22, 0x03, 0x00, 0x01, 0x00, 0x02, 0x92, 0x98};
//    constexpr std::uint8_t                address = 0x22;
//    test_master                           master{};
//    test_slave                            slave{};
//    static bool                           result_modbus = false;
//
//    using type = read_registers_static<address, 0x0001, 2, [&](exception ex, std::uint16_t* begin, std::uint16_t* end) {
//        std::cout << "read_holdings callback" << std::endl;
//        EXPECT_TRUE(std::distance(begin, end) == 2);
//        result_modbus = (ex == xitren::modbus::exception::no_error);
//    }>;
//    constexpr type r1{};
//    std::cout << r1 << std::endl;
//    EXPECT_TRUE(arrays_match(test, type::output_command));
//
//    master << r1;
//    slave.data(type::output_command, type::output_command.size());
//    master.receive(slave.begin_last(), slave.end_last());
//    EXPECT_TRUE(r1.error() == exception::no_error);
//    EXPECT_TRUE(result_modbus);
//}

//TEST(modbus_master_command_test, modbus_master_write_registers_adv)
//{
//    GTEST_SKIP();
//    constexpr std::array<std::uint8_t, 13> test{0x22, 0x10, 0x00, 0x01, 0x00, 0x02, 0x04,
//                                                0x22, 0x46, 0x03, 0x56, 0xfd, 0x84};
//    constexpr std::uint8_t                 address = 0x22;
//    test_master                            master{};
//    test_slave                             slave{};
//    static bool                            result_modbus = false;
//    constexpr std::array<std::uint16_t, 2> array{0x2246, 0x0356};
//    using type = write_registers_static<address, 0x0001, 2, array, [](exception ex) {
//        std::cout << "write_registers callback" << std::endl;
//        result_modbus = (ex == xitren::modbus::exception::no_error);
//    }>;
//    constexpr type r1{};
//    std::cout << r1 << std::endl;
//    EXPECT_TRUE(arrays_match(test, type::output_command));
//
//    master << r1;
//    slave.data(type::output_command, type::output_command.size());
//    master.receive(slave.begin_last(), slave.end_last());
//    EXPECT_TRUE(r1.error() == exception::no_error);
//    EXPECT_TRUE(result_modbus);
//}
//
//TEST(modbus_master_command_test, modbus_master_read_diagnostics_adv)
//{
//    GTEST_SKIP();
//    using namespace xitren::modbus;
//    constexpr std::array<std::uint8_t, 6> test{0x22, 0x08, 0x00, 0x0b, 0xca, 0x59};
//    constexpr std::uint8_t                address = 0x22;
//    test_master                           master{};
//    test_slave                            slave{};
//    static bool                           result_modbus = false;
//
//    using type = read_diagnostics_cnt_static<address, diagnostics_sub_function::return_bus_message_count,
//                                             [&](exception ex, std::uint16_t data) {
//                                                 std::cout << "return_bus_message_count callback " << data << std::endl;
//                                                 result_modbus = (ex == xitren::modbus::exception::no_error);
//                                             }>;
//    constexpr type r1{};
//    EXPECT_TRUE(arrays_match(test, type::output_command));
//    master << r1;
//    slave.data(type::output_command, type::output_command.size());
//    master.receive(slave.begin_last(), slave.end_last());
//    EXPECT_TRUE(r1.error() == exception::no_error);
//    EXPECT_TRUE(result_modbus);
//}

template <size_t Size>
bool
arrays_match(std::array<std::uint8_t, Size> const& expected, modbus_base::msg_type const& actual)
{
    for (size_t i{0}; i < Size && i < actual.size(); ++i) {
        if (expected[i] != actual.storage()[i]) {
            std::cout << "array[" << i << "] (" << actual.storage()[i] << ") != expected[" << i << "] (" << expected[i]
                      << ")" << std::endl;
            return false;
        }
    }
    return true;
}

TEST(modbus_master_command_test, modbus_master_read_log)
{
    using namespace xitren::modbus;
    constexpr std::array<std::uint8_t, 8>  test{0x22, 0x41, 0x00, 0x00, 0x00, 0x10, 0x3b, 0x5a};
    constexpr std::array<std::uint8_t, 11> test_recv{0x22, 0x41, 0x00, 0x00, 0x00, 0x03, 0x31, 0x32, 0x33, 0x4b, 0x53};
    constexpr std::uint8_t                 slave_address = 0x22;
    constexpr std::uint16_t                address       = 0x00;
    constexpr std::size_t                  size          = 0x10;
    test_master                            master{};
    test_slave                             slave{};
    static bool                            result_modbus = false;

    class read_log r1(slave_address, address, size,
                      [&](exception ex, std::uint16_t address, std::uint8_t* begin, std::uint8_t* end) {
                          std::cout << "read_log callback " << address << std::endl;
                          for (auto i{begin}; i != end; ++i) {
                              std::cout << std::hex << static_cast<int>(*i) << " ";
                          }
                          std::cout << std::endl;
                          result_modbus = (ex == xitren::modbus::exception::no_error);
                      });
    EXPECT_TRUE(arrays_match(test, r1.msg()));
    master << r1;
    slave.log().push(0x31);
    slave.log().push(0x32);
    slave.log().push(0x33);
    slave.data(r1.msg().storage(), r1.msg().size());
    EXPECT_TRUE(std::equal(slave.begin_last(), slave.end_last(), test_recv.begin()));
    master.receive(slave.begin_last(), slave.end_last());
    EXPECT_TRUE(r1.error() == exception::no_error);
    EXPECT_TRUE(result_modbus);
}

TEST(modbus_master_command_test, modbus_master_set_max_log_lvl)
{
    using namespace xitren::modbus;
    constexpr std::array<std::uint8_t, 5> test{0x22, 0x42, 0x03, 0xa1, 0x6b};
    constexpr std::array<std::uint8_t, 5> test_recv{0x22, 0x42, 0x03, 0xa1, 0x6b};
    constexpr std::uint8_t                slave_address = 0x22;
    test_master                           master{};
    test_slave                            slave{};
    static bool                           result_modbus = false;

    class set_max_log_lvl r1(slave_address, LOG_LEVEL_WARN,
                             [&](exception ex) { result_modbus = (ex == xitren::modbus::exception::no_error); });
    EXPECT_TRUE(arrays_match(test, r1.msg()));
    master << r1;
    slave.data(r1.msg().storage(), r1.msg().size());
    EXPECT_TRUE(std::equal(slave.begin_last(), slave.end_last(), test_recv.begin()));
    master.receive(slave.begin_last(), slave.end_last());
    EXPECT_TRUE(r1.error() == exception::no_error);
    EXPECT_TRUE(result_modbus);
}

TEST(modbus_master_command_test, modbus_master_get_log_lvl)
{
    using namespace xitren::modbus;
    constexpr std::array<std::uint8_t, 5> test{0x22, 0x43, 0x00, 0xe0, 0xfa};
    constexpr std::array<std::uint8_t, 5> test_recv{0x22, 0x43, 0x03, 0xa0, 0xfb};
    constexpr std::uint8_t                slave_address = 0x22;
    test_master                           master{};
    test_slave                            slave{};
    static bool                           result_modbus = false;

    class get_log_lvl r1(slave_address,
                         [&](exception ex) { result_modbus = (ex == xitren::modbus::exception::no_error); });
    EXPECT_TRUE(arrays_match(test, r1.msg()));
    master << r1;
    slave.data(r1.msg().storage(), r1.msg().size());
    EXPECT_TRUE(std::equal(slave.begin_last(), slave.end_last(), test_recv.begin()));
    master.receive(slave.begin_last(), slave.end_last());
    EXPECT_TRUE(r1.error() == exception::no_error);
    EXPECT_TRUE(result_modbus);
}

TEST(modbus_master_command_test, modbus_master_read_identity)
{
    constexpr std::uint8_t                 address = 0x22;
    test_master                            master{};
    test_slave                             slave{};
    std::array<std::uint8_t, 7>            array{0x22, 0x2b, 0x0e, 0x04, 0x00, 0xb6, 0xe0};
    constexpr std::array<std::uint8_t, 21> test_recv{0x22, 0x2b, 0x0e, 0x04, 0x81, 0x00, 0x00, 0x01, 0x00, 0x09, 0x52,
                                                     0x6f, 0x62, 0x6f, 0x6c, 0x61, 0x76, 0x6b, 0x61, 0xcc, 0xfe};
    bool                                   result_modbus_master_read_identity = false;

    class read_identification t1(address, 0, [&](exception, std::uint8_t address, char* begin, char* end) mutable {
        std::cout << "response_identification callback" << std::endl;
        std::cout << "id " << std::hex << static_cast<int>(address) << std::endl;
        for (auto it{begin}; it != end; it++) {
            std::cout << static_cast<std::uint8_t>(*it);
        }
        std::cout << std::endl;
        result_modbus_master_read_identity = true;
    });
    EXPECT_TRUE(t1.error() == exception::no_error);
    EXPECT_TRUE(arrays_match(t1.msg().storage(), array, t1.msg().size()));

    master << t1;
    slave.data(t1.msg().storage(), t1.msg().size());
    EXPECT_TRUE(std::equal(slave.begin_last(), slave.end_last(), test_recv.begin()));
    master.receive(slave.begin_last(), slave.end_last());
    EXPECT_TRUE(t1.error() == exception::no_error);
    EXPECT_TRUE(result_modbus_master_read_identity);
}
