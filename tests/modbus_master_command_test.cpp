#include <xitren/circular_buffer.hpp>
#include <xitren/comm/observer.hpp>
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
#include <xitren/modbus/crc16ansi.hpp>
#include <xitren/modbus/master.hpp>
#include <xitren/modbus/packet.hpp>
#include <xitren/modbus/slave.hpp>

#include <gtest/gtest.h>

#include <iterator>
#include <limits>
#include <vector>

using namespace xitren::modbus;
using namespace xitren::modbus::commands;

namespace {
using xitren::func::msb_t;

template <typename MsgType, typename Header, typename Fields, typename Type>
bool fill_message(MsgType& msg, Header const& header, Fields const& fields, std::size_t count, Type const* data)
{
    typename MsgType::template fields_in<Header, Fields, Type> input{header, fields, count, data};
    return msg.template serialize<Header, Fields, Type, crc16ansi>(input);
}

template <typename CommandType>
void touch_accessors(CommandType& cmd)
{
    (void)cmd.begin();
    (void)cmd.end();
    (void)cmd.size();
    (void)cmd.msg();
    const CommandType& cref = cmd;
    (void)cref.begin();
    (void)cref.end();
    (void)cref.size();

    command::command_vault_type vault{};
    EXPECT_NE(cmd.clone(vault), nullptr);
    EXPECT_NE(cmd.clone().get(), nullptr);
}

template <typename CommandType>
void touch_accessors_no_msg(CommandType& cmd)
{
    (void)cmd.begin();
    (void)cmd.end();
    (void)cmd.size();
    const CommandType& cref = cmd;
    (void)cref.begin();
    (void)cref.end();
    (void)cref.size();

    command::command_vault_type vault{};
    EXPECT_NE(cmd.clone(vault), nullptr);
    EXPECT_NE(cmd.clone().get(), nullptr);
}

class dummy_command : public command {
public:
    dummy_command() : command(0x11, 0) {}

    iterator
    begin() noexcept override
    {
        return buffer_.begin();
    }

    const_iterator
    begin() const noexcept override
    {
        return buffer_.begin();
    }

    iterator
    end() noexcept override
    {
        return buffer_.end();
    }

    const_iterator
    end() const noexcept override
    {
        return buffer_.end();
    }

    std::size_t
    size() noexcept override
    {
        return buffer_.size();
    }

    std::size_t
    size() const noexcept override
    {
        return buffer_.size();
    }

    command*
    clone(command_vault_type& vault) const noexcept override
    {
        return new (&vault) dummy_command(*this);
    }

    std::shared_ptr<command>
    clone() const noexcept override
    {
        return std::make_shared<dummy_command>(*this);
    }

private:
    std::array<std::uint8_t, 2> buffer_{0x01, 0x02};
};
}    // namespace

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
    timer_start(std::size_t) override
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

TEST(modbus_master_command_test, modbus_master_command_error_paths)
{
    dummy_command base_cmd{};
    EXPECT_EQ(base_cmd.receive(command::msg_type{}), exception::no_error);
    base_cmd.no_answer();
    EXPECT_EQ(base_cmd.error(), exception::bad_slave);

    bool callback_called = false;
    get_log_lvl get_lvl_cmd(0x11, [&](exception ex) {
        EXPECT_EQ(ex, exception::bad_slave);
        callback_called = true;
    });
    touch_accessors(get_lvl_cmd);
    get_lvl_cmd.no_answer();
    EXPECT_TRUE(callback_called);

    command::msg_type error_msg{};
    ASSERT_TRUE(fill_message(error_msg, header{0x11, static_cast<std::uint8_t>(function::get_current_log_level) | error_reply_mask},
                             std::uint8_t{0}, 0, static_cast<std::uint8_t const*>(nullptr)));
    EXPECT_EQ(get_lvl_cmd.receive(error_msg), exception::illegal_function);


    set_max_log_lvl set_lvl_bad(0x11, -1, [&](exception) {});
    EXPECT_EQ(set_lvl_bad.error(), exception::illegal_data_value);

    set_max_log_lvl set_lvl_cmd(0x11, LOG_LEVEL_INFO, [&](exception ex) { EXPECT_EQ(ex, exception::bad_slave); });
    touch_accessors(set_lvl_cmd);
    set_lvl_cmd.no_answer();

    command::msg_type bad_set_lvl_msg{};
    ASSERT_TRUE(fill_message(bad_set_lvl_msg, header{0x22, static_cast<std::uint8_t>(function::set_max_log_level)},
                             std::uint8_t{0}, 0, static_cast<std::uint8_t const*>(nullptr)));
    EXPECT_EQ(set_lvl_cmd.receive(bad_set_lvl_msg), exception::bad_slave);

    read_bits read_bits_cmd(0x11, 0, 8, [&](exception ex, types::bits_array_type::iterator, types::bits_array_type::iterator) {
        EXPECT_EQ(ex, exception::bad_slave);
    });
    touch_accessors(read_bits_cmd);
    read_bits_cmd.no_answer();

    command::msg_type bad_slave_msg{};
    std::array<std::uint8_t, 1> bits_data{0};
    ASSERT_TRUE(fill_message(bad_slave_msg, header{0x22, static_cast<std::uint8_t>(function::read_coils)},
                             std::uint8_t{1}, bits_data.size(), bits_data.data()));
    EXPECT_EQ(read_bits_cmd.receive(bad_slave_msg), exception::bad_slave);

    read_bits read_bits_large(0x11, 0, 8, [&](exception ex, types::bits_array_type::iterator, types::bits_array_type::iterator) {
        EXPECT_EQ(ex, exception::no_error);
    });
    std::vector<std::uint8_t> large_bits(modbus_base::max_read_bits / 8, 0xFF);
    command::msg_type large_bits_msg{};
    ASSERT_TRUE(fill_message(large_bits_msg, header{0x11, static_cast<std::uint8_t>(function::read_coils)},
                             static_cast<std::uint8_t>(large_bits.size()), large_bits.size(), large_bits.data()));
    EXPECT_EQ(read_bits_large.receive(large_bits_msg), exception::no_error);

    read_input_bits input_bits_cmd(0x11, 0, 8, [&](exception, types::bits_array_type::iterator, types::bits_array_type::iterator) {});
    touch_accessors(input_bits_cmd);
    input_bits_cmd.no_answer();

    command::msg_type bad_input_bits_msg{};
    ASSERT_TRUE(fill_message(bad_input_bits_msg, header{0x22, static_cast<std::uint8_t>(function::read_discrete_inputs)},
                             std::uint8_t{1}, bits_data.size(), bits_data.data()));
    EXPECT_EQ(input_bits_cmd.receive(bad_input_bits_msg), exception::bad_slave);

    command::msg_type large_input_bits_msg{};
    ASSERT_TRUE(fill_message(large_input_bits_msg, header{0x11, static_cast<std::uint8_t>(function::read_discrete_inputs)},
                             static_cast<std::uint8_t>(large_bits.size()), large_bits.size(), large_bits.data()));
    EXPECT_EQ(input_bits_cmd.receive(large_input_bits_msg), exception::no_error);

    read_registers read_regs_cmd(0x11, 0, 1, [&](exception, types::array_type::iterator, types::array_type::iterator) {});
    touch_accessors(read_regs_cmd);
    read_regs_cmd.no_answer();

    command::msg_type ok_regs_msg{};
    std::array<msb_t<std::uint16_t>, 1> ok_regs{msb_t<std::uint16_t>{0x1234}};
    ASSERT_TRUE(fill_message(ok_regs_msg, header{0x11, static_cast<std::uint8_t>(function::read_holding_registers)},
                             std::uint8_t{2}, ok_regs.size(), ok_regs.data()));
    EXPECT_EQ(read_regs_cmd.receive(ok_regs_msg), exception::no_error);

    command::msg_type bad_regs_msg{};
    ASSERT_TRUE(fill_message(bad_regs_msg, header{0x22, static_cast<std::uint8_t>(function::read_holding_registers)},
                             std::uint8_t{2}, ok_regs.size(), ok_regs.data()));
    EXPECT_EQ(read_regs_cmd.receive(bad_regs_msg), exception::bad_slave);


    read_input_registers read_input_regs_cmd(0x11, 0, 1,
                                             [&](exception, types::array_type::iterator, types::array_type::iterator) {});
    touch_accessors(read_input_regs_cmd);
    read_input_regs_cmd.no_answer();

    command::msg_type ok_input_regs_msg{};
    ASSERT_TRUE(fill_message(ok_input_regs_msg, header{0x11, static_cast<std::uint8_t>(function::read_input_registers)},
                             std::uint8_t{2}, ok_regs.size(), ok_regs.data()));
    EXPECT_EQ(read_input_regs_cmd.receive(ok_input_regs_msg), exception::no_error);

    command::msg_type bad_input_regs_msg{};
    ASSERT_TRUE(fill_message(bad_input_regs_msg, header{0x22, static_cast<std::uint8_t>(function::read_input_registers)},
                             std::uint8_t{2}, ok_regs.size(), ok_regs.data()));
    EXPECT_EQ(read_input_regs_cmd.receive(bad_input_regs_msg), exception::bad_slave);


    read_log read_log_cmd(0x11, 0, 1, [&](exception, std::uint16_t, std::uint8_t*, std::uint8_t*) {});
    touch_accessors(read_log_cmd);
    read_log_cmd.no_answer();

    std::array<std::uint8_t, 1> log_byte{0};
    command::msg_type bad_log_msg{};
    ASSERT_TRUE(fill_message(bad_log_msg, header{0x22, static_cast<std::uint8_t>(function::read_log)},
                             request_fields_log{msb_t<std::uint16_t>{0}, msb_t<std::uint16_t>{1}},
                             log_byte.size(), log_byte.data()));
    EXPECT_EQ(read_log_cmd.receive(bad_log_msg), exception::bad_slave);

    read_identification read_id_cmd(0x11, static_cast<std::uint8_t>(object_id_code::vendor_name),
                                    [&](exception, std::uint8_t, char*, char*) {});
    touch_accessors(read_id_cmd);
    read_id_cmd.no_answer();

    command::msg_type bad_id_msg{};
    ASSERT_TRUE(fill_message(bad_id_msg, header{0x22, static_cast<std::uint8_t>(function::read_device_identification)},
                             response_identification{modbus_base::mei_type, 0, 0, 0, 0, 0, 0, 0},
                             log_byte.size(), log_byte.data()));
    EXPECT_EQ(read_id_cmd.receive(bad_id_msg), exception::bad_slave);

    std::array<bool, 8> bits{};
    write_bits write_bits_cmd(0x11, 0, bits, [&](exception ex) { EXPECT_EQ(ex, exception::bad_slave); });
    touch_accessors(write_bits_cmd);
    write_bits_cmd.no_answer();

    std::array<bool, 9> bits_odd{};
    write_bits write_bits_odd(0x11, 0, bits_odd, [&](exception ex) { EXPECT_EQ(ex, exception::bad_slave); });
    touch_accessors(write_bits_odd);
    write_bits_odd.no_answer();

    write_bit write_bit_on(0x11, 0, true, [&](exception ex) { EXPECT_EQ(ex, exception::bad_slave); });
    touch_accessors(write_bit_on);
    write_bit_on.no_answer();

    write_bit write_bit_off(0x11, 1, false, [&](exception) {});
    touch_accessors(write_bit_off);

    std::array<std::uint16_t, 1> regs{0};
    write_registers write_regs_cmd(0x11, 0, regs, [&](exception ex) { EXPECT_EQ(ex, exception::bad_slave); });
    touch_accessors(write_regs_cmd);
    write_regs_cmd.no_answer();

    write_register write_reg_cmd(0x11, 0, 0x1234, [&](exception ex) { EXPECT_EQ(ex, exception::bad_slave); });
    touch_accessors(write_reg_cmd);
    write_reg_cmd.no_answer();

    command::msg_type bad_write_bits_msg{};
    ASSERT_TRUE(fill_message(bad_write_bits_msg, header{0x22, static_cast<std::uint8_t>(function::write_multiple_coils)},
                             std::uint8_t{0}, 0, static_cast<std::uint8_t const*>(nullptr)));
    EXPECT_EQ(write_bits_cmd.receive(bad_write_bits_msg), exception::bad_slave);

    command::msg_type bad_write_regs_msg{};
    ASSERT_TRUE(fill_message(bad_write_regs_msg, header{0x22, static_cast<std::uint8_t>(function::write_multiple_registers)},
                             std::uint8_t{0}, 0, static_cast<std::uint8_t const*>(nullptr)));
    EXPECT_EQ(write_regs_cmd.receive(bad_write_regs_msg), exception::bad_slave);

    read_diagnostics_cnt diag_cmd(0x11, diagnostics_sub_function::return_bus_message_count,
                                  [&](exception ex, types::array_type::iterator, types::array_type::iterator) {
                                      EXPECT_EQ(ex, exception::bad_slave);
                                  });
    touch_accessors_no_msg(diag_cmd);
    diag_cmd.no_answer();

    command::msg_type bad_diag_msg{};
    std::array<msb_t<std::uint16_t>, 1> diag_data{msb_t<std::uint16_t>{0}};
    ASSERT_TRUE(fill_message(bad_diag_msg, header{0x22, static_cast<std::uint8_t>(function::diagnostic)},
                             msb_t<std::uint16_t>{
                                 static_cast<std::uint16_t>(diagnostics_sub_function::return_bus_message_count)},
                             diag_data.size(), diag_data.data()));
    EXPECT_EQ(diag_cmd.receive(bad_diag_msg), exception::bad_slave);

    read_diagnostics_cnt diag_invalid(0x11, diagnostics_sub_function::clear_counters,
                                      [&](exception, types::array_type::iterator, types::array_type::iterator) {});
    EXPECT_EQ(diag_invalid.error(), exception::illegal_data_address);

    for (auto sub : {diagnostics_sub_function::return_bus_message_count,
                     diagnostics_sub_function::return_bus_comm_error_count,
                     diagnostics_sub_function::return_server_exception_error_count,
                     diagnostics_sub_function::return_server_message_count,
                     diagnostics_sub_function::return_server_no_response_count,
                     diagnostics_sub_function::return_server_nak_count,
                     diagnostics_sub_function::return_server_busy_count,
                     diagnostics_sub_function::return_bus_char_overrun_count}) {
        read_diagnostics_cnt diag_case(0x11, sub,
                                       [&](exception ex, types::array_type::iterator, types::array_type::iterator) {
                                           EXPECT_NE(ex, exception::illegal_data_address);
                                       });
        EXPECT_EQ(diag_case.error(), exception::no_error);
    }

    read_diagnostics_cnt diag_ok(0x11, diagnostics_sub_function::return_bus_message_count,
                                 [&](exception ex, types::array_type::iterator begin, types::array_type::iterator end) {
                                     EXPECT_EQ(ex, exception::no_error);
                                     EXPECT_EQ(std::distance(begin, end), 1);
                                 });
    command::msg_type diag_ok_msg{};
    std::array<msb_t<std::uint16_t>, 1> diag_ok_data{msb_t<std::uint16_t>{1}};
    ASSERT_TRUE(fill_message(diag_ok_msg, header{0x11, static_cast<std::uint8_t>(function::diagnostic)},
                             msb_t<std::uint16_t>{
                                 static_cast<std::uint16_t>(diagnostics_sub_function::return_bus_message_count)},
                             diag_ok_data.size(), diag_ok_data.data()));
    EXPECT_EQ(diag_ok.receive(diag_ok_msg), exception::no_error);


    read_bits bad_bits_cmd(0x11, std::numeric_limits<std::uint16_t>::max(), 2,
                           [&](exception, types::bits_array_type::iterator, types::bits_array_type::iterator) {});
    EXPECT_EQ(bad_bits_cmd.error(), exception::illegal_data_address);

    read_input_bits bad_input_bits_cmd(0x11, std::numeric_limits<std::uint16_t>::max(), 2,
                                       [&](exception, types::bits_array_type::iterator, types::bits_array_type::iterator) {});
    EXPECT_EQ(bad_input_bits_cmd.error(), exception::illegal_data_address);

    read_registers bad_regs_cmd(0x11, std::numeric_limits<std::uint16_t>::max(), 2,
                                [&](exception, types::array_type::iterator, types::array_type::iterator) {});
    EXPECT_EQ(bad_regs_cmd.error(), exception::illegal_data_address);

    read_input_registers bad_input_regs_cmd(0x11, std::numeric_limits<std::uint16_t>::max(), 2,
                                            [&](exception, types::array_type::iterator, types::array_type::iterator) {});
    EXPECT_EQ(bad_input_regs_cmd.error(), exception::illegal_data_address);

    read_log bad_log_cmd(0x11, std::numeric_limits<std::uint16_t>::max(), 2,
                         [&](exception, std::uint16_t, std::uint8_t*, std::uint8_t*) {});
    EXPECT_EQ(bad_log_cmd.error(), exception::illegal_data_address);

    read_identification bad_id_cmd(0x11, static_cast<std::uint8_t>(object_id_code::max),
                                   [&](exception, std::uint8_t, char*, char*) {});
    EXPECT_EQ(bad_id_cmd.error(), exception::illegal_data_address);

    write_bits bad_write_bits_cmd(0x11, std::numeric_limits<std::uint16_t>::max(), bits, [&](exception) {});
    EXPECT_EQ(bad_write_bits_cmd.error(), exception::illegal_data_address);
}
