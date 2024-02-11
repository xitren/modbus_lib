#include <xitren/circular_buffer.hpp>
#include <xitren/comm/observer.hpp>
#include <xitren/modbus/crc16ansi.hpp>
#include <xitren/modbus/master/modbus_master.hpp>
#include <xitren/modbus/packet.hpp>
#include <xitren/modbus/slave/modbus_slave.hpp>

#include <gtest/gtest.h>

using namespace xitren::modbus;

using slave_type      = modbus_slave<10, 10, 10, 10, 64>;
using observer_type   = xitren::comm::observer<std::vector<std::uint8_t>>;
using observable_type = xitren::comm::observable<std::vector<std::uint8_t>>;

class test_slave : public slave_type, public observer_type, public observable_type {

    bool
    send(msg_type::array_type::iterator begin, msg_type::array_type::iterator end) noexcept override
    {
        std::cout << std::endl;
        std::cout << "<<<<<<<<<<<<<<Slave output msg " << std::endl;
        std::cout << std::noshowbase << std::internal << std::setfill('0');
        last_.clear();
        for (auto& i{begin}; i < end; i++) {
            last_.push_back(*i);
            std::cout << std::hex << std::setw(2) << static_cast<int>(*i) << std::dec << " ";
        }
        notify_observers(last_);
        std::cout << std::endl;
        return true;
    }

public:
    test_slave() : modbus_slave(0x22) { exception_status_ = 0x55; }

    inline std::vector<std::uint8_t>&
    last()
    {
        return last_;
    }

    void
    data(void const*, std::vector<std::uint8_t> const& nd) override
    {
        receive(nd.begin(), nd.end());
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

private:
    std::vector<std::uint8_t> last_;
};

using custom_slave_bits_type      = std::array<bool, 10>;
using custom_slave_registers_type = std::array<std::uint16_t, 10>;
using custom_slave_type = modbus_slave_base<custom_slave_bits_type, custom_slave_bits_type, custom_slave_registers_type,
                                            custom_slave_registers_type, 64>;

class test_custom_slave : public custom_slave_type, public observer_type, public observable_type {

    bool
    send(msg_type::array_type::iterator begin, msg_type::array_type::iterator end) noexcept override
    {
        std::cout << std::endl;
        std::cout << "<<<<<<<<<<<<<<Custom slave output msg " << std::endl;
        std::cout << std::noshowbase << std::internal << std::setfill('0');
        last_.clear();
        for (auto& i{begin}; i < end; i++) {
            last_.push_back(*i);
            std::cout << std::hex << std::setw(2) << static_cast<int>(*i) << std::dec << " ";
        }
        notify_observers(last_);
        std::cout << std::endl;
        return true;
    }

public:
    test_custom_slave() : modbus_slave_base(0x22, bits_, bits_, registers_, registers_) { exception_status_ = 0x55; }

    inline std::vector<std::uint8_t>&
    last()
    {
        return last_;
    }

    void
    data(void const*, std::vector<std::uint8_t> const& nd) override
    {
        receive(nd.begin(), nd.end());
        processing();
        processing();
        processing();
    }

private:
    std::vector<std::uint8_t>   last_;
    custom_slave_bits_type      bits_{};
    custom_slave_registers_type registers_{};
};

class test_master : public modbus_master, public observer_type, public observable_type {

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
        auto current_state = state_;
        notify_observers(last_);
        // TODO: Workaround Force to change the internal state when we want to test write modbus
        // functions. so that we could skip  infinite loop in wait_input_msg().
        if ((state_ == current_state) && (state_ == master_state::waiting_reply)) {
            state_ = master_state::processing_reply;
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
    timer_start(std::size_t microseconds) override
    {
        return true;
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

    void
    data(void const*, std::vector<std::uint8_t> const& nd) override
    {
        receive(nd.begin(), nd.end());
    }

private:
    std::vector<std::uint8_t> last_;
};

template <typename T, size_t Size>
bool
arrays_match(std::array<T, Size> const& expected, std::array<T, Size> const& actual)
{
    std::cout << "===========Expected " << std::endl;
    std::cout << std::noshowbase << std::internal << std::setfill('0');
    for (auto i : expected) {
        std::cout << std::hex << std::setw(2) << static_cast<int>(i) << std::dec << " ";
    }
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

template <typename T, typename U>
bool
container_match(T const& expected, U const& actual)
{
    if (expected.size() != actual.size()) {
        std::cout << "expected.size() (" << expected.size() << ") != actual.size() (" << actual.size() << ")\n";
        return false;
    }

    std::cout << "===========Expected " << std::endl;
    std::cout << std::noshowbase << std::internal << std::setfill('0');
    for (auto i : expected) {
        std::cout << std::hex << std::setw(2) << static_cast<int>(i) << std::dec << " ";
    }
    std::cout << std::endl;
    std::cout << "===========Actual " << std::endl;
    std::cout << std::noshowbase << std::internal << std::setfill('0');
    for (auto i : actual) {
        std::cout << std::hex << std::setw(2) << static_cast<int>(i) << std::dec << " ";
    }
    std::cout << std::endl;
    auto   it_actual = actual.begin();
    size_t i         = 0;
    for (auto item : expected) {
        if (item != *it_actual) {
            std::cout << "actual[" << i << "] (" << *it_actual << ") != expected[" << i << "] (" << item << ")"
                      << std::endl;
            return false;
        }
        ++it_actual;
        ++i;
    }
    return true;
}

TEST(modbus_test, modbus_slave_crc_first)
{
    test_slave                sl;
    std::vector<std::uint8_t> array_v_v_v{0x22, 0x01, 0x00, 0x00, 0x00, 0x08, 0x3A, 0x9F};
    std::vector<std::uint8_t> array_i_v_v{0x20, 0x01, 0x00, 0x00, 0x00, 0x08, 0x3B, 0x7D};
    std::vector<std::uint8_t> array_v_i_v{0x22, 0x15, 0x00, 0x00, 0x00, 0x08, 0x0A, 0x9C};
    std::vector<std::uint8_t> array_v_v_i{0x22, 0x01, 0x00, 0x00, 0x00, 0x08, 0x3A, 0x9A};
    std::vector<std::uint8_t> array_v_0_0{0x22, 0x01};

    sl.receive(array_v_v_v.begin(), array_v_v_v.end());
    EXPECT_TRUE(exception::no_error == sl.processing());
    EXPECT_TRUE(slave_state::processing_action == sl.state());
    sl.reset();

    sl.receive(array_i_v_v.begin(), array_i_v_v.end());
    EXPECT_TRUE(exception::bad_slave == sl.processing());
    EXPECT_TRUE(slave_state::idle == sl.state());
    sl.reset();

    sl.receive(array_v_i_v.begin(), array_v_i_v.end());
    EXPECT_TRUE(exception::illegal_function == sl.processing());
    EXPECT_TRUE(slave_state::formatting_error_reply == sl.state());
    sl.reset();

    auto err = sl.receive(array_v_v_i.begin(), array_v_v_i.end());
    EXPECT_TRUE(exception::bad_crc == err);
    EXPECT_TRUE(slave_state::idle == sl.state());
    sl.reset();

    err = sl.receive(array_v_0_0.begin(), array_v_0_0.end());
    EXPECT_TRUE(exception::bad_data == err);
    EXPECT_TRUE(slave_state::idle == sl.state());
    sl.reset();
}
