#include "loveka/components/modbus/master/modbus_master.hpp"
#include "loveka/components/modbus/slave/modbus_slave.hpp"
#include <loveka/components/modbus/crc16ansi.hpp>
#include <loveka/components/modbus/packet.hpp>
#include <loveka/components/utils/circular_buffer.hpp>
#include <loveka/components/utils/observer.hpp>

#include <gtest/gtest.h>

using namespace loveka::components::modbus;

using slave_type      = modbus_slave<0x02, 10, 10, 10, 10, 64>;
using observer_type   = loveka::components::utils::observer<std::vector<std::uint8_t>>;
using observable_type = loveka::components::utils::observable<std::vector<std::uint8_t>>;

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
    test_slave() : modbus_slave(0x20) { exception_status_ = 0x55; }

    inline std::vector<std::uint8_t>&
    last()
    {
        return last_;
    }

    void
    data(const void*, const std::vector<std::uint8_t>& nd) override
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
    timer_start(std::size_t) override
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
    data(const void*, const std::vector<std::uint8_t>& nd) override
    {
        receive(nd.begin(), nd.end());
    }

private:
    std::vector<std::uint8_t> last_;
};

template <typename T, size_t Size>
bool
arrays_match(const std::array<T, Size>& expected, const std::array<T, Size>& actual)
{
    std::cout << "===========Expected " << std::endl;
    std::cout << std::noshowbase << std::internal << std::setfill('0');
    for (auto i : expected) {
        std::cout << std::hex << std::setw(2) << static_cast<int>(i) << std::dec << " ";
    }
    std::cout << std::endl;
    for (size_t i{0}; i < Size; ++i) {
        if (expected[i] != actual[i]) {
            std::cout << "array[" << i << "] (" << actual[i] << ") != expected[" << i << "] ("
                      << expected[i] << ")" << std::endl;
            return false;
        }
    }
    return true;
}

TEST(modbus_test, modbus_master_multiple_registers)
{
    constexpr auto address = 0x22;
    test_slave     sl;
    test_master    ms;
    sl.add_observer(ms);
    ms.add_observer(sl);

    std::array<std::uint16_t, 6> val2{};
    std::array<std::uint16_t, 6> arr{};
    for (std::uint16_t i{}; i < 6; i++) {
        val2[i] = ((i + 1) << 8) | 0x000A;
    }
    ms.write_registers(0x22, 0x02, val2);
    ms.read_registers(address, 0x02, arr);
    EXPECT_TRUE(arrays_match(val2, arr));
}

TEST(modbus_test, modbus_master_registers)
{
    constexpr auto address = 0x22;
    test_slave     sl;
    test_master    ms;
    sl.add_observer(ms);
    ms.add_observer(sl);

    std::array<std::uint16_t, 3> arr{};
    std::array<std::uint16_t, 3> arr_target{0, 0, 0};
    ms.read_registers(address, 1, arr);
    EXPECT_TRUE(arrays_match(arr_target, arr));

    std::array<std::uint16_t, 3> arr_target2{0, 15, 0};
    ms.write_register(address, 2, 15);
    ms.read_input_registers(address, 1, arr);
    EXPECT_TRUE(arrays_match(arr_target2, arr));
}

TEST(modbus_test, modbus_master_coils)
{
    constexpr auto address = 0x22;
    test_slave     sl;
    test_master    ms;
    sl.add_observer(ms);
    ms.add_observer(sl);

    std::array<bool, 3> arr{};
    std::array<bool, 3> arr_target{};
    ms.read_bits(address, 1, arr);
    EXPECT_TRUE(arrays_match(arr_target, arr));

    std::array<bool, 3> arr_target2{false, true, false};
    ms.write_bit(address, 2, true);
    ms.read_input_bits(address, 1, arr);
    EXPECT_TRUE(arrays_match(arr_target2, arr));
}

TEST(modbus_test, modbus_master_multiple_write_coils)
{
    constexpr auto address = 0x22;
    test_slave     sl;
    test_master    ms;
    sl.add_observer(ms);
    ms.add_observer(sl);

    std::array<bool, 4> arr{};
    std::array<bool, 4> arr_target{};
    ms.read_bits(address, 0, arr);
    EXPECT_TRUE(arrays_match(arr_target, arr));

    std::array<bool, 4> arr_target2{false, true, false, true};
    ms.write_bits(address, 0, arr_target2);
    ms.read_input_bits(address, 0, arr);
    EXPECT_TRUE(arrays_match(arr_target2, arr));
}

TEST(modbus_test, modbus_master_exception_status)
{
    constexpr auto address = 0x22;
    test_slave     sl;
    test_master    ms;
    sl.add_observer(ms);
    ms.add_observer(sl);

    std::uint8_t val;
    ms.read_exception(address, val);
    EXPECT_TRUE(val == 0x55);
}

TEST(modbus_test, modbus_master_diagnostics)
{
    constexpr auto address = 0x22;
    test_slave     sl;
    test_master    ms;
    sl.add_observer(ms);
    ms.add_observer(sl);

    std::array<std::uint16_t, 3> arr{0, 0, 0};
    ms.diagnostics<diagnostics_sub_function::return_query_data>(address, arr);
    ms.diagnostics<diagnostics_sub_function::return_bus_message_count>(address, arr);
    EXPECT_TRUE(arr[0] == 2);
    arr[0] = 0;
    ms.diagnostics<diagnostics_sub_function::restart_comm_option>(address, arr);
    ms.diagnostics<diagnostics_sub_function::return_diagnostic_register>(address, arr);
    ms.diagnostics<diagnostics_sub_function::return_bus_message_count>(address, arr);
    EXPECT_TRUE(arr[0] == 2);
}

TEST(modbus_test, modbus_master_fifo)
{
    constexpr auto address = 0x22;
    test_slave     sl;
    test_master    ms;
    sl.add_observer(ms);
    ms.add_observer(sl);

    std::array<std::uint16_t, 3> arr{3, 4, 5};
    sl << arr;
    loveka::components::utils::circular_buffer<uint16_t, 16> buffer;
    ms.read_fifo_queue(address, 0x01, buffer);
    EXPECT_TRUE(buffer.front() == 4);
}
