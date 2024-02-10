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
    test_slave() : modbus_slave(0x20) { exception_status_ = 0x55; }

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
