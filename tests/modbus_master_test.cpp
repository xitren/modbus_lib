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
    timer_start(std::size_t microseconds) override
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
