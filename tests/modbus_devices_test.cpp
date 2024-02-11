#include <xitren/circular_buffer.hpp>
#include <xitren/comm/observer.hpp>
#include <xitren/func/interval_event.hpp>
#include <xitren/modbus/crc16ansi.hpp>
#include <xitren/modbus/master/modbus_command_bits.hpp>
#include <xitren/modbus/master/modbus_command_diagnostic.hpp>
#include <xitren/modbus/master/modbus_command_regs.hpp>
#include <xitren/modbus/master/modbus_command_regs_adv.hpp>
#include <xitren/modbus/master/modbus_master.hpp>
#include <xitren/modbus/packet.hpp>
#include <xitren/modbus/slave/modbus_slave.hpp>

#include <gtest/gtest.h>
#include <spdlog/spdlog.h>

using namespace xitren::modbus;
using namespace xitren;

class bus_singleton : public comm::observable_dynamic<modbus_base::msg_type> {
    using data_type       = modbus_base::msg_type;
    using observable_type = comm::observable_dynamic<data_type>;
    using observer_type   = comm::observer<data_type>&;
    using interval_type   = func::interval_event;

public:
    static inline auto&
    instance()
    {
        static bus_singleton bus;
        return bus;
    }

    static inline void
    bus_emit(observer_type emitter, modbus_base::msg_type const& data)
    {
        instance().remove_observer(emitter);
        instance().notify_observers(data);
        instance().add_observer(emitter);
    }

    bus_singleton&
    operator=(bus_singleton const& other)
        = delete;
    bus_singleton&
    operator=(bus_singleton const&& other)
        = delete;
    bus_singleton(bus_singleton const& val) = delete;
    bus_singleton(bus_singleton&& val)      = delete;
    ~bus_singleton() override               = default;

private:
    bus_singleton() = default;
};

class test_master : public modbus_master, public comm::observer<modbus_base::msg_type> {
    using data_type       = modbus_base::msg_type;
    using observable_type = comm::observable_dynamic<data_type>;
    using observer_type   = comm::observer<data_type>;

public:
    test_master() = default;

    void
    data(void const*, data_type const& nd) override
    {
        receive(nd.storage().data(), nd.storage().data() + nd.size());
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

private:
    bool
    send(msg_type::array_type::iterator begin, msg_type::array_type::iterator end) noexcept override
    {
        TRACE() << ">>>>>>>>>>>>>>Master output msg ";
        std::ostringstream hex;
        hex << std::noshowbase << std::internal << std::setfill('0');
        for (auto i{begin}; i < end; i++) {
            hex << std::hex << std::setw(2) << static_cast<int>(*i) << std::dec << " ";
        }
        TRACE() << hex.str();
        bus_singleton::bus_emit(*this, modbus_master::output());
        return true;
    }
};

template <typename T, size_t Size, size_t Size1>
bool
arrays_match(std::array<T, Size> const& expected, std::array<T, Size1> const& actual, std::size_t size)
{
    TRACE() << "===========Expected ";
    TRACE() << std::noshowbase << std::internal << std::setfill('0');
    for (size_t i{0}; i < size; ++i) {
        TRACE() << std::hex << std::setw(2) << static_cast<int>(expected[i]) << std::dec << " ";
    }
    for (size_t i{0}; i < size; ++i) {
        if (expected[i] != actual[i]) {
            TRACE() << "array[" << i << "] (" << actual[i] << ") != expected[" << i << "] (" << expected[i] << ")"
                    << std::endl;
            return false;
        }
    }
    return true;
}

using slave_type = modbus_slave<10, 10, 10, 10, 64>;

class test_slave : public slave_type, public comm::observer<modbus_base::msg_type> {
    using data_type       = modbus_base::msg_type;
    using observable_type = comm::observable_dynamic<data_type>;
    using observer_type   = comm::observer<data_type>;

public:
    bool
    send(msg_type::array_type::iterator begin, msg_type::array_type::iterator end) noexcept override
    {
        TRACE() << "<<<<<<<<<<<<<<Slave output msg ";
        std::ostringstream hex;
        hex << std::noshowbase << std::internal << std::setfill('0');
        for (auto i{begin}; i < end; i++) {
            hex << std::hex << std::setw(2) << static_cast<int>(*i) << std::dec << " ";
        }
        TRACE() << hex.str();
        bus_singleton::bus_emit(*this, slave_type::output());
        return true;
    }

    explicit test_slave(std::uint8_t id) : modbus_slave(id) {}

    void
    data(void const*, data_type const& nd) override
    {
        if (!idle()) {
            TRACE() << ">>>>>>>>>>>>>>Slave not in IDLE " << nd.size();
        }
        TRACE() << ">>>>>>>>>>>>>>Slave receive msg " << nd.size();
        receive(nd.storage().data(), nd.storage().data() + nd.size());
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

class bus_model : public func::interval_event {
    static constexpr auto period{2ms};
    static constexpr auto waiting_period{1ms};

public:
    bus_model() : func::interval_event{[&]() mutable { process(); }, period, waiting_period}
    {
        bus_singleton::instance().add_observer(master_);
        bus_singleton::instance().add_observer(imu_);
        bus_singleton::instance().add_observer(motor_fl_);
        bus_singleton::instance().add_observer(motor_fr_);
        bus_singleton::instance().add_observer(motor_rl_);
        bus_singleton::instance().add_observer(motor_rr_);
    }

    bus_model&
    operator=(bus_model const& other)
        = delete;
    bus_model&
    operator=(bus_model const&& other)
        = delete;
    bus_model(bus_model const& val) = delete;
    bus_model(bus_model&& val)      = delete;
    ~bus_model()
    {
        bus_singleton::instance().remove_observer(master_);
        bus_singleton::instance().remove_observer(imu_);
        bus_singleton::instance().remove_observer(motor_fl_);
        bus_singleton::instance().remove_observer(motor_fr_);
        bus_singleton::instance().remove_observer(motor_rl_);
        bus_singleton::instance().remove_observer(motor_rr_);
    }

    template <class T>
    void
    bus_send(T& cmd)
    {
        using namespace xitren::modbus;
        master_.run_async(cmd);
        std::atomic_flag answer{false};
        auto             timer = std::thread([&]() {
            for (auto i{0}; (i < 3); i++) {
                std::this_thread::sleep_for(period * 5);
                if (answer.test()) {
                    return;
                }
            }
            if (master_.state() != master_state::idle) {
                master_.timer_expired();
            }
        });
        while (master_.state() != master_state::idle) {
            std::this_thread::sleep_for(period * 2);
        }
        answer.test_and_set();
        timer.join();
    }

private:
    void
    process()
    {
        master_.processing();
        imu_.processing();
        motor_fl_.processing();
        motor_fr_.processing();
        motor_rl_.processing();
        motor_rr_.processing();
    }

    test_master master_;
    test_slave  imu_{0xAE};
    test_slave  motor_fl_{0xB2};
    test_slave  motor_fr_{0xE2};
    test_slave  motor_rl_{0x72};
    test_slave  motor_rr_{0x02};
};

TEST(modbus_devices_test, modbus_scanning)
{
    GTEST_SKIP();
    using namespace xitren::modbus;
    using namespace std::chrono;

    LEVEL(LOG_LEVEL_ERROR);
    std::vector<std::uint8_t> devices{};
    bus_model                 bus;

    fmt::print("    0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f\n");
    fmt::print("{0:2x}:-- ", 0);
    for (std::uint8_t id = 1; id != 0; ++id) {
        if (id % 16 == 0) {
            if (id != 0)
                fmt::print("\n");
            fmt::print("{0:2x}:", id);
        }
        if (id != 0) {
            bool data{true};
            auto callback = [&](exception err, bool* start, [[maybe_unused]] bool* end) {
                if ((start == nullptr) || (err != exception::no_error)) {
                    return;
                }
                data = false;
                fmt::print("{0:2x} ", id);
                devices.push_back(id);
            };
            read_input_bits read{id, 0, 0x1, callback};
            bus.bus_send(read);
            if (data) {
                fmt::print("-- ");
            }
        } else {
            fmt::print("-- ");
        }
    }
    fmt::print("\t");
    fmt::print("Available devices: [{0:2x}] \t", fmt::join(devices, ", "));
    EXPECT_TRUE(devices.size() == 5);
    for (auto& device : devices) {
        EXPECT_TRUE((device == 0x02) || (device == 0x72) || (device == 0xae) || (device == 0xb2) || (device == 0xe2));
    }
}

TEST(modbus_devices_test, modbus_write_read_registers)
{
    GTEST_SKIP();
    LEVEL(LOG_LEVEL_TRACE);
    bus_model bus;
    for (auto& id : std::array<std::uint8_t, 5>{0x02, 0x72, 0xae, 0xb2, 0xe2}) {
        std::array<std::uint16_t, 5> const values{0x02, 0x72, 0xae, 0xb2, 0xe2};
        bool                               data{false};
        auto                               callback = [&](exception err) {
            if (err != exception::no_error) {
                return;
            }
            data = true;
        };
        class write_registers writer {
            id, 1, values, callback
        };
        bus.bus_send(writer);
        EXPECT_TRUE(data);
        std::array<std::uint16_t, 5> recv_values{};
        data               = false;
        auto callback_read = [&](exception err, std::uint16_t* begin, std::uint16_t* end) {
            if ((err != exception::no_error) && ((end - begin) == static_cast<int>(recv_values.size()))) {
                return;
            }
            std::copy(begin, end, recv_values.begin());
            data = true;
        };
        class read_registers reader {
            id, 1, recv_values.size(), callback_read
        };
        bus.bus_send(reader);
        EXPECT_TRUE(data);
        EXPECT_EQ(recv_values, values);
    }
}

TEST(modbus_devices_test, modbus_write_read_coils)
{
    GTEST_SKIP();
    LEVEL(LOG_LEVEL_TRACE);
    bus_model bus;
    for (auto& id : std::array<std::uint8_t, 5>{0x02, 0x72, 0xae, 0xb2, 0xe2}) {
        std::array<bool, 5> const values{true, false, true, true, false};
        volatile bool             data{false};
        auto                      callback = [&](exception err) {
            if (err != exception::no_error) {
                return;
            }
            data = true;
        };
        class write_bits writer {
            id, 1, values, callback
        };
        bus.bus_send(writer);
        EXPECT_TRUE(data);
        std::array<bool, 5> recv_values{};
        data               = false;
        auto callback_read = [&](exception err, bool* begin, bool* end) {
            if ((err != exception::no_error) && ((end - begin) == static_cast<int>(recv_values.size()))) {
                return;
            }
            std::copy(begin, end, recv_values.begin());
            data = true;
        };
        class read_bits reader {
            id, 1, recv_values.size(), callback_read
        };
        bus.bus_send(reader);
        EXPECT_TRUE(data);
        EXPECT_EQ(recv_values, values);
    }
}
