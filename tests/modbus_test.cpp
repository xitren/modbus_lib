#include "xitren/modbus/master.hpp"
#include "xitren/modbus/slave.hpp"
#include <xitren/circular_buffer.hpp>
#include <xitren/comm/observer.hpp>
#include <xitren/modbus/crc16ansi.hpp>
#include <xitren/modbus/functions/diagnostics.hpp>
#include <xitren/modbus/functions/identification.hpp>
#include <xitren/modbus/functions/read_coils.hpp>
#include <xitren/modbus/functions/read_holding.hpp>
#include <xitren/modbus/functions/read_input_regs.hpp>
#include <xitren/modbus/functions/read_inputs.hpp>
#include <xitren/modbus/functions/read_log.hpp>
#include <xitren/modbus/functions/set_max_log_level.hpp>
#include <xitren/modbus/functions/write_coils.hpp>
#include <xitren/modbus/functions/write_registers.hpp>
#include <xitren/modbus/functions/write_single_coil.hpp>
#include <xitren/modbus/functions/write_single_register.hpp>
#include <xitren/modbus/log/embedded.hpp>
#include <xitren/modbus/packet.hpp>

#include <gtest/gtest.h>

#include <array>
#include <limits>
#include <vector>

using namespace xitren::modbus;

namespace {
using xitren::func::msb_t;

template <typename MsgType, typename Header, typename Fields, typename Type>
bool fill_message(MsgType& msg, Header const& header, Fields const& fields, std::size_t count, Type const* data)
{
    typename MsgType::template fields_in<Header, Fields, Type> input{header, fields, count, data};
    return msg.template serialize<Header, Fields, Type, crc16ansi>(input);
}
}    // namespace

using slave_type      = slave<10, 10, 10, 10, 64>;
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
    test_slave() : slave(0x22) { exception_status_ = 0x55; }

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
using custom_slave_type = slave_base<custom_slave_bits_type, custom_slave_bits_type, custom_slave_registers_type,
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
    test_custom_slave() : slave_base(0x22, bits_, bits_, registers_, registers_) { exception_status_ = 0x55; }

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

class test_master : public master, public observer_type, public observable_type {

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

class simple_slave : public slave_type {
    bool
    send(msg_type::array_type::iterator, msg_type::array_type::iterator) noexcept override
    {
        return true;
    }

public:
    explicit simple_slave(std::uint8_t id = 0x22) : ::slave_type(id) { exception_status_ = 0x55; }
};

class restart_slave : public slave_type {
    bool
    send(msg_type::array_type::iterator, msg_type::array_type::iterator) noexcept override
    {
        return true;
    }

    void
    restart_comm() noexcept override
    {
        restart_called_ = true;
    }

public:
    explicit restart_slave(std::uint8_t id = 0x22) : ::slave_type(id) { exception_status_ = 0x55; }

    bool restart_called() const noexcept
    {
        return restart_called_;
    }

private:
    bool restart_called_{};
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

TEST(modbus_functions, diagnostics_and_identification)
{
    restart_slave slave{};
    msb_t<std::uint16_t> data{0};

    ASSERT_TRUE(fill_message(slave.input(),
                             header{slave.id(), static_cast<std::uint8_t>(function::diagnostic)},
                             msb_t<std::uint16_t>{
                                 static_cast<std::uint16_t>(diagnostics_sub_function::restart_comm_option)},
                             1,
                             &data));
    EXPECT_EQ(functions::diagnostics(slave), exception::no_error);
    EXPECT_TRUE(slave.restart_called());

    ASSERT_TRUE(fill_message(slave.input(),
                             header{slave.id(), static_cast<std::uint8_t>(function::diagnostic)},
                             msb_t<std::uint16_t>{
                                 static_cast<std::uint16_t>(diagnostics_sub_function::return_query_data)},
                             1,
                             &data));
    EXPECT_EQ(functions::diagnostics(slave), exception::no_error);
    EXPECT_EQ(slave.output().size(), slave.input().size());

    ASSERT_TRUE(fill_message(slave.input(),
                             header{slave.id(), static_cast<std::uint8_t>(function::diagnostic)},
                             msb_t<std::uint16_t>{
                                 static_cast<std::uint16_t>(diagnostics_sub_function::return_diagnostic_register)},
                             1,
                             &data));
    EXPECT_EQ(functions::diagnostics(slave), exception::no_error);

    ASSERT_TRUE(fill_message(slave.input(),
                             header{slave.id(), static_cast<std::uint8_t>(function::diagnostic)},
                             msb_t<std::uint16_t>{
                                 static_cast<std::uint16_t>(diagnostics_sub_function::return_bus_message_count)},
                             1,
                             &data));
    EXPECT_EQ(functions::diagnostics(slave), exception::no_error);

    for (auto sub : {diagnostics_sub_function::return_bus_comm_error_count,
                     diagnostics_sub_function::return_server_exception_error_count,
                     diagnostics_sub_function::return_server_message_count,
                     diagnostics_sub_function::return_server_no_response_count,
                     diagnostics_sub_function::return_server_nak_count,
                     diagnostics_sub_function::return_server_busy_count,
                     diagnostics_sub_function::return_bus_char_overrun_count}) {
        ASSERT_TRUE(fill_message(slave.input(),
                                 header{slave.id(), static_cast<std::uint8_t>(function::diagnostic)},
                                 msb_t<std::uint16_t>{static_cast<std::uint16_t>(sub)},
                                 1,
                                 &data));
        EXPECT_EQ(functions::diagnostics(slave), exception::no_error);
    }

    ASSERT_TRUE(fill_message(slave.input(),
                             header{slave.id(), static_cast<std::uint8_t>(function::diagnostic)},
                             msb_t<std::uint16_t>{
                                 static_cast<std::uint16_t>(diagnostics_sub_function::force_listen_only_mode)},
                             1,
                             &data));
    EXPECT_EQ(functions::diagnostics(slave), exception::no_error);

    ASSERT_TRUE(fill_message(slave.input(),
                             header{slave.id(), static_cast<std::uint8_t>(function::diagnostic)},
                             msb_t<std::uint16_t>{
                                 static_cast<std::uint16_t>(diagnostics_sub_function::clear_counters)},
                             1,
                             &data));
    EXPECT_EQ(functions::diagnostics(slave), exception::no_error);

    ASSERT_TRUE(fill_message(slave.input(),
                             header{slave.id(), static_cast<std::uint8_t>(function::diagnostic)},
                             msb_t<std::uint16_t>{0xFFFF},
                             1,
                             &data));
    EXPECT_EQ(functions::diagnostics(slave), exception::illegal_function);

    request_identification req_vendor{modbus_base::mei_type,
                                      static_cast<std::uint8_t>(read_device_id_code::individual_access),
                                      static_cast<std::uint8_t>(object_id_code::vendor_name)};
    ASSERT_TRUE(fill_message(slave.input(),
                             header{slave.id(), static_cast<std::uint8_t>(function::read_device_identification)},
                             req_vendor,
                             0,
                             static_cast<std::uint8_t const*>(nullptr)));
    EXPECT_EQ(functions::identification(slave), exception::no_error);

    request_identification req_product{modbus_base::mei_type,
                                       static_cast<std::uint8_t>(read_device_id_code::individual_access),
                                       static_cast<std::uint8_t>(object_id_code::product_code)};
    ASSERT_TRUE(fill_message(slave.input(),
                             header{slave.id(), static_cast<std::uint8_t>(function::read_device_identification)},
                             req_product,
                             0,
                             static_cast<std::uint8_t const*>(nullptr)));
    EXPECT_EQ(functions::identification(slave), exception::no_error);

    request_identification req_revision{modbus_base::mei_type,
                                        static_cast<std::uint8_t>(read_device_id_code::individual_access),
                                        static_cast<std::uint8_t>(object_id_code::major_minor_revision)};
    ASSERT_TRUE(fill_message(slave.input(),
                             header{slave.id(), static_cast<std::uint8_t>(function::read_device_identification)},
                             req_revision,
                             0,
                             static_cast<std::uint8_t const*>(nullptr)));
    EXPECT_EQ(functions::identification(slave), exception::no_error);

    request_identification bad_mei{0xFF,
                                   static_cast<std::uint8_t>(read_device_id_code::individual_access),
                                   static_cast<std::uint8_t>(object_id_code::vendor_name)};
    ASSERT_TRUE(fill_message(slave.input(),
                             header{slave.id(), static_cast<std::uint8_t>(function::read_device_identification)},
                             bad_mei,
                             0,
                             static_cast<std::uint8_t const*>(nullptr)));
    EXPECT_EQ(functions::identification(slave), exception::illegal_data_value);

    request_identification bad_mode{modbus_base::mei_type, 0xFF,
                                    static_cast<std::uint8_t>(object_id_code::vendor_name)};
    ASSERT_TRUE(fill_message(slave.input(),
                             header{slave.id(), static_cast<std::uint8_t>(function::read_device_identification)},
                             bad_mode,
                             0,
                             static_cast<std::uint8_t const*>(nullptr)));
    EXPECT_EQ(functions::identification(slave), exception::illegal_data_value);

    request_identification bad_id{modbus_base::mei_type,
                                  static_cast<std::uint8_t>(read_device_id_code::individual_access),
                                  static_cast<std::uint8_t>(object_id_code::max)};
    ASSERT_TRUE(fill_message(slave.input(),
                             header{slave.id(), static_cast<std::uint8_t>(function::read_device_identification)},
                             bad_id,
                             0,
                             static_cast<std::uint8_t const*>(nullptr)));
    EXPECT_EQ(functions::identification(slave), exception::illegal_data_address);
}

TEST(modbus_functions, read_write_validation)
{
    simple_slave slave{};

    request_fields_read read_fields{msb_t<std::uint16_t>{0}, msb_t<std::uint16_t>{0}};
    ASSERT_TRUE(fill_message(slave.input(),
                             header{slave.id(), static_cast<std::uint8_t>(function::read_coils)},
                             read_fields,
                             0,
                             static_cast<std::uint8_t const*>(nullptr)));
    EXPECT_EQ(functions::read_coils(slave), exception::illegal_data_value);

    request_fields_read too_many_bits{msb_t<std::uint16_t>{0},
                                      msb_t<std::uint16_t>{modbus_base::max_read_bits + 1}};
    ASSERT_TRUE(fill_message(slave.input(),
                             header{slave.id(), static_cast<std::uint8_t>(function::read_coils)},
                             too_many_bits,
                             0,
                             static_cast<std::uint8_t const*>(nullptr)));
    EXPECT_EQ(functions::read_coils(slave), exception::illegal_data_value);

    slave.coils()[0] = true;
    request_fields_read good_bits{msb_t<std::uint16_t>{0}, msb_t<std::uint16_t>{1}};
    ASSERT_TRUE(fill_message(slave.input(),
                             header{slave.id(), static_cast<std::uint8_t>(function::read_coils)},
                             good_bits,
                             0,
                             static_cast<std::uint8_t const*>(nullptr)));
    EXPECT_EQ(functions::read_coils(slave), exception::no_error);

    for (std::size_t i = 0; i < 8; ++i) {
        slave.coils()[i] = (i % 2) == 0;
    }
    request_fields_read good_bits_aligned{msb_t<std::uint16_t>{0}, msb_t<std::uint16_t>{8}};
    ASSERT_TRUE(fill_message(slave.input(),
                             header{slave.id(), static_cast<std::uint8_t>(function::read_coils)},
                             good_bits_aligned,
                             0,
                             static_cast<std::uint8_t const*>(nullptr)));
    EXPECT_EQ(functions::read_coils(slave), exception::no_error);

    ASSERT_TRUE(fill_message(slave.input(),
                             header{slave.id(), static_cast<std::uint8_t>(function::read_coils)},
                             read_fields,
                             0,
                             static_cast<std::uint8_t const*>(nullptr)));
    slave.input().size(0);
    EXPECT_EQ(functions::read_coils(slave), exception::bad_data);

    request_fields_read bad_qty{msb_t<std::uint16_t>{0}, msb_t<std::uint16_t>{modbus_base::max_read_bits + 1}};
    ASSERT_TRUE(fill_message(slave.input(),
                             header{slave.id(), static_cast<std::uint8_t>(function::read_discrete_inputs)},
                             bad_qty,
                             0,
                             static_cast<std::uint8_t const*>(nullptr)));
    EXPECT_EQ(functions::read_inputs(slave), exception::illegal_data_value);

    ASSERT_TRUE(fill_message(slave.input(),
                             header{slave.id(), static_cast<std::uint8_t>(function::read_discrete_inputs)},
                             read_fields,
                             0,
                             static_cast<std::uint8_t const*>(nullptr)));
    slave.input().size(0);
    EXPECT_EQ(functions::read_inputs(slave), exception::bad_data);

    slave.inputs()[0] = true;
    ASSERT_TRUE(fill_message(slave.input(),
                             header{slave.id(), static_cast<std::uint8_t>(function::read_discrete_inputs)},
                             good_bits,
                             0,
                             static_cast<std::uint8_t const*>(nullptr)));
    EXPECT_EQ(functions::read_inputs(slave), exception::no_error);

    for (std::size_t i = 0; i < 8; ++i) {
        slave.inputs()[i] = (i % 2) == 1;
    }
    ASSERT_TRUE(fill_message(slave.input(),
                             header{slave.id(), static_cast<std::uint8_t>(function::read_discrete_inputs)},
                             good_bits_aligned,
                             0,
                             static_cast<std::uint8_t const*>(nullptr)));
    EXPECT_EQ(functions::read_inputs(slave), exception::no_error);

    request_fields_read bad_addr_bits{msb_t<std::uint16_t>{1000}, msb_t<std::uint16_t>{1}};
    ASSERT_TRUE(fill_message(slave.input(),
                             header{slave.id(), static_cast<std::uint8_t>(function::read_coils)},
                             bad_addr_bits,
                             0,
                             static_cast<std::uint8_t const*>(nullptr)));
    EXPECT_EQ(functions::read_coils(slave), exception::illegal_data_address);

    ASSERT_TRUE(fill_message(slave.input(),
                             header{slave.id(), static_cast<std::uint8_t>(function::read_discrete_inputs)},
                             bad_addr_bits,
                             0,
                             static_cast<std::uint8_t const*>(nullptr)));
    EXPECT_EQ(functions::read_inputs(slave), exception::illegal_data_address);

    request_fields_read zero_qty{msb_t<std::uint16_t>{0}, msb_t<std::uint16_t>{0}};
    ASSERT_TRUE(fill_message(slave.input(),
                             header{slave.id(), static_cast<std::uint8_t>(function::read_discrete_inputs)},
                             zero_qty,
                             0,
                             static_cast<std::uint8_t const*>(nullptr)));
    EXPECT_EQ(functions::read_inputs(slave), exception::illegal_data_value);

    request_fields_read bad_addr_regs{msb_t<std::uint16_t>{1000}, msb_t<std::uint16_t>{1}};
    ASSERT_TRUE(fill_message(slave.input(),
                             header{slave.id(), static_cast<std::uint8_t>(function::read_holding_registers)},
                             bad_addr_regs,
                             0,
                             static_cast<std::uint8_t const*>(nullptr)));
    EXPECT_EQ(functions::read_holding(slave), exception::illegal_data_address);

    ASSERT_TRUE(fill_message(slave.input(),
                             header{slave.id(), static_cast<std::uint8_t>(function::read_holding_registers)},
                             zero_qty,
                             0,
                             static_cast<std::uint8_t const*>(nullptr)));
    EXPECT_EQ(functions::read_holding(slave), exception::illegal_data_value);

    request_fields_read too_many_regs{msb_t<std::uint16_t>{0},
                                      msb_t<std::uint16_t>{modbus_base::max_read_registers + 1}};
    ASSERT_TRUE(fill_message(slave.input(),
                             header{slave.id(), static_cast<std::uint8_t>(function::read_holding_registers)},
                             too_many_regs,
                             0,
                             static_cast<std::uint8_t const*>(nullptr)));
    EXPECT_EQ(functions::read_holding(slave), exception::illegal_data_value);

    slave.holding_registers()[0] = 0x1111;
    request_fields_read good_holding{msb_t<std::uint16_t>{0}, msb_t<std::uint16_t>{1}};
    ASSERT_TRUE(fill_message(slave.input(),
                             header{slave.id(), static_cast<std::uint8_t>(function::read_holding_registers)},
                             good_holding,
                             0,
                             static_cast<std::uint8_t const*>(nullptr)));
    EXPECT_EQ(functions::read_holding(slave), exception::no_error);

    ASSERT_TRUE(fill_message(slave.input(),
                             header{slave.id(), static_cast<std::uint8_t>(function::read_input_registers)},
                             bad_addr_regs,
                             0,
                             static_cast<std::uint8_t const*>(nullptr)));
    EXPECT_EQ(functions::read_input_regs(slave), exception::illegal_data_address);

    ASSERT_TRUE(fill_message(slave.input(),
                             header{slave.id(), static_cast<std::uint8_t>(function::read_input_registers)},
                             zero_qty,
                             0,
                             static_cast<std::uint8_t const*>(nullptr)));
    EXPECT_EQ(functions::read_input_regs(slave), exception::illegal_data_value);

    ASSERT_TRUE(fill_message(slave.input(),
                             header{slave.id(), static_cast<std::uint8_t>(function::read_input_registers)},
                             too_many_regs,
                             0,
                             static_cast<std::uint8_t const*>(nullptr)));
    EXPECT_EQ(functions::read_input_regs(slave), exception::illegal_data_value);

    slave.input_registers()[0] = 0x1234;
    request_fields_read good_input_regs{msb_t<std::uint16_t>{0}, msb_t<std::uint16_t>{1}};
    ASSERT_TRUE(fill_message(slave.input(),
                             header{slave.id(), static_cast<std::uint8_t>(function::read_input_registers)},
                             good_input_regs,
                             0,
                             static_cast<std::uint8_t const*>(nullptr)));
    EXPECT_EQ(functions::read_input_regs(slave), exception::no_error);

    ASSERT_TRUE(fill_message(slave.input(),
                             header{slave.id(), static_cast<std::uint8_t>(function::read_input_registers)},
                             good_input_regs,
                             0,
                             static_cast<std::uint8_t const*>(nullptr)));
    slave.input().size(0);
    EXPECT_EQ(functions::read_input_regs(slave), exception::bad_data);

    request_fields_read bad_write_coil_value{msb_t<std::uint16_t>{0}, msb_t<std::uint16_t>{0x1234}};
    ASSERT_TRUE(fill_message(slave.input(),
                             header{slave.id(), static_cast<std::uint8_t>(function::write_single_coil)},
                             bad_write_coil_value,
                             0,
                             static_cast<std::uint8_t const*>(nullptr)));
    EXPECT_EQ(functions::write_single_coil(slave), exception::illegal_data_value);

    request_fields_read bad_write_coil_addr{msb_t<std::uint16_t>{1000}, msb_t<std::uint16_t>{modbus_base::on_coil_value}};
    ASSERT_TRUE(fill_message(slave.input(),
                             header{slave.id(), static_cast<std::uint8_t>(function::write_single_coil)},
                             bad_write_coil_addr,
                             0,
                             static_cast<std::uint8_t const*>(nullptr)));
    EXPECT_EQ(functions::write_single_coil(slave), exception::illegal_data_address);

    request_fields_read good_write_coil{msb_t<std::uint16_t>{0}, msb_t<std::uint16_t>{modbus_base::on_coil_value}};
    ASSERT_TRUE(fill_message(slave.input(),
                             header{slave.id(), static_cast<std::uint8_t>(function::write_single_coil)},
                             good_write_coil,
                             0,
                             static_cast<std::uint8_t const*>(nullptr)));
    EXPECT_EQ(functions::write_single_coil(slave), exception::no_error);

    request_fields_read good_write_coil_off{msb_t<std::uint16_t>{0}, msb_t<std::uint16_t>{modbus_base::off_coil_value}};
    ASSERT_TRUE(fill_message(slave.input(),
                             header{slave.id(), static_cast<std::uint8_t>(function::write_single_coil)},
                             good_write_coil_off,
                             0,
                             static_cast<std::uint8_t const*>(nullptr)));
    EXPECT_EQ(functions::write_single_coil(slave), exception::no_error);

    ASSERT_TRUE(fill_message(slave.input(),
                             header{slave.id(), static_cast<std::uint8_t>(function::write_single_coil)},
                             read_fields,
                             0,
                             static_cast<std::uint8_t const*>(nullptr)));
    slave.input().size(0);
    EXPECT_EQ(functions::write_single_coil(slave), exception::bad_data);

    ASSERT_TRUE(fill_message(slave.input(),
                             header{slave.id(), static_cast<std::uint8_t>(function::write_single_register)},
                             bad_addr_regs,
                             0,
                             static_cast<std::uint8_t const*>(nullptr)));
    EXPECT_EQ(functions::write_single_register(slave), exception::illegal_data_address);

    request_fields_read good_write_reg{msb_t<std::uint16_t>{0}, msb_t<std::uint16_t>{0x55AA}};
    ASSERT_TRUE(fill_message(slave.input(),
                             header{slave.id(), static_cast<std::uint8_t>(function::write_single_register)},
                             good_write_reg,
                             0,
                             static_cast<std::uint8_t const*>(nullptr)));
    EXPECT_EQ(functions::write_single_register(slave), exception::no_error);

    ASSERT_TRUE(fill_message(slave.input(),
                             header{slave.id(), static_cast<std::uint8_t>(function::write_single_register)},
                             read_fields,
                             0,
                             static_cast<std::uint8_t const*>(nullptr)));
    slave.input().size(0);
    EXPECT_EQ(functions::write_single_register(slave), exception::bad_data);

    request_fields_wr_multi bad_write_regs{msb_t<std::uint16_t>{1000}, msb_t<std::uint16_t>{1}, 2};
    ASSERT_TRUE(fill_message(slave.input(),
                             header{slave.id(), static_cast<std::uint8_t>(function::write_multiple_registers)},
                             bad_write_regs,
                             0,
                             static_cast<std::uint8_t const*>(nullptr)));
    EXPECT_EQ(functions::write_registers(slave), exception::illegal_data_address);

    std::array<msb_t<std::uint16_t>, 1> write_regs_data{msb_t<std::uint16_t>{0x0102}};
    request_fields_wr_multi good_write_regs{msb_t<std::uint16_t>{0}, msb_t<std::uint16_t>{1}, 2};
    ASSERT_TRUE(fill_message(slave.input(),
                             header{slave.id(), static_cast<std::uint8_t>(function::write_multiple_registers)},
                             good_write_regs,
                             write_regs_data.size(),
                             write_regs_data.data()));
    EXPECT_EQ(functions::write_registers(slave), exception::no_error);

    request_fields_wr_single bad_write_coils{msb_t<std::uint16_t>{1000}, msb_t<std::uint16_t>{1}, 1};
    ASSERT_TRUE(fill_message(slave.input(),
                             header{slave.id(), static_cast<std::uint8_t>(function::write_multiple_coils)},
                             bad_write_coils,
                             0,
                             static_cast<std::uint8_t const*>(nullptr)));
    EXPECT_EQ(functions::write_coils(slave), exception::illegal_data_address);

    request_fields_wr_single bad_write_coils_count{msb_t<std::uint16_t>{0}, msb_t<std::uint16_t>{1}, 2};
    ASSERT_TRUE(fill_message(slave.input(),
                             header{slave.id(), static_cast<std::uint8_t>(function::write_multiple_coils)},
                             bad_write_coils_count,
                             0,
                             static_cast<std::uint8_t const*>(nullptr)));
    EXPECT_EQ(functions::write_coils(slave), exception::illegal_data_value);

    request_fields_wr_single zero_write_coils{msb_t<std::uint16_t>{0}, msb_t<std::uint16_t>{0}, 0};
    ASSERT_TRUE(fill_message(slave.input(),
                             header{slave.id(), static_cast<std::uint8_t>(function::write_multiple_coils)},
                             zero_write_coils,
                             0,
                             static_cast<std::uint8_t const*>(nullptr)));
    EXPECT_EQ(functions::write_coils(slave), exception::illegal_data_value);

    request_fields_wr_single too_many_coils{msb_t<std::uint16_t>{0},
                                            msb_t<std::uint16_t>{modbus_base::max_write_bits + 1},
                                            0};
    ASSERT_TRUE(fill_message(slave.input(),
                             header{slave.id(), static_cast<std::uint8_t>(function::write_multiple_coils)},
                             too_many_coils,
                             0,
                             static_cast<std::uint8_t const*>(nullptr)));
    EXPECT_EQ(functions::write_coils(slave), exception::illegal_data_value);

    std::array<std::uint8_t, 1> write_coils_data{0x01};
    request_fields_wr_single good_write_coils{msb_t<std::uint16_t>{0}, msb_t<std::uint16_t>{1}, 1};
    ASSERT_TRUE(fill_message(slave.input(),
                             header{slave.id(), static_cast<std::uint8_t>(function::write_multiple_coils)},
                             good_write_coils,
                             write_coils_data.size(),
                             write_coils_data.data()));
    EXPECT_EQ(functions::write_coils(slave), exception::no_error);

    std::array<std::uint8_t, 1> write_coils_aligned{0xAA};
    request_fields_wr_single good_write_coils_aligned{msb_t<std::uint16_t>{0}, msb_t<std::uint16_t>{8}, 1};
    ASSERT_TRUE(fill_message(slave.input(),
                             header{slave.id(), static_cast<std::uint8_t>(function::write_multiple_coils)},
                             good_write_coils_aligned,
                             write_coils_aligned.size(),
                             write_coils_aligned.data()));
    EXPECT_EQ(functions::write_coils(slave), exception::no_error);

    ASSERT_TRUE(fill_message(slave.input(),
                             header{slave.id(), static_cast<std::uint8_t>(function::set_max_log_level)},
                             std::uint8_t{static_cast<std::uint8_t>(LOG_LEVEL_CRITICAL + 1)},
                             0,
                             static_cast<std::uint8_t const*>(nullptr)));
    EXPECT_EQ(functions::set_max_log_level(slave), exception::bad_data);

    ASSERT_TRUE(fill_message(slave.input(),
                             header{slave.id(), static_cast<std::uint8_t>(function::set_max_log_level)},
                             std::uint8_t{LOG_LEVEL_INFO},
                             0,
                             static_cast<std::uint8_t const*>(nullptr)));
    EXPECT_EQ(functions::set_max_log_level(slave), exception::no_error);
}

TEST(modbus_functions, read_log_and_embedded_log)
{
    simple_slave slave{};
    std::array<std::uint8_t, 4> log_data{1, 2, 3, 4};
    slave.to_log(log_data);

    request_fields_log log_req{msb_t<std::uint16_t>{0}, msb_t<std::uint16_t>{4}};
    ASSERT_TRUE(fill_message(slave.input(),
                             header{slave.id(), static_cast<std::uint8_t>(function::read_log)},
                             log_req,
                             0,
                             static_cast<std::uint8_t const*>(nullptr)));
    EXPECT_EQ(functions::read_log(slave), exception::no_error);

    request_fields_log log_req_out{msb_t<std::uint16_t>{999}, msb_t<std::uint16_t>{2}};
    ASSERT_TRUE(fill_message(slave.input(),
                             header{slave.id(), static_cast<std::uint8_t>(function::read_log)},
                             log_req_out,
                             0,
                             static_cast<std::uint8_t const*>(nullptr)));
    EXPECT_EQ(functions::read_log(slave), exception::no_error);

    for (std::size_t i = 0; i < log::log_size + 2; ++i) {
        slave.log().push(static_cast<std::uint8_t>(i & 0xFF));
    }
    request_fields_log log_req_wrap{msb_t<std::uint16_t>{0}, msb_t<std::uint16_t>{1}};
    ASSERT_TRUE(fill_message(slave.input(),
                             header{slave.id(), static_cast<std::uint8_t>(function::read_log)},
                             log_req_wrap,
                             0,
                             static_cast<std::uint8_t const*>(nullptr)));
    EXPECT_EQ(functions::read_log(slave), exception::no_error);


    ASSERT_TRUE(fill_message(slave.input(),
                             header{slave.id(), static_cast<std::uint8_t>(function::read_log)},
                             log_req,
                             0,
                             static_cast<std::uint8_t const*>(nullptr)));
    slave.input().size(0);
    EXPECT_EQ(functions::read_log(slave), exception::bad_data);

    xitren::circular_buffer<std::uint8_t, log::log_size> sink{};
    log::embedded::register_sink(sink);
    log::embedded::set_current_lvl(LOG_LEVEL_CRITICAL);

    log::embedded logger(LOG_LEVEL_INFO);
    logger << "hello";
    logger << 42;

    log::embedded::unregister_sink();
    EXPECT_NE(sink.size(), 0U);
}
