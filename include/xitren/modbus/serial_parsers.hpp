#pragma once

#include <loveka/components/hal/usart_base.hpp>
#include <loveka/components/modbus/packet.hpp>
#include <loveka/components/utils/observer.hpp>

#include <variant>
#include <vector>

namespace loveka::components::modbus {

template <typename Header, typename Fields, typename Crc, std::size_t Queue, std::size_t RxSize,
          std::size_t TxSize>
class simple_serial_parser : public hal::usart_base<RxSize, TxSize>,
                             public utils::observable<Fields> {
    using size_type   = std::size_t;
    using packet_type = packet<Header, Fields, Crc>;
    using buffer_type = utils::circular_buffer<packet_type, Queue>;

public:
    template <class Field>
    void
    send(const Field& field)
    {
        packet_type pack(Field::header, field);
        hal::usart_base<RxSize, TxSize>::write(pack.to_array());
        sended_packets_++;
    }

    size_type
    size() noexcept
    {
        return rx_msgs_.size();
    }

    packet_type&
    pop() noexcept
    {
        auto& ret = *rx_msgs_.begin();
        rx_msgs_.pop_front();
        return ret;
    }

    void
    receive()
    {
        const data<Header> head{Fields::header};
        size_type          i = 0;
        while ((hal::usart_base<RxSize, TxSize>::rx_buffer_.size() - i)
               >= packet_type::packet_length) {
            if (hal::usart_base<RxSize, TxSize>::rx_buffer_ == head.pure) {
                auto pack
                    = packet_type::deserialize(hal::usart_base<RxSize, TxSize>::rx_buffer_.begin());
                if (pack.valid()) {
                    hal::usart_base<RxSize, TxSize>::rx_buffer_ >> (i + packet_type::packet_length);
                    unexpected_bytes_ += i;
                    i = 0;
                    rx_msgs_.push_back(pack);
                    found_packets_++;
                    utils::observable<Fields>::notify_observers(pack.get_fields());
                    continue;
                } else {
                    hal::usart_base<RxSize, TxSize>::rx_buffer_ >> i;
                    i = 0;
                    missed_packets_++;
                }
            }
            i++;
        }
    }

    [[nodiscard]] size_type
    unexpected_bytes() const
    {
        return unexpected_bytes_;
    }

    [[nodiscard]] size_type
    missed_packets() const
    {
        return missed_packets_;
    }

    [[nodiscard]] size_type
    found_packets() const
    {
        return found_packets_;
    }

    [[nodiscard]] size_type
    sended_packets() const
    {
        return sended_packets_;
    }

private:
    buffer_type rx_msgs_;
    // Counter for noise bytes or bytes not belongs to any packet
    size_type unexpected_bytes_ = 0;
    size_type missed_packets_   = 0;
    size_type found_packets_    = 0;
    size_type sended_packets_   = 0;
};

template <std::size_t Queue, std::size_t RxSize, std::size_t TxSize, typename Header, typename Crc,
          typename... Fields>
class serial_parser : public hal::usart_base<RxSize, TxSize>,
                      public utils::observable<std::variant<Fields...>> {
    using tuple_t                 = std::tuple<Fields...>;
    using size_type               = std::size_t;
    static constexpr size_type sz = sizeof...(Fields);
    static_assert(std::variant_size_v<std::variant<Fields...>> == sz);
    enum class state_receiver { not_found, found, no_data };

public:
    using event_t     = std::variant<Fields...>;
    using buffer_type = utils::circular_buffer<event_t, Queue>;

    explicit constexpr serial_parser() : headers_{Fields::header...} {}

    template <class Field>
    void
    send_packet(const Field& field)
    {
        packet<Header, Field, Crc>       pack(Field::header, field);
        hal::usart_base<RxSize, TxSize>::operator<<(pack.to_array());
        sended_packets_++;
    }

    void
    parse() noexcept
    {
        state_receiver state;
        while (((state = packet_receiver_helper<Fields...>::get_received_packet(*this, 0))
                != state_receiver::no_data)
               && (!hal::usart_base<RxSize, TxSize>::rx_buffer_.empty())) {
            if (state == state_receiver::not_found) {
                hal::usart_base<RxSize, TxSize>::rx_buffer_.pop();
                unexpected_bytes_++;
            }
        }
    }

    [[nodiscard]] size_type
    size() const noexcept
    {
        return rx_msg_.size();
    }

    event_t
    pop() noexcept
    {
        auto ret = rx_msg_.front();
        rx_msg_.pop();
        return ret;
    }

    [[nodiscard]] size_type
    unexpected_bytes() const
    {
        return unexpected_bytes_;
    }

    [[nodiscard]] size_type
    missed_packets() const
    {
        return missed_packets_;
    }

    [[nodiscard]] size_type
    found_packets() const
    {
        return found_packets_;
    }

    [[nodiscard]] size_type
    sended_packets() const
    {
        return sended_packets_;
    }

private:
    std::array<Header, sz> headers_;
    buffer_type            rx_msg_;
    // Counter for noise bytes or bytes not belongs to any packet
    size_type unexpected_bytes_ = 0;
    size_type missed_packets_   = 0;
    size_type found_packets_    = 0;
    size_type sended_packets_   = 0;

    template <typename First, typename... Args>
    struct packet_receiver_helper {
        using packet_type = modbus::packet<Header, First, Crc>;
        static constexpr state_receiver
        get_received_packet(serial_parser<Queue, RxSize, TxSize, Header, Crc, Fields...>& t,
                            size_type                                                     i)
        {
            const data<Header> header{t.headers_[i]};
            if (t.rx_buffer_ == header.pure) {
                if ((t.rx_buffer_.size()) >= packet_type::length) {
                    auto pack = packet_type::deserialize(t.rx_buffer().begin());
                    if (pack.valid()) {
                        t.rx_buffer_ >> packet_type::length;
                        t.rx_msg_.push(pack.fields());
                        t.found_packets_++;
                        t.notify_observers(pack.fields());
                        return state_receiver::found;
                    } else {
                        t.missed_packets_++;
                    }
                } else {
                    return state_receiver::no_data;
                }
            }
            return packet_receiver_helper<Args...>::get_received_packet(t, i + 1);
        }
    };
    template <typename First>
    struct packet_receiver_helper<First> {
        using packet_type = packet<Header, First, Crc>;
        static constexpr state_receiver
        get_received_packet(serial_parser<Queue, RxSize, TxSize, Header, Crc, Fields...>& t,
                            size_type                                                     i)
        {
            const data<Header> header{t.headers_[i]};
            if (t.rx_buffer_ == header.pure) {
                if ((t.rx_buffer_.size()) >= packet_type::length) {
                    auto pack = packet_type::deserialize(t.rx_buffer().begin());
                    if (pack.valid()) {
                        t.rx_buffer_ >> packet_type::length;
                        t.rx_msg_.push(pack.fields());
                        t.found_packets_++;
                        t.notify_observers(pack.fields());
                        return state_receiver::found;
                    } else {
                        t.missed_packets_++;
                    }
                } else {
                    return state_receiver::no_data;
                }
            }
            return state_receiver::not_found;
        }
    };
};

}    // namespace xitren::components::modbus
