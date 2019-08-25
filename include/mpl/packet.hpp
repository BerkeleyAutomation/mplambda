#pragma once
#ifndef MPL_PACKET_HPP
#define MPL_PACKET_HPP

#include "buffer.hpp"
#include <jilog.hpp>

namespace mpl::packet {

    using Type = std::uint32_t;
    using Size = std::uint32_t;

    static constexpr Type PROBLEM_SE3 = 0xdbb69672;
    static constexpr Type HELLO = 0x3864caca;

    static constexpr std::size_t MAX_PACKET_SIZE = 1024*1024;

    class protocol_error : public std::runtime_error {
    public:
        protocol_error(const std::string& msg)
            : std::runtime_error(msg)
        {
        }
    };

    template <class S>
    class ProblemSE3 {
        // do not use std::is_floating_point_v, since we don't support
        // long double (or any other thing that might qualify as a
        // floating point value)
        static_assert(std::is_same_v<S, float> || std::is_same_v<S, double>,
                      "S must be float or double");

        using State = std::tuple<Eigen::Quaternion<S>, Eigen::Matrix<S,3,1>>;
        using Bound = Eigen::Matrix<S, 3, 1>;

        static constexpr Type TYPE = PROBLEM_SE3 + sizeof(S)/8;
        
        State start_;
        State goal_;
        Bound min_;
        Bound max_;
        
    public:
        ProblemSE3(const State& start, const State& goal,
                   const Bound& min, const Bound& max)
            : start_(start)
            , goal_(goal)
            , min_(min)
            , max_(max)
        {
        }

        ProblemSE3(Type, Size, Buffer& buf)
            : start_{buf.get<State>()}
            , goal_{buf.get<State>()}
            , min_{buf.get<Bound>()}
            , max_{buf.get<Bound>()}
        {
        }

        operator Buffer () const {
            Size size = buffer_size_v<Type> +
                buffer_size_v<Size> +
                buffer_size_v<State>*2 +
                buffer_size_v<Bound>*2;
            
            Buffer buf{size};
            buf.put(TYPE);
            buf.put(size);
            buf.put(start_);
            buf.put(goal_);
            buf.put(min_);
            buf.put(max_);
            buf.flip();
            return buf;
        }

        const auto& start() const {
            return start_;
        }

        const auto& goal() const {
            return goal_;
        }

        const auto& min() const {
            return min_;
        }

        const auto& max() const {
            return max_;
        }
    };
    
    class Hello {
        std::uint64_t id_;
        
    public:
        explicit Hello(std::uint64_t id)
            : id_(id)
        {
        }

        explicit Hello(Type type, Size size, Buffer& buf)
            : id_(buf.get<std::uint64_t>())
        {
        }

        std::uint64_t id() const {
            return id_;
        }
        
        operator Buffer () const {
            Size size = 16;
            Buffer buf{size};
            buf.put(HELLO);
            buf.put(size);
            buf.put(id_);
            buf.flip();
            return buf;
        }
    };

    template <class Fn>
    std::size_t parse(Buffer& buf, Fn fn) {
        static constexpr auto head = buffer_size_v<Type> + buffer_size_v<Size>;
            
        if (buf.remaining() < head)
            return 8; // head - buf.remaining();

        // bounds checking
        char *start = buf.begin();

        Type type = buf.peek<Type>(0);
        Size size = buf.peek<Size>(buffer_size_v<Type>);

        if (size > MAX_PACKET_SIZE)
            throw protocol_error("maximum packet size exceeded: " + std::to_string(size));

        if (buf.remaining() < size) {
            JI_LOG(TRACE) << "short packet recv, have " << buf.remaining() << ", need " << size;
            return size; // size - buf.remaining();
        }

        buf += head;
        
        switch (type) {
        case HELLO:
            fn(Hello(type, size, buf));
            break;
        case PROBLEM_SE3:
            fn(ProblemSE3<float>(type, size, buf));
            break;
        case PROBLEM_SE3+1:
            fn(ProblemSE3<double>(type, size, buf));
            break;
        default:
            throw protocol_error("bad packet type: " + std::to_string(type));
        }

        assert(buf.begin() == start + size);
        
        return 0;
    }
}

#endif
