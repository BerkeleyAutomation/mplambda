#pragma once
#ifndef MPL_PACKET_HPP
#define MPL_PACKET_HPP

#include "buffer.hpp"
#include <jilog.hpp>

namespace mpl::packet {

    using Type = std::uint32_t;
    using Size = std::uint32_t;

    // hexdump -n 4 -e '"0x" 1 "%08x" "\n"' /dev/urandom 
    static constexpr Type PROBLEM_SE3 = 0xdbb69672;
    static constexpr Type HELLO = 0x3864caca;
    static constexpr Type PATH_SE3 = 0xa9cb6e7d;
    static constexpr Type DONE = 0x6672e31a;

    static constexpr std::size_t MAX_PACKET_SIZE = 1024*1024;

    static constexpr std::uint32_t ALGORITHM_RRT = 1;
    static constexpr std::uint32_t ALGORITHM_CFOREST = 2;
    
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

        std::uint32_t algorithm_;
        std::uint32_t timeLimitMillis_;

        double discretization_;

        std::string envMesh_;
        std::string robotMesh_;

    public:
        inline ProblemSE3(
            const std::string& envMesh,
            const std::string& robotMesh,
            const State& start, const State& goal,
            const Bound& min, const Bound& max,
            std::uint32_t algorithm,
            std::uint32_t timeLimitMillis,
            double discretization)
            : start_(start)
            , goal_(goal)
            , min_(min)
            , max_(max)
            , envMesh_(envMesh)
            , robotMesh_(robotMesh)
            , algorithm_(algorithm)
            , timeLimitMillis_(timeLimitMillis)
            , discretization_(discretization)
        {
        }

        inline ProblemSE3(Type, BufferView buf)
            : start_{buf.get<State>()}
            , goal_{buf.get<State>()}
            , min_{buf.get<Bound>()}
            , max_{buf.get<Bound>()}
            , algorithm_{buf.get<std::uint32_t>()}
            , timeLimitMillis_{buf.get<std::uint32_t>()}
            , discretization_{buf.get<double>()}
        {
            std::uint8_t len = buf.get<std::uint8_t>();
            if (len > buf.remaining())
                throw protocol_error("invalid string length in `problem` packet");
            
            envMesh_ = buf.getString(len);
            robotMesh_ = buf.getString();
        }

        inline operator Buffer () const {
            Size size =
                buffer_size_v<Type> +
                buffer_size_v<Size> +
                buffer_size_v<State>*2 +
                buffer_size_v<Bound>*2 +
                buffer_size_v<std::uint32_t>*2 +
                buffer_size_v<double> +
                1 + envMesh_.size() + robotMesh_.size();
            
            Buffer buf{size};
            buf.put(TYPE);
            buf.put(size);
            buf.put(start_);
            buf.put(goal_);
            buf.put(min_);
            buf.put(max_);
            buf.put(algorithm_);
            buf.put(timeLimitMillis_);
            buf.put(discretization_);
            buf.put(static_cast<std::uint8_t>(envMesh_.size()));
            buf.put(envMesh_);
            buf.put(robotMesh_);
            buf.flip();
            return buf;
        }

        inline const auto& envMesh() const {
            return envMesh_;
        }

        inline const auto& robotMesh() const {
            return robotMesh_;
        }

        inline const auto& start() const {
            return start_;
        }

        inline const auto& goal() const {
            return goal_;
        }

        inline const auto& min() const {
            return min_;
        }

        inline const auto& max() const {
            return max_;
        }

        inline auto algorithm() const {
            return algorithm_;
        }

        inline auto timeLimitMillis() const {
            return timeLimitMillis_;
        }

        inline double discretization() const {
            return discretization_;
        }
    };
    
    class Hello {
        std::uint64_t id_;
        
    public:
        explicit Hello(std::uint64_t id)
            : id_(id)
        {
        }

        explicit Hello(Type type, BufferView buf)
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

    class Done {
        std::uint64_t id_;
        
    public:
        explicit Done(std::uint64_t id)
            : id_(id)
        {
        }

        explicit Done(Type type, BufferView buf)
            : id_(buf.get<std::uint64_t>())
        {
        }

        std::uint64_t id() const {
            return id_;
        }
        
        operator Buffer () const {
            Size size = 16;
            Buffer buf{size};
            buf.put(DONE);
            buf.put(size);
            buf.put(id_);
            buf.flip();
            return buf;
        }
    };

    template <class S>
    class PathSE3 {
        using State = std::tuple<Eigen::Quaternion<S>, Eigen::Matrix<S, 3, 1>>;
        static constexpr std::size_t stateSize_ = buffer_size_v<State>;
        static constexpr Type TYPE = PATH_SE3 + sizeof(S)/8;
            
        std::vector<State> path_;

    public:
        explicit PathSE3(std::vector<State>&& path)
            : path_(std::move(path))
        {
        }

        inline PathSE3(Type, BufferView buf) {
            if (buf.remaining() % stateSize_ != 0)
                throw protocol_error("invalid path packet size: " + std::to_string(buf.remaining()));

            std::size_t n = buf.remaining() / stateSize_;
            path_.reserve(n);
            while (path_.size() < n)
                path_.emplace_back(buf.get<State>());
        }

        inline operator Buffer () const {
            Size size = buffer_size_v<Type> + buffer_size_v<Size>
                + stateSize_ * path_.size();
            Buffer buf{size};
            buf.put(TYPE);
            buf.put(size);
            for (const State& q : path_)
                buf.put(q);
            buf.flip();
            return buf;
        }

        const std::vector<State>& path() const & {
            return path_;
        }

        std::vector<State>&& path() && {
            return std::move(path_);
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
        size -= head;
        
        switch (type) {
        case HELLO:
            fn(Hello(type, buf.view(size)));
            break;
        case DONE:
            fn(Done(type, buf.view(size)));
            break;            
        case PROBLEM_SE3:
            fn(ProblemSE3<float>(type, buf.view(size)));
            break;
        case PROBLEM_SE3+1:
            fn(ProblemSE3<double>(type, buf.view(size)));
            break;
        case PATH_SE3:
            fn(PathSE3<float>(type, buf.view(size)));
            break;
        case PATH_SE3+1:
            fn(PathSE3<double>(type, buf.view(size)));
            break;
        default:
            throw protocol_error("bad packet type: " + std::to_string(type));
        }

        buf += size;
        
        return 0;
    }
}

#endif
