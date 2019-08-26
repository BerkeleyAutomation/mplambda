#pragma once
#ifndef MPL_COMM_HPP
#define MPL_COMM_HPP

#include <string>
#include <netdb.h>
#include "write_queue.hpp"
#include "packet.hpp"

namespace mpl {
    class Comm {
        static constexpr int DEFAULT_PORT = 0x415E;

        enum {
            DISCONNECTED,
            CONNECTING,
            CONNECTED,
        };

        int state_{DISCONNECTED};
        int done_{false};
        int socket_{-1};
        
        struct addrinfo *addrInfo_{nullptr};
        struct addrinfo *connectAddr_{nullptr};

        Buffer rBuf_{4*1024};
        WriteQueue writeQueue_;

        std::uint64_t problemId_{0};
        
        void close();
        void connected();
        void tryConnect();

        template <class T>
        void process(T&&) {
            JI_LOG(WARN) << "unexpected packet type received: " << T::name();
        }

        void process(packet::Done&&);
        
    public:
        ~Comm();

        void setProblemId(std::uint64_t id) {
            problemId_ = id;
        }

        inline operator bool () const {
            return socket_ != -1;
        }
        
        void connect(const std::string& host, int port = DEFAULT_PORT);
        void process();

        template <class S>
        void sendPath(S cost, std::vector<std::tuple<Eigen::Quaternion<S>, Eigen::Matrix<S, 3, 1>>>&& path);
        void sendDone();

        inline bool isDone() {
            return done_;
        }
    };

}

template <class S>
void mpl::Comm::sendPath(
    S cost,
    std::vector<std::tuple<Eigen::Quaternion<S>, Eigen::Matrix<S, 3, 1>>>&& path)
{
    writeQueue_.push_back(packet::PathSE3<S>(cost, std::move(path)));
}


#endif
