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
        
        int socket_{-1};
        
        struct addrinfo *addrInfo_{nullptr};
        struct addrinfo *connectAddr_{nullptr};

        WriteQueue writeQueue_;

        std::uint64_t problemId_{0};
        
        void close();
        void connected();
        void tryConnect();
        
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
        void sendPath(std::vector<std::tuple<Eigen::Quaternion<S>, Eigen::Matrix<S, 3, 1>>>&& path);

        void done();
    };

}

template <class S>
void mpl::Comm::sendPath(std::vector<std::tuple<Eigen::Quaternion<S>, Eigen::Matrix<S, 3, 1>>>&& path) {
    writeQueue_.push_back(packet::PathSE3<S>(std::move(path)));
}


#endif
