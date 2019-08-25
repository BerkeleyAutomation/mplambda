#pragma once
#ifndef MPL_COMM_HPP
#define MPL_COMM_HPP

#include <string>
#include <netdb.h>
#include "write_queue.hpp"

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
    };
}

#endif
