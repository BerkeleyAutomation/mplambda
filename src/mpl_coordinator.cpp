#include <jilog.hpp>
#include <mpl/buffer.hpp>
#include <mpl/write_queue.hpp>
#include <mpl/packet.hpp>
#include <netinet/ip.h>
#include <netinet/tcp.h>
#include <poll.h>
#include <vector>
#include <list>
#include <map>
#include <sys/socket.h>
#include <system_error>


namespace mpl {
    class Coordinator {
        int listen_{-1};

        class Problem;
        class Connection;

        std::map<std::uint64_t, Problem> problems_;
        
    public:
        Coordinator(int port = 0x415E)
            : listen_(::socket(PF_INET, SOCK_STREAM, 0))
        {
            if (listen_ == -1)
                throw std::system_error(errno, std::system_category(), "socket()");

            int on = 1;
            if (::setsockopt(listen_, SOL_SOCKET, SO_REUSEADDR, &on, sizeof(on)) == -1)
                throw std::system_error(errno, std::system_category(), "set reuse addr");
            
            struct sockaddr_in addr;
            addr.sin_family = AF_INET;
            addr.sin_addr.s_addr = htonl(INADDR_ANY);
            addr.sin_port = htons(port);
            
            if (::bind(listen_, reinterpret_cast<struct sockaddr*>(&addr), sizeof(addr)) == -1)
                throw std::system_error(errno, std::system_category(), "bind()");

            socklen_t addrLen = sizeof(addr);
            if (::getsockname(listen_, reinterpret_cast<struct sockaddr*>(&addr), &addrLen) == -1)
                throw std::system_error(errno, std::system_category(), "getsockname()");

            JI_LOG(INFO) << "listening on port: " << ntohs(addr.sin_port);
            
            if (::listen(listen_, 5) == -1)
                throw std::system_error(errno, std::system_category(), "listen()");
        }
        
        ~Coordinator() {
            if (listen_ != -1 && !::close(listen_))
                JI_LOG(WARN) << "failed to close listening socket";                
        }

        void loop();
    };

    class Coordinator::Problem {
    };

    class Coordinator::Connection {
        struct sockaddr_in addr_;
        socklen_t addrLen_{sizeof(addr_)};
        
        int socket_{-1};

        Buffer rBuf_{1024*4};
        WriteQueue writeQueue_;

        bool doRead() {
            assert(rBuf_.remaining() > 0); // we may need to grow the buffer
            
            ssize_t n = ::recv(socket_, rBuf_.begin(), rBuf_.remaining(), 0);
            JI_LOG(TRACE) << "recv " << n;
            if (n < 0)
                throw std::system_error(errno, std::system_category(), "recv");

            if (n == 0)
                return false;

            rBuf_ += n;
            rBuf_.flip();
            // call the appropriate process overload for each packet
            // that arrives
            std::size_t needed;
            while ((needed = packet::parse(rBuf_, [&] (auto&& pkt) {
                            process(std::forward<decltype(pkt)>(pkt));
                        })) == 0);
            rBuf_.compact(needed);
            return true;
        }

        void process(packet::Hello&& pkt) {
            JI_LOG(INFO) << "got HELLO (id=" << pkt.id() << ")";
        }

        template <class S>
        void process(packet::ProblemSE3<S>&& pkt) {
            JI_LOG(INFO) << "got ProblemSE3 " << sizeof(S);
        }
        
    public:
        explicit Connection(int listen)
            : socket_(::accept(listen, reinterpret_cast<struct sockaddr*>(&addr_), &addrLen_))
        {
            JI_LOG(TRACE) << "connection accepted";
        }
        
        ~Connection() {
            JI_LOG(TRACE) << "closing connection";
            if (socket_ != -1 && ::close(socket_) == -1)
                JI_LOG(WARN) << "connection close error: " << errno;
        }
        
        operator bool () const {
            return socket_ != -1;
        }

        operator struct pollfd () const {
            return { socket_, static_cast<short>(writeQueue_.empty() ? POLLIN : (POLLIN | POLLOUT)), 0 };
        }

        bool process(const struct pollfd& pfd) {
            try {
                if ((pfd.revents & POLLIN) && !doRead())
                    return false;

                if (pfd.revents & POLLOUT)
                    writeQueue_.writeTo(socket_);

                return true;
            } catch (const std::exception& ex) {
                JI_LOG(WARN) << "exception processing connection: " << ex.what();
                return false;
            }
        }
    };
}

void mpl::Coordinator::loop() {
    std::list<Connection> connections;
    std::vector<struct pollfd> pfds;

    for (;;) {
        pfds.clear();
        for (Connection& conn : connections)
            pfds.emplace_back(conn);
        pfds.emplace_back();
        pfds.back().fd = listen_;
        pfds.back().events = POLLIN;

        JI_LOG(TRACE) << "polling " << pfds.size();
        int nReady = ::poll(pfds.data(), pfds.size(), -1);

        JI_LOG(TRACE) << "poll returned " << nReady;
        
        if (nReady == -1) {
            if (errno == EAGAIN || errno == EINTR)
                continue;
            throw std::system_error(errno, std::system_category(), "poll");
        }
        
        auto pit = pfds.begin();
        for (auto cit = connections.begin() ; cit != connections.end() ; ++pit) {
            if (cit->process(*pit))
                ++cit;
            else
                cit = connections.erase(cit);
        }
        
        assert(pit+1 == pfds.end());
        
        if (pit->revents & (POLLERR | POLLHUP))
            break;
        if (pit->revents & POLLIN) {
            connections.emplace_back(listen_);
            if (!connections.back()) {
                JI_LOG(WARN) << "accept failed with error: " << errno;
                connections.pop_back();
            }
            // struct sockaddr_in addr;
            // socklen_t addLen = sizeof(addr);
            
            // int fd = ::accept(listen, &addr, &addrLen);
            // if (fd != -1)
            //     connections.emplace_back(socket_);
            // && !connections.emplace_back(listen_))
            // connections.pop_back();
        }
    }
}


int main(int argc, char *argv[]) {
    mpl::Coordinator coordinator;
    
    coordinator.loop();
}
