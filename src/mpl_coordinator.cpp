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
#include <unistd.h>
#include <fcntl.h>

namespace mpl {
    template <class S>
    std::pair<int, int> launchLambda(std::uint64_t pId, packet::ProblemSE3<S>& prob);

    class Coordinator {
        using ID = std::uint64_t;
        
        int listen_{-1};

        class GroupData;
        class Connection;

        using Group = std::pair<const ID, GroupData>;
        
        ID nextGroupId_{1};

        std::list<Connection> connections_;
        std::list<std::pair<int, int>> childProcesses_;
        std::map<ID, GroupData> groups_;
        
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

        int accept(struct sockaddr *addr, socklen_t * addrLen) {
            return ::accept(listen_, addr, addrLen);
        }

        void loop();

        template <class S>
        void launchLambdas(ID groupId, packet::ProblemSE3<S>&& prob, int nLambdas);
        
        Group* createGroup(Connection* initiator);
        Group* addToGroup(ID id, Connection* conn);
        void done(Group* group, Connection* conn);
    };

    class Coordinator::GroupData {
        Connection* initiator_;
        std::list<Connection*> connections_;
        
    public:
        GroupData(Connection* initiator)
            : initiator_(initiator)
        {
        }

        Connection* initiator() {
            return initiator_;
        }

        auto& connections() {
            return connections_;
        }
    };

    class Coordinator::Connection {
        Coordinator& coordinator_;
        
        struct sockaddr_in addr_;
        socklen_t addrLen_{sizeof(addr_)};
        
        int socket_{-1};

        Buffer rBuf_{1024*4};
        WriteQueue writeQueue_;

        Group* group_{nullptr};

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
            group_ = coordinator_.addToGroup(pkt.id(), this);
            // this is a possible sign that the group already ended
            // before this connection arrived.  Respond with DONE.
            if (group_ == nullptr)
                writeQueue_.push_back(packet::Done(pkt.id()));
        }

        void process(packet::Done&& pkt) {
            JI_LOG(INFO) << "got DONE (id=" << pkt.id() << ")";
            if (group_ == nullptr || group_->first != pkt.id()) {
                JI_LOG(WARN) << "DONE group id mismatch";
            } else {
                // coordinator_.groupDone(group_, this);
                coordinator_.done(group_, this);
            }
        }

        template <class S>
        void process(packet::ProblemSE3<S>&& pkt) {
            JI_LOG(INFO) << "got ProblemSE3 " << sizeof(S);
            if (group_)
                coordinator_.done(group_, this);
            
            group_ = coordinator_.createGroup(this);
            int nLambdas = 4;
            coordinator_.launchLambdas(group_->first, std::move(pkt), nLambdas);
        }

        template <class S>
        void process(packet::PathSE3<S>&& pkt) {
            JI_LOG(INFO) << "got PathSE3 " << sizeof(S);
            for (auto& q : pkt.path())
                JI_LOG(TRACE) << "  " << q;
        }
        
    public:
        explicit Connection(Coordinator& coordinator)
            : coordinator_(coordinator)
            , socket_(coordinator.accept(reinterpret_cast<struct sockaddr*>(&addr_), &addrLen_))
        {
            JI_LOG(TRACE) << "connection accepted";
        }
        
        ~Connection() {
            if (group_)
                coordinator_.done(group_, this);
            
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

        template <class Packet>
        void write(Packet&& packet) {
            writeQueue_.push_back(std::forward<Packet>(packet));
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

    template <class ... T>
    std::string to_string(T&& ... args) {
        std::ostringstream str;
        (str << ... << std::forward<T>(args));
        return str.str();
    }
}

// returns a pair of process-id and pipe-fd associated with the
// child process
template <class S>
std::pair<int, int> mpl::launchLambda(std::uint64_t pId, packet::ProblemSE3<S>& prob) {
    static const std::string resourceDirectory = "../../resources/";
    static int lambdaId;
    ++lambdaId;

    // We create a pipe solely for tracking when a child process
    // terminates.  When the child terminates, it will
    // automatically close its end of the pipe, causing a POLLHUP
    // event in the poll() loop.
    int p[2];
    if (::pipe(p) == -1)
        throw std::system_error(errno, std::system_category(), "pipe");
    
    if (int pid = ::fork()) {
        // parent process
        ::close(p[1]);
        return { pid, p[0] };
    }
    
    // child process
    ::close(p[0]);
    
    // child process
    Eigen::IOFormat fmt(Eigen::FullPrecision, Eigen::DontAlignCols, ",", ",", "", "", "", "");
    
    std::string path = "./mpl_lambda";
    
    std::string groupId = std::to_string(pId);
    std::string env = resourceDirectory + prob.envMesh();
    std::string robot = resourceDirectory + prob.robotMesh();
    
    std::string discretization = std::to_string(prob.discretization());
    
    std::string alg;
    if (prob.algorithm() == packet::ALGORITHM_RRT)
        alg = "rrt";
    else if (prob.algorithm() == packet::ALGORITHM_CFOREST)
        alg = "cforest";
    else
        alg = "unknown";
    
    std::string timeLimit = std::to_string(prob.timeLimitMillis() / 1e3);
    
    std::string start = to_string(
        std::get<0>(prob.start()).coeffs().format(fmt), ",",
        std::get<1>(prob.start()).format(fmt));
    std::string goal = to_string(
        std::get<0>(prob.goal()).coeffs().format(fmt), ",",
        std::get<1>(prob.goal()).format(fmt));
    
    std::string min = to_string(prob.min().format(fmt));
    std::string max = to_string(prob.max().format(fmt));
    
    const char * const argv[] = {
        path.c_str(),
        "--coordinator=localhost",
        "--algorithm", alg.c_str(),
        "-I", groupId.c_str(),
        "-t", timeLimit.c_str(),
        "-d", discretization.c_str(),
        "--env", env.c_str(),
        "--robot", robot.c_str(),
        "--start", start.c_str(),
        "--goal", goal.c_str(),
        "--min", min.c_str(),
        "--max", max.c_str(),
        nullptr
    };
    
    char file[20];
    snprintf(file, sizeof(file), "lambda-%04d.out", lambdaId);
    
    // JI_LOG(TRACE) << "RUNNING Lambda: " <<
    //     "./mpl_lambda"
    std::ostringstream args;
    for (int i=0 ; argv[i] ; ++i)
        args << ' ' << argv[i];
    JI_LOG(TRACE) << "Running lambda:" << args.str();
    
    int fd = ::open(file, O_RDWR | O_CREAT, S_IRUSR | S_IWUSR);
    dup2(fd, 1); // make stdout write to file
    dup2(fd, 2); // make stderr write to file
    close(fd); // close fd, dups remain open
    
    execv(path.c_str(), const_cast<char*const*>(argv));
    
    // if exec returns, then there was a problem
    throw std::system_error(errno, std::system_category(), "exec");
}

auto mpl::Coordinator::createGroup(Connection* initiator) -> Group* {
    auto [ it, inserted ] = groups_.emplace(nextGroupId_++, initiator);
    assert(inserted);
    JI_LOG(INFO) << "starting new group " << it->first;
    return &*it;
}

auto mpl::Coordinator::addToGroup(ID id, Connection* conn) -> Group* {
    auto it = groups_.find(id);
    if (it == groups_.end())
        return nullptr;
    
    it->second.connections().push_back(conn);
    return &*it;
}

void mpl::Coordinator::done(Group* group, Connection* conn) {
    auto& connections = group->second.connections();
    for (auto it = connections.begin() ; it != connections.end() ; ) {
        if (*it == conn) {
            it = connections.erase(it);
        } else {
            (*it)->write(packet::Done(group->first));
            ++it;
        }
    }
    
    if (group->second.initiator() == conn) {
        JI_LOG(INFO) << "removing group " << group->first;
        auto it = groups_.find(group->first);
        if (it != groups_.end())
            groups_.erase(it);
    }
}

template <class S>
void mpl::Coordinator::launchLambdas(ID groupId, packet::ProblemSE3<S>&& prob, int nLambdas) {
    for (int i=0 ; i<nLambdas ; ++i)
        childProcesses_.emplace_back(launchLambda(groupId, prob));
}

void mpl::Coordinator::loop() {
    std::vector<struct pollfd> pfds;

    for (;;) {
        pfds.clear();

        // first comes the fds of the child processes (note that
        // connection processing may change the child process list, so
        // this must be processed first)
        for (auto [ pid, fd ] : childProcesses_) {
            pfds.emplace_back();
            pfds.back().fd = fd;
            pfds.back().events = POLLIN;
        }

        // then et of pollfds is 1:1 with connections
        for (Connection& conn : connections_)
            pfds.emplace_back(conn);

        // then comes the accepting socket
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
        for (auto cit = childProcesses_.begin() ; cit != childProcesses_.end() ; ++pit) {
            assert(pit != pfds.end());
            
            if ((pit->revents & POLLHUP) == 0) {
                ++cit;
            } else {
                int stat = 0;
                if (::waitpid(cit->first, &stat, 0) == -1)
                    JI_LOG(WARN) << "waitpid failed with error: " << errno;
                if (::close(cit->second) == -1)
                    JI_LOG(WARN) << "close failed with error: " << errno;
                JI_LOG(INFO) << "child process " << cit->first << " exited with status " << stat;
                
                cit = childProcesses_.erase(cit);
            }
        }

        for (auto cit = connections_.begin() ; cit != connections_.end() ; ++pit) {
            assert(pit != pfds.end());
            
            if (cit->process(*pit))
                ++cit;
            else
                cit = connections_.erase(cit);
        }
        
        assert(pit+1 == pfds.end());
        
        if (pit->revents & (POLLERR | POLLHUP))
            break;
        if (pit->revents & POLLIN) {
            connections_.emplace_back(*this);
            if (!connections_.back()) {
                JI_LOG(WARN) << "accept failed with error: " << errno;
                connections_.pop_back();
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
