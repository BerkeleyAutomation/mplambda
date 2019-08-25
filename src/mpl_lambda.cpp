#include <mpl/demo/se3_rigid_body_scenario.hpp>
#include <mpl/prrt.hpp>
#include <mpl/comm.hpp>
#include <mpl/pcforest.hpp>
#include <mpl/option.hpp>
#include <getopt.h>
#include <optional>

namespace mpl::demo {


    template <class T>
    T decode(const char* buf) {
        T value;
        if (__BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__) {
            std::reverse_copy(buf, buf + sizeof(T), reinterpret_cast<char *>(&value));
        } else if (__BYTE_ORDER__ == __ORDER_BIG_ENDIAN__) {
            std::copy(buf, buf + sizeof(T), reinterpret_cast<char *>(&value));
        } else{
            abort();
        }
        return value;
    }

    template <class T>
    void encode(char *buf, const T& value) {
        if (__BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__) {
            std::reverse_copy(
                reinterpret_cast<char*>(&value),
                reinterpret_cast<char*>(&value) + sizeof(value),
                buf);
        } else if (__BYTE_ORDER__ == __ORDER_BIG_ENDIAN__) {
            std::copy(
                reinterpret_cast<char*>(&value),
                reinterpret_cast<char*>(&value) + sizeof(value),
                buf);
        } else {
            abort();
        }
                              
    }
/*
    class Comm {
        static constexpr int DEFAULT_PORT = 0x415E;
        using byte = char;
        
        enum {
            DISCONNECTED,
            CONNECTING,
            CONNECTED
        };

        int state_{DISCONNECTED};
        int socket_{-1};
        
        struct addrinfo* addr_{nullptr};

        Buffer rBuf_{1024*4};
        WriteQueue writeQueue_;

        void processData() {
            JI_LOG(DEBUG) << "processing " << rLen_ << " bytes";
            rLen_ = 0;
        }

        void drainWriteQueue() {
            struct pollfd pfd;

            writeQueue_.sendTo(socket_);
            
            // other option might be to just make the socket blocking
            // again...
            while (!writeQueue_.empty()) {
                pfd.fd = socket_;
                pfd.events = POLLOUT;
                if (::poll(&pfd, 1, -1) == -1) {
                    JI_LOG(WARN) << "poll failed: " << errno;
                    close();
                    return;
                }
                if (pfd.revents & POLLOUT)
                    writeQueue_.sendTo(socket_);
            }
        }

    public:
        ~Comm() {
            if (socket_ != -1)
                ::close(socket_);
            
            if (addr_)
                ::freeaddrinfo(addr_);
        }

        void close() {
            state_ = DISCONNECTED;
            if (::close(std::exchange(socket_, -1)) == -1)
                JI_LOG(WARN) << "close failed (" << errno << ")";
        }
        
        void connect(const std::string& host, int port = DEFAULT_PORT) {
            JI_LOG(INFO) << "connecting to [" << host << "], port " << port;

            if (socket_ != -1)
                close();

            sys::AddrInfo addr(host, port);
            for (auto it = addr.begin() ; it != addr.end() ; ++it) {
                if ((socket_ = ::socket(it->ai_family, it->ai_socktype, it->ai_protocol)) == -1) {
                    JI_LOG(INFO) << "failed to create socket (" << errno << ")";
                    continue;
                }

                int nonBlocking = 1;
                if (::ioctl(socket_, FIONBIO, reinterpret_cast<char*>(&nonBlocking)) == -1)
                    JI_LOG(INFO) << "set non-blocking failed (" << errno << ")";

                if (::connect(socket_, it->ai_addr, it->ai_addrlen) == 0) {
                    state_ = CONNECTED;
                    JI_LOG(INFO) << "connected";
                    return;
                }

                if (errno == EINPROGRESS) {
                    state_ = CONNECTING;
                    JI_LOG(INFO) << "non-blocking connection in progress";
                    return;
                }
                
                JI_LOG(INFO) << "connect failed";
                close();
            }

            assert(socket_ == -1);
            JI_LOG(INFO) << "exhausted address options for establishing connection";
        }

        void process() {
            struct pollfd pfd;
            
            switch (state_) {
            case DISCONNECTED:
                return;
            case CONNECTING:
                pfd.fd = socket_;
                pfd.events = POLLOUT;
                if (::poll(&pfd, 1, 0) == -1) {
                    JI_LOG(WARN) << "poll failed while waiting for connection (" << errno << ")";
                    close();
                    return;
                }

                if (pfd.revents & POLLERR) {
                    int err;
                    socklen_t len = sizeof(err);
                    if (::getsockopt(socket_, SOL_SOCKET, SO_ERROR, &err, len) == -1)
                        JI_LOG(WARN) << "getsockopt failed";
                    else
                        JI_LOG(WARN) << "connection failed with error (" << err << ")";
                    close();
                    return;
                }
                
                if (pfd.revents & POLLOUT) {
                    // potentially connected
                    state_ = CONNECTED;
                    JI_LOG(INFO) << "connected";

                    // if (::getpeername(comm_, &addr, &len) == -1)
                    //      check if errno == ENOTCONN
                }
                break;
            case CONNECTED:
                // we could poll for POLLIN/POLLOUT, but the
                // connection is non-blocking so we change just
                // attempt to read or write.
                if (!writeQueue_.sendTo(socket_)) {
                    JI_LOG(WARN) << "write queue failed";
                    close();
                    return;
                }                    
                
                ssize_t n = ::recv(rBuf_.start(), rBuf_.remaining(), 0);
                if (n == -1 && errno != EAGAIN) {
                    JI_LOG(WARN) << "recv error: " << errno;
                    close();
                    return;
                }
                if (n) {
                    rBuf_ += n;
                    processData();
                }
                break;
            }
        }

        void done() {
            switch (state_) {
            case CONNECTED:
                sendDonePacket();
                drainWriteQueue();
                // fall through
            case CONNECTING:
                close();
            }
        }
    };
*/
    
    template <class Scenario>
    class App {            
        using State = typename Scenario::State;
        using Distance = typename Scenario::Distance;
        using Bound = Eigen::Matrix<Distance, 3, 1>;

        std::string algorithm_;
        std::string coordinator_;
        
        std::string envMesh_;
        std::string robotMesh_;

        std::optional<State> qStart_;
        std::optional<State> qGoal_;

        std::optional<Bound> qMin_;
        std::optional<Bound> qMax_;

        Comm comm_;

        static void usage() {
            std::cerr << R"(Usage: [options]
Options:
  --coordinator=HOST:PORT
  --env=MESH
  --robot=MESH
  --start=W,I,J,K,X,Y,Z
  --goal=W,I,J,K,X,Y,Z
  --min=X,Y,Z
  --max=X,Y,Z
)";
        }

    public:
        App(int argc, char *argv[]) {
            static struct option longopts[] = {
                { "algorithm", required_argument, NULL, 'a' },
                { "env", required_argument, NULL, 'e' },
                { "robot", required_argument, NULL, 'r' },
                { "goal", required_argument, NULL, 'g' },
                { "start", required_argument, NULL, 's' },
                { "min", required_argument, NULL, 'm' },
                { "max", required_argument, NULL, 'M' },
                { "coordinator", required_argument, NULL, 'c' },
                
                { NULL, 0, NULL, 0 }
            };

            for (int ch ; (ch = getopt_long(argc, argv, "a:e:r:g:s:m:M:c:", longopts, NULL)) != -1 ; ) {
                switch (ch) {
                case 'a':
                    algorithm_ = optarg;
                    break;
                case 'e':
                    envMesh_ = optarg;
                    break;
                case 'r':
                    robotMesh_ = optarg;
                    break;
                case 'g':
                    parse("goal", qGoal_, optarg);
                    break;
                case 's':
                    parse("start", qStart_, optarg);
                    break;
                case 'm':
                    parse("min", qMin_, optarg);
                    break;
                case 'M':
                    parse("max", qMax_, optarg);
                    break;
                case 'c':
                    coordinator_ = optarg;
                    break;
                default:
                    usage();
                    throw std::invalid_argument("unknown option");
                }            
            }

            if (algorithm_.empty())
                throw std::invalid_argument("--algorithm is required");
            if (robotMesh_.empty())
                throw std::invalid_argument("--robot is required");
            if (envMesh_.empty())
                throw std::invalid_argument("--env is required");
            if (!qGoal_)
                throw std::invalid_argument("--goal is required");
            if (!qStart_)
                throw std::invalid_argument("--start is required");
            if (!qMin_)
                throw std::invalid_argument("--min is required");
            if (!qMax_)
                throw std::invalid_argument("--max is required");
        }


        void connect() {
            if (coordinator_.empty())
                return;

            comm_.setProblemId(12345678);

            auto i = coordinator_.find(':');
            if (i == std::string::npos)
                comm_.connect(coordinator_);
            else
                comm_.connect(coordinator_.substr(0, i), std::stoi(coordinator_.substr(i+1)));
        }

        template <class Algorithm>
        void runImpl() {
            JI_LOG(INFO) << "start: " << qStart_;
            JI_LOG(INFO) << "goal: " << qGoal_;
            JI_LOG(INFO) << "bounds: " << *qMin_ << " to " << *qMax_;
    
            Planner<Scenario, Algorithm> planner{envMesh_, robotMesh_, *qGoal_, *qMin_, *qMax_, 0.1};

            planner.addStart(*qStart_);

            using Clock = std::chrono::steady_clock;
            auto start = Clock::now();
            planner.solve([&] {
                comm_.process();
                return planner.isSolved();
            });
            //comm_.done();

            JI_LOG(INFO) << "solution found after " << (Clock::now() - start);
            JI_LOG(INFO) << "graph size = " << planner.size();
            planner.solution([] (const State& q) {
                    JI_LOG(INFO) << "  " << q;
                });
        }

        void run() {
            connect();
            
            if ("rrt" == algorithm_)
                runImpl<mpl::PRRT>();
            else if ("cforest" == algorithm_)
                runImpl<mpl::PCForest>();
            else
                throw std::invalid_argument("unknown algorithm: " + algorithm_);
        }
    };
}

int main(int argc, char *argv[]) try {
    using S = double;
    using Scenario = mpl::demo::SE3RigidBodyScenario<S>;

    mpl::demo::App<Scenario> app(argc, argv);

    app.run();

    return EXIT_SUCCESS;
} catch (const std::invalid_argument& ex) {
    std::cerr << "Invalid argument: " << ex.what() << std::endl;
    return EXIT_FAILURE;
} catch (const std::exception& ex) {
    std::cerr << "Error: " << ex.what() << std::endl;
    return EXIT_FAILURE;
}
