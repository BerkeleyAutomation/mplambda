#include <jilog.hpp>
#include <mpl/buffer.hpp>
#include <mpl/write_queue.hpp>
#include <mpl/packet.hpp>
#include <mpl/demo/se3_app_options.hpp>
#include <mpl/demo/se3_rigid_body_scenario.hpp>
#include <netdb.h>
#include <getopt.h>
#include <iostream>
#include <poll.h>

namespace mpl {
    class RobotClient {
        int socket_{-1};

        Buffer rBuf_{1024*4};
        WriteQueue writeQueue_;

    public:
        RobotClient() {
        }

        ~RobotClient() {
            if (socket_ != -1)
                ::close(socket_);
        }

        void connect(const std::string& host, int port) {
            struct addrinfo hints, *addrInfo;
            std::memset(&hints, 0, sizeof(hints));
            hints.ai_family = PF_UNSPEC;
            hints.ai_socktype = SOCK_STREAM;
            hints.ai_flags = AI_PASSIVE;

            std::string service = std::to_string(port);

            if (int err = ::getaddrinfo(host.c_str(), service.c_str(), &hints, &addrInfo))
                throw std::invalid_argument("getaddrinfo failed: " + std::to_string(err));

            for (auto it = addrInfo ; it ; it = it->ai_next) {
                if ((socket_ = ::socket(it->ai_family, it->ai_socktype, it->ai_protocol)) == -1) {
                    JI_LOG(INFO) << "failed to create socket: " << errno;
                } else if (::connect(socket_, it->ai_addr, it->ai_addrlen) == 0) {
                    JI_LOG(INFO) << "connected";
                    break;
                } else {
                    ::close(std::exchange(socket_, -1));
                }
            }

            if (socket_ == -1)
                JI_LOG(WARN) << "connect failed: " << errno;
        }

        void connect(const std::string& host) {
            auto i = host.find(':');
            if (i == std::string::npos) {
                connect(host, 0x415E);
            } else {
                connect(host.substr(0, i), std::stoi(host.substr(i+1)));
            }
        }

    private:
        void doRead() {
            assert(rBuf_.remaining() > 0); // we may need to grow the buffer

            ssize_t n = ::recv(socket_, rBuf_.begin(), rBuf_.remaining(), 0);
            if (n < 0)
                throw std::system_error(errno, std::system_category(), "recv");
            if (n == 0) {
                ::close(std::exchange(socket_, -1));
                return;
            }

            rBuf_ += n;
            rBuf_.flip();
            std::size_t needed;
            while ((needed = packet::parse(rBuf_, [&] (auto&& pkt) {
                            process(std::forward<decltype(pkt)>(pkt));
                        })) == 0);
            rBuf_.compact(needed);
        }

        template <class T>
        void process(T&&) {
            JI_LOG(WARN) << "unexpected packet type";
        }

    public:
        template <class S>
        void sendProblemSE3(
            const std::string& env,
            const std::string& robot,
            const std::tuple<Eigen::Quaternion<S>, Eigen::Matrix<S, 3, 1>>& start,
            const std::tuple<Eigen::Quaternion<S>, Eigen::Matrix<S, 3, 1>>& goal,
            const Eigen::Matrix<S, 3, 1>& min,
            const Eigen::Matrix<S, 3, 1>& max,
            const std::string& algorithm,
            double timeLimit,
            double discretization)
        {
            std::uint32_t alg;
                
            if ("rrt" == algorithm)
                alg = packet::ALGORITHM_RRT;
            else if ("cforest" == algorithm)
                alg = packet::ALGORITHM_CFOREST;
            else
                throw std::invalid_argument("unknown algorithm: " + algorithm);

            std::uint32_t timeLimitMillis = static_cast<std::uint32_t>(timeLimit * 1e3);
            
            writeQueue_.push_back(packet::ProblemSE3<S>(env, robot, start, goal, min, max, alg, timeLimitMillis, discretization));
        }
        
        void loop() {
            while (socket_ != -1) {
                struct pollfd pfd;
                pfd.fd = socket_;
                pfd.events = POLLIN;
                if (!writeQueue_.empty())
                    pfd.events |= POLLOUT;

                if (::poll(&pfd, 1, -1) == -1)
                    throw std::system_error(errno, std::system_category(), "poll()");

                if (pfd.revents & POLLIN)
                    doRead();
                
                if (pfd.revents & POLLOUT)
                    writeQueue_.writeTo(socket_);
            }
        }
    };
}

static void usage(const char *argv0) {
    std::clog << "Usage: " << argv0 << " [options]\n"
        "Options:\n"
        "  -c, --coordinator=HOST[:PORT]" << std::endl;
}

int main(int argc, char *argv[]) try {
    using S = double;
    using Scenario = mpl::demo::SE3RigidBodyScenario<S>;
    mpl::demo::SE3AppOptions<Scenario> options(argc, argv);

    if (options.coordinator().empty())
        throw std::invalid_argument("--coordinator is required");
    
    mpl::RobotClient robot;
    
    robot.connect(options.coordinator());
    robot.sendProblemSE3(
        options.env(),
        options.robot(),
        options.start(),
        options.goal(),
        options.min(),
        options.max(),
        options.algorithm(),
        options.timeLimit(),
        options.discretization());
    robot.loop();

    return EXIT_SUCCESS;
} catch (const std::exception& ex) {
    JI_LOG(FATAL) << "error: " << ex.what();
    return EXIT_FAILURE;
}
