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
