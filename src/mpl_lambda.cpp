#include <mpl/demo/se3_app_options.hpp>
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

        demo::SE3AppOptions<Scenario> options_;
        Comm comm_;

    public:
        App(int argc, char *argv[])
            : options_(argc, argv)
        {
        }

        void connect() {
            const auto& coordinator = options_.coordinator();
            
            if (coordinator.empty())
                return;

            comm_.setProblemId(options_.problemId());

            auto i = coordinator.find(':');
            if (i == std::string::npos)
                comm_.connect(coordinator);
            else
                comm_.connect(coordinator.substr(0, i), std::stoi(coordinator.substr(i+1)));
        }

        template <class Algorithm>
        void runImpl() {
            JI_LOG(INFO) << "start: " << options_.start();
            JI_LOG(INFO) << "goal: " << options_.goal();
            JI_LOG(INFO) << "bounds: " << options_.min() << " to " << options_.max();

            double discretization = options_.discretization();
            if (discretization <= 0)
                discretization = 0.1;
            
            Planner<Scenario, Algorithm> planner(
                options_.env(),
                options_.robot(),
                options_.goal(),
                options_.min(),
                options_.max(),
                discretization);
            // envMesh_, robotMesh_, *qGoal_, *qMin_, *qMax_, 0.1};

            planner.addStart(options_.start());

            using Clock = std::chrono::steady_clock;
            Clock::duration maxElapsedSolveTime = std::chrono::duration_cast<Clock::duration>(
                std::chrono::duration<double>(options_.timeLimit()));
            auto start = Clock::now();
            planner.solve([&] {
                if (maxElapsedSolveTime.count() > 0 && Clock::now() - start > maxElapsedSolveTime)
                    return true;
                comm_.process();
                return planner.isSolved();
            });
            

            JI_LOG(INFO) << "solution " << (planner.isSolved() ? "" : "not ") << "found after " << (Clock::now() - start);
            JI_LOG(INFO) << "graph size = " << planner.size();
            planner.solution([] (const State& q) { JI_LOG(INFO) << "  " << q; });

            if (planner.isSolved()) {
                std::vector<State> path;
                planner.solution([&] (const State& q) { path.push_back(q); });
                comm_.sendPath(std::move(path));
            }
            
            comm_.done();
        }

        void run() {
            connect();
            
            if ("rrt" == options_.algorithm())
                runImpl<mpl::PRRT>();
            else if ("cforest" == options_.algorithm())
                runImpl<mpl::PCForest>();
            else
                throw std::invalid_argument("unknown algorithm: " + options_.algorithm());
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
