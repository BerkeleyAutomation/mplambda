#include <mpl/demo/app_options.hpp>
#include <mpl/demo/se3_rigid_body_scenario.hpp>
#include <mpl/prrt.hpp>
#include <mpl/comm.hpp>
#include <mpl/pcforest.hpp>
#include <mpl/option.hpp>
#include <getopt.h>
#include <optional>

namespace mpl::demo {

    template <class T, class U>
    struct ConvertPath;

    template <class R, class S>
    struct ConvertPath<
        std::tuple<Eigen::Quaternion<R>, Eigen::Matrix<R, 3, 1>>,
        std::tuple<Eigen::Quaternion<S>, Eigen::Matrix<S, 3, 1>>>
    {
        using Result = std::tuple<Eigen::Quaternion<R>, Eigen::Matrix<R, 3, 1>>;
        using Source = std::tuple<Eigen::Quaternion<S>, Eigen::Matrix<S, 3, 1>>;
        
        static std::vector<Result> apply(std::vector<Source>&& arg) {
            std::vector<Result> r;
            r.reserve(arg.size());
            for (auto& q : arg)
                r.emplace_back(
                    std::get<0>(q).template cast<R>(),
                    std::get<1>(q).template cast<R>());
            return r;
        }
    };
    
    template <class T, class U>
    std::vector<T> convertPath(std::vector<U>&& path) {
        if constexpr (std::is_same_v<T, U>)
            return path;
        else
            return ConvertPath<T, U>::apply(std::move(path));
    }
    
    template <class Scenario>
    class App {
        using State = typename Scenario::State;
        using Distance = typename Scenario::Distance;
        using Bound = Eigen::Matrix<Distance, 3, 1>;

        demo::AppOptions options_;
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

        template <class T>
        void sendPath(T& solution) {
            std::vector<State> path;
            solution.visit([&] (const State& q) { path.push_back(q); });
            std::reverse(path.begin(), path.end());
            Distance cost = solution.cost();
            comm_.sendPath(cost, std::move(path));
        }

        template <class Algorithm>
        void runImpl() {
            State qStart = options_.start<State>();
            State qGoal = options_.goal<State>();
            Bound min = options_.min<Bound>();
            Bound max = options_.max<Bound>();
            
            JI_LOG(INFO) << "start: " << qStart;
            JI_LOG(INFO) << "goal: " << qGoal;
            JI_LOG(INFO) << "bounds: " << min << " to " << max;

            double discretization = options_.discretization();
            if (discretization <= 0)
                discretization = 0.1;
            
            Planner<Scenario, Algorithm> planner(
                options_.env(),
                options_.robot(),
                qGoal, min, max, discretization);

            planner.addStart(qStart);

            using Clock = std::chrono::steady_clock;
            Clock::duration maxElapsedSolveTime = std::chrono::duration_cast<Clock::duration>(
                std::chrono::duration<double>(options_.timeLimit()));
            auto start = Clock::now();

            auto solution = planner.solution();

            if constexpr (Algorithm::asymptotically_optimal) {
                // asymptotically-optimal planner, run for the
                // time-limit, and update the graph with best paths
                // from the network.
                planner.solve([&] {
                    if (maxElapsedSolveTime.count() > 0 && Clock::now() - start > maxElapsedSolveTime)
                        return true;
                    comm_.process(
                        [&] (auto cost, auto&& path) {
                            planner.addPath(cost, convertPath<State>(std::forward<decltype(path)>(path)));

                            // update our best solution if it has the
                            // same cost as the solution we just got
                            // from a peer.  If we have a different
                            // solution, then we'll update and send
                            // the solution after the comm_.process().
                            // This avoids re-broadcasting the same
                            // solution.
                            auto newSol = planner.solution();
                            if (newSol.cost() == cost)
                                solution = newSol;
                        });

                    auto s = planner.solution();
                    if (s < solution) {
                        sendPath(s);
                        solution = s;
                    }
                    
                    return comm_.isDone();
                });
            } else {
                // non-asymptotically-optimal.  Stop as soon as we
                // have a solution (either locally or from the
                // network)
                planner.solve([&] {
                    if (maxElapsedSolveTime.count() > 0 && Clock::now() - start > maxElapsedSolveTime)
                        return true;
                    comm_.process();
                    return comm_.isDone() || planner.isSolved();
                });
            }
            

            JI_LOG(INFO) << "solution " << (planner.isSolved() ? "" : "not ") << "found after " << (Clock::now() - start);
            JI_LOG(INFO) << "graph size = " << planner.size();

            
            auto finalSolution = planner.solution();
            if (finalSolution != solution) {
                finalSolution.visit([] (const State& q) { JI_LOG(INFO) << "  " << q; });
                sendPath(finalSolution);
            }
            
            comm_.sendDone();
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
