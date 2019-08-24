#include <mpl/demo/se3_rigid_body_scenario.hpp>
#include <mpl/prrt.hpp>
#include <mpl/pcforest.hpp>
#include <getopt.h>
#include <optional>

namespace mpl::demo {

    template <class T>
    struct Option;

    template <>
    struct Option<float> {
        static float parse(const std::string& name, const char *arg, char **endp) {
            float v = std::strtof(arg, endp);
            if (*endp == arg)
                throw std::invalid_argument("bad value for " + name + ": " + arg);
            return v;
        }
    };
    
    template <>
    struct Option<double> {
        static double parse(const std::string& name, const char *arg, char **endp) {
            float v = std::strtod(arg, endp);
            if (*endp == arg)
                throw std::invalid_argument("bad value for " + name + ": " + arg);
            return v;
        }
    };

    template <class S, int dim>
    struct Option<Eigen::Matrix<S, dim, 1>> {
        static Eigen::Matrix<S, dim, 1> parse(const std::string& name, const char *arg, char **endp) {
            Eigen::Matrix<S, dim, 1> q;
            q[0] = Option<S>::parse(name + "[0]", arg, endp);
            for (int i=1 ; i<dim ; ++i) {
                if (**endp != ',')
                    throw std::invalid_argument("expected comma");
                q[i] = Option<S>::parse(name + "[" + std::to_string(i) + "]", *endp + 1, endp);
            }
            return q;
        }
    };

    template <class S>
    struct Option<Eigen::Quaternion<S>> {
        static Eigen::Quaternion<S> parse(const std::string& name, const char *arg, char **endp) {
            auto v = Option<Eigen::Matrix<S, 4, 1>>::parse(name, arg, endp);
            Eigen::Quaternion<S> q;
            q = Eigen::AngleAxis<S>{v[0], v.template tail<3>().normalized()};
            return q;
        }
    };

    template <class T>
    struct Option<std::optional<T>> {
        static std::optional<T> parse(const std::string& name, const char *arg, char **endp) {
            return Option<T>::parse(name, arg, endp);
        }
    };

    template <class A, class B>
    struct Option<std::tuple<A, B>> {
        static std::tuple<A, B> parse(const std::string& name, const char *arg, char **endp) {
            A a = Option<A>::parse(name, arg, endp);
            if (**endp != ',')
                throw std::invalid_argument("expected comma");
            return { a, Option<B>::parse(name, *endp + 1, endp) };
        }
    };    

    template <class T>
    void parse(const std::string& name, T& value, const char *arg) {
        char *endp;
        value = Option<T>::parse(name, arg, &endp);
        if (*endp != '\0')
            throw std::invalid_argument("extra characters in option");
    }
    
    template <class Scenario>
    class App {
        using State = typename Scenario::State;
        using Distance = typename Scenario::Distance;
        using Bound = Eigen::Matrix<Distance, 3, 1>;

        std::string algorithm_;
        
        std::string envMesh_;
        std::string robotMesh_;

        std::optional<State> qStart_;
        std::optional<State> qGoal_;

        std::optional<Bound> qMin_;
        std::optional<Bound> qMax_;

        static void usage() {
            std::cerr << R"(Usage: [options]
Options:
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
                
                { NULL, 0, NULL, 0 }
            };

            for (int ch ; (ch = getopt_long(argc, argv, "a:e:r:g:s:m:M:", longopts, NULL)) != -1 ; ) {
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

        template <class Algorithm>
        void runImpl() {
            JI_LOG(INFO) << "start: " << qStart_;
            JI_LOG(INFO) << "goal: " << qGoal_;
            JI_LOG(INFO) << "bounds: " << *qMin_ << " to " << *qMax_;
    
            Planner<Scenario, Algorithm> planner{envMesh_, robotMesh_, *qGoal_, *qMin_, *qMax_, 0.1};

            planner.addStart(*qStart_);

            using Clock = std::chrono::steady_clock;
            auto start = Clock::now();
            planner.solve([&] { return planner.isSolved(); });

            JI_LOG(INFO) << "solution found after " << (Clock::now() - start);
            JI_LOG(INFO) << "graph size = " << planner.size();
            planner.solution([] (const State& q) {
                    JI_LOG(INFO) << "  " << q;
                });
        }

        void run() {
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
