#pragma once
#ifndef MPL_DEMO_SE3_APP_OPTIONS_HPP
#define MPL_DEMO_SE3_APP_OPTIONS_HPP

#include <string>
#include <optional>
#include <Eigen/Dense>
#include <getopt.h>
#include <iostream>
#include <cstdlib>
#include "../option.hpp"

namespace mpl::demo {
    template <class Scenario>
    class SE3AppOptions {            
        using State = typename Scenario::State;
        using Distance = typename Scenario::Distance;
        using Bound = Eigen::Matrix<Distance, 3, 1>;

        std::string algorithm_;
        std::string coordinator_;
        std::uint64_t problemId_;
        
        std::string envMesh_;
        std::string robotMesh_;

        std::optional<State> qStart_;
        std::optional<State> qGoal_;

        std::optional<Bound> qMin_;
        std::optional<Bound> qMax_;

        double timeLimit_{std::numeric_limits<double>::infinity()};
        double discretization_{0};

        static void usage() {
            std::cerr << R"(Usage: [options]
Options:
  -c, --coordinator=HOST:PORT
  -I, --problem-id=ID
  -t, --time-limit=TIME
  -e, --env=MESH
  -r, --robot=MESH
  -s, --start=W,I,J,K,X,Y,Z
  -g, --goal=W,I,J,K,X,Y,Z
  -m, --min=X,Y,Z
  -M, --max=X,Y,Z
  -d, --discretization=DIST
)";
        }

    public:
        SE3AppOptions(int argc, char *argv[]) {
            static struct option longopts[] = {
                { "algorithm", required_argument, NULL, 'a' },
                { "env", required_argument, NULL, 'e' },
                { "robot", required_argument, NULL, 'r' },
                { "goal", required_argument, NULL, 'g' },
                { "start", required_argument, NULL, 's' },
                { "min", required_argument, NULL, 'm' },
                { "max", required_argument, NULL, 'M' },
                { "coordinator", required_argument, NULL, 'c' },
                { "problem-id", required_argument, NULL, 'I' },
                { "time-limit", required_argument, NULL, 't' },
                { "discretization", required_argument, NULL, 'd' },
                
                { NULL, 0, NULL, 0 }
            };

            for (int ch ; (ch = getopt_long(argc, argv, "a:e:r:g:s:m:M:c:I:t:d:", longopts, NULL)) != -1 ; ) {
                char *endp;
                
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
                case 'I':
                    problemId_ = strtoull(optarg, &endp, 0);
                    if (endp == optarg || *endp)
                        throw std::invalid_argument("bad value for --problem-id");
                    break;
                case 't':
                    timeLimit_ = std::strtod(optarg, &endp);
                    if (endp == optarg || *endp || timeLimit_ < 0)
                        throw std::invalid_argument("bad value for --time-limit");
                    break;
                case 'd':
                    discretization_ = std::strtod(optarg, &endp);
                    if (endp == optarg || *endp || discretization_ <= 0)
                        throw std::invalid_argument("bad value for --discretization");
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

        const auto& algorithm() const {
            return algorithm_;
        }

        const auto& coordinator() const {
            return coordinator_;
        }

        std::uint64_t problemId() const {
            return problemId_;
        }

        double timeLimit() const {
            return timeLimit_;
        }

        double discretization() const {
            return discretization_;
        }

        const auto& robot() const {
            return robotMesh_;
        }

        const auto& env() const {
            return envMesh_;
        }

        const auto& start() const {
            return qStart_.value();
        }

        const auto& goal() const {
            return qGoal_.value();
        }

        const auto& min() const {
            return qMin_.value();
        }

        const auto& max() const {
            return qMax_.value();
        }

    };
}

#endif
