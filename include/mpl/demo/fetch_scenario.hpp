#pragma once
#ifndef MPL_DEMO_FETCH_SCENARIO_HPP
#define MPL_DEMO_FETCH_SCENARIO_HPP

#include "fetch_robot.hpp"
#include "load_mesh.hpp"
#include <nigh/lp_space.hpp>

namespace mpl::demo {
    
    template <class S>
    class FetchScenario {
    public:
        using Scalar = S;
        using Robot = FetchRobot<S>;
        using Frame = typename Robot::Frame;
        using Transform = Frame;
        using State = typename Robot::Config;
        using Space = unc::robotics::nigh::L1Space<S, Robot::kDOF>;
        using Distance = S;
        
        static_assert(std::is_same_v<State, typename Space::Type>, "odd that this wouldn't be the case");

    private:
        using Mesh = fcl::BVHModel<fcl::OBBRSS<S>>;

        Space space_;
        
        Mesh environment_;
        Frame envFrame_{Frame::Identity()};

        Frame goal_;

        S invStepSize_;

        static const auto& ikWeights() {
            using Weights = Eigen::Matrix<S, 6, 1>;
            static Weights L = (Weights() << 1, 1, 1, 0.1, 0.1, 0.1).finished();
            return L;
        }

    public:
        FetchScenario(
            const std::string& envMesh,
            const Frame& goal,
            S checkResolution = 0.01)
            : environment_(MeshLoad<Mesh>::load(envMesh, false))
            , goal_(goal)
            , invStepSize_(1 / checkResolution)
        {
        }

        const Space& space() const {
            return space_;
        }

        template <class RNG>
        State randomSample(RNG& rng) {
            return Robot::randomConfig(rng);
        }

        // this isn't always a goal-biased sample
        template <class RNG>
        State sampleGoal(RNG& rng) {
            State q = randomSample(rng);
            Robot robot(q);
            return (robot.ik(goal_, ikWeights(), 1e-5, 50)) ? robot.config() : q;
        }


        bool isGoal(const State& q) const {
            // HACKY!  using ik solver to see if we're 0 steps away
            // from the goal!
            Robot robot(q);
            return robot.ik(goal_, ikWeights(), 1e-5, 0);
        }

        bool isValid(const State& q) const {
            Robot robot(q);
            
            if (robot.selfCollision())
                return false;

            if (robot.inCollisionWith(&environment_, envFrame_))
                return false;
            
            return true;
        }

        bool isValid(const State& from, const State& to) const {
            assert(isValid(from));
            if (!isValid(to))
                return false;

            std::size_t steps = std::ceil(space_.distance(from, to) * invStepSize_);

            // JI_LOG(DEBUG) << "steps = " << steps;
            
            if (steps < 2)
                return true;

            Distance delta = 1 / Distance(steps);
            // for (std::size_t i = 1 ; i < steps ; ++i)
            //     if (!isValid(interpolate(from, to, i*delta)))
            //         return false;
            
            // if (true) return true;
            
            std::array<std::pair<std::size_t, std::size_t>, 1024> queue;
            queue[0] = std::make_pair(std::size_t(1), steps-1);
            std::size_t qStart = 0, qEnd = 1;
            while (qStart != qEnd) {
                auto [min, max] = queue[qStart++ % queue.size()];
                if (min == max) {
                    if (!isValid(interpolate(from, to, min * delta)))
                        return false;
                } else if (qEnd + 2 < qStart + queue.size()) {
                    std::size_t mid = (min + max) / 2;
                    if (!isValid(interpolate(from, to, mid * delta)))
                        return false;
                    if (min < mid)
                        queue[qEnd++ % queue.size()] = std::make_pair(min, mid-1);
                    if (mid < max)
                        queue[qEnd++ % queue.size()] = std::make_pair(mid+1, max);
                } else {
                    // queue is full
                    for (std::size_t i=min ; i<=max ; ++i)
                        if (!isValid(interpolate(from, to, i*delta)))
                            return false;
                }
            }

            return true;
        }
        
    };
}

#endif
