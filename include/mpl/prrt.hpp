#pragma once
#ifndef MPL_PRRT_HPP
#define MPL_PRRT_HPP

#include <jilog.hpp>
#include <random>
#include <vector>
#include <thread>
#include <deque>
#include <nigh/auto_strategy.hpp>
#include <omp.h>
#include "interpolate.hpp"
#include "planner.hpp"

namespace mpl {
    class PRRT {};
    
    template <class Scenario>
    class Planner<Scenario, PRRT> {
        using Space = typename Scenario::Space;
        using State = typename Scenario::State;
        using Distance = typename Scenario::Distance;
        
        using RNG = std::mt19937_64;
        
        class Node;
        class NodeKey;
        class Thread;

        using Concurrency = unc::robotics::nigh::Concurrent;
        using NNStrategy = unc::robotics::nigh::auto_strategy_t<Space, Concurrency>;

        Scenario scenario_;
        
        unc::robotics::nigh::Nigh<Node*, Space, NodeKey, Concurrency, NNStrategy> nn_;

        Distance maxDistance_{std::numeric_limits<Distance>::infinity()};
        
        std::vector<Thread> threads_;

        std::atomic<Node*> solution_{nullptr};
        
        decltype(auto) randomSample(RNG& rng) {
            return scenario_.randomSample(rng);
        }

        decltype(auto) sampleGoal(RNG& rng) {
            return scenario_.sampleGoal(rng);
        }

        void addNode(Node *n) {
            nn_.insert(n);
            if (scenario_.isGoal(n->state()))
                solution_.store(n, std::memory_order_release);
        }

        decltype(auto) nearest(const State& q) {
            return nn_.nearest(q);
        }

        decltype(auto) isValid(const State& q) {
            return scenario_.isValid(q);
        }

        decltype(auto) isValid(const State& from, const State& to) {
            return scenario_.isValid(from, to);
        }
        
    public:
        template <class ... Args>
        Planner(Args&& ... args)
            : scenario_(std::forward<Args>(args)...)
        {
            int nThreads = std::max(1, omp_get_max_threads());
            threads_.reserve(nThreads);
            std::random_device rdev;
            std::array<typename RNG::result_type, RNG::state_size> rdata;
            for (int i=0 ; i<nThreads ; ++i) {
                std::generate(rdata.begin(), rdata.end(), std::ref(rdev));
                std::seed_seq sseq(rdata.begin(), rdata.end());
                threads_.emplace_back(sseq);
            }

            setGoalBias(0.01);
        }

        const Space& space() const {
            return scenario_.space();
        }
        
        void setGoalBias(Distance d) {
            threads_[0].setGoalBias(d * threads_.size());
        }

        bool isSolved() const {
            return solution_.load(std::memory_order_relaxed) != nullptr;
        }

        void addStart(const State& q) {
            threads_[0].addStart(*this, q);
        }

        std::size_t size() const {
            return nn_.size();
        }

        template <class Fn>
        void solution(Fn fn) const {
            for (const Node* node = solution_.load(std::memory_order_acquire) ;
                 node != nullptr ;
                 node = node->parent())
                fn(node->state());
        }

        template <class DoneFn>
        void solve(DoneFn doneFn) {
            int nThreads = threads_.size();
            JI_LOG(INFO) << "solving on " << nThreads << " threads";
            std::atomic_bool done{false};
#pragma omp parallel for shared(done) schedule(static, 1) num_threads(nThreads)
            for (int i=0 ; i<nThreads ; ++i) {
                try {
                    if (int tNo = omp_get_thread_num()) {
                        threads_[tNo].solve(*this, [&] { return done.load(std::memory_order_relaxed); });
                    } else {
                        threads_[0].solve(*this, doneFn);
                        done.store(true);
                    }
                } catch (const std::exception& ex) {
                    JI_LOG(ERROR) << "solve died with exception: " << ex.what();
                }
            }
        }
    };

    template <class Scenario>
    class Planner<Scenario, PRRT>::Node {
        const Node* parent_;
        State state_;

    public:
        Node(const Node* parent, const State& q)
            : parent_(parent)
            , state_(q)
        {
        }

        const Node* parent() const {
            return parent_;
        }
        
        const State& state() const {
            return state_;
        }
    };

    template <class Scenario>
    struct Planner<Scenario, PRRT>::NodeKey {
        const State& operator() (const Node* node) const {
            return node->state();
        }
    };

    template <class Scenario>
    class Planner<Scenario, PRRT>::Thread {
        RNG rng_;
        std::deque<Node> nodes_;

        Distance goalBias_{0};

    public:
        template <class SSeq>
        Thread(SSeq& sseq)
            : rng_(sseq)
        {
        }

        void setGoalBias(Distance d) {
            JI_LOG(TRACE) << "thread goal bias set to " << d;
            goalBias_ = d;
        }

        void addStart(Planner& planner, const State& q) {
            planner.addNode(&nodes_.emplace_back(nullptr, q));
        }
        
        void addSample(Planner& planner, State qRand) {
            auto [nNear, d] = planner.nearest(qRand).value();

            if (d == 0)
                return;

            if (d > planner.maxDistance_)
                qRand = interpolate(nNear->state(), qRand, planner.maxDistance_ / d);

            if (!planner.isValid(qRand))
                return;

            if (!planner.isValid(nNear->state(), qRand))
                return;

            Node *nNew = &nodes_.emplace_back(nNear, qRand);
            planner.addNode(nNew);
        }

        State randomSample(Planner& planner) {
            static std::uniform_real_distribution<Distance> unif01;
            return (goalBias_ > 0 && unif01(rng_) < goalBias_)
                ? planner.sampleGoal(rng_)
                : planner.randomSample(rng_);
        }

        template <class Done>
        void solve(Planner& planner, Done done) {
            while (!done())
                addSample(planner, randomSample(planner));
        }
    };
}

#endif
