#pragma once
#ifndef MPL_FEMO_FETCH_ROBOT_HPP
#define MPL_FEMO_FETCH_ROBOT_HPP

#include "twist.hpp"
#include <Eigen/Dense>

namespace mpl::demo {

    template <class S>
    class FetchRobot {
    public:
        using Scalar = S;
        using Transform = Eigen::Transform<Scalar, 3, Eigen::AffineCompact>;
        using Frame = Transform;
        using Vec3 = Eigen::Matrix<Scalar, 3, 1>;

        enum {
            kTorsoLiftJoint,
            
            kShoulderPanJoint,
            kShoulderLiftJoint,
            kUpperarmRollJoint,
            kElbowFlexJoint,
            kForearmRollJoint,
            kWristFlexJoint,
            kWristRollJoint,

            kDOF
        };

        static constexpr S PI = 3.14159265358979323846264338327950288419716939937510582097494459230781640628620L;
        
        using Config = Eigen::Matrix<S, kDOF, 1>;
        using Jacobian = Eigen::Matrix<S, 6, kDOF>;

        static Config jointMin() {
            Config q;
            q[kTorsoLiftJoint] = 0;
            q[kShoulderPanJoint] = -1.6056;
            q[kShoulderLiftJoint] = -1.221;
            q[kUpperarmRollJoint] = -std::numeric_limits<Scalar>::infinity();
            q[kElbowFlexJoint] = -2.251;
            q[kForearmRollJoint] = -std::numeric_limits<Scalar>::infinity();
            q[kWristFlexJoint] = -2.16;
            q[kWristRollJoint] = -std::numeric_limits<Scalar>::infinity();
            return q;
        }

        static Config jointMax() {
            Config q;
            q[kTorsoLiftJoint] = 0.38615;
            q[kShoulderPanJoint] = 1.6056;
            q[kShoulderLiftJoint] = 1.518;
            q[kUpperarmRollJoint] = std::numeric_limits<Scalar>::infinity();
            q[kElbowFlexJoint] = 2.251;
            q[kForearmRollJoint] = std::numeric_limits<Scalar>::infinity();
            q[kWristFlexJoint] = 2.16;
            q[kWristRollJoint] = std::numeric_limits<Scalar>::infinity();
            return q;
        }

        static Config vMax() {
            Config q;
            q[kTorsoLiftJoint] = 0.1;
            q[kShoulderPanJoint] = 1.256;
            q[kShoulderLiftJoint] = 1.454;
            q[kUpperarmRollJoint] = 1.571;
            q[kElbowFlexJoint] = 1.521;
            q[kForearmRollJoint] = 1.571;
            q[kWristFlexJoint] = 2.268;
            q[kWristRollJoint] = 2.268;
            return q;
        }

        static Config eMax() {
            Config q;
            q[kTorsoLiftJoint] = 450.0;
            q[kShoulderPanJoint] = 33.82;
            q[kShoulderLiftJoint] = 131.76;
            q[kUpperarmRollJoint] = 76.94;
            q[kElbowFlexJoint] = 66.18;
            q[kForearmRollJoint] = 29.35;
            q[kWristFlexJoint] = 25.7;
            q[kWristRollJoint] = 7.36;
            return q;
        }

    public:
        template <class RNG>
        static Config randomConfig(RNG& rng) {
            // sample unbounded joints in the range -PI to +PI.  Note:
            // torso_lift is included in this cwise min/max, but to no
            // effect since its bounds are less than +/- 3.14 meters.
            static Config min = jointMax().cwiseMin(-PI);
            static Config max = jointMin().cwiseMax(+PI);
            Config q;
            for (int i=0 ; i < kDOF ; ++i) {
                std::uniform_real_distribution<S> dist(min[i], max[i]);
                q[i] = dist(rng);
            }
            return q;
        }

    private:
        Config config_;

        Frame baseLink_{Frame::Identity()};
        
        Frame torsoLiftJointOrigin_;
        Frame torsoLiftLink_;
        Frame shoulderPanJointOrigin_;
        Frame shoulderPanLink_;
        Frame shoulderLiftJointOrigin_;
        Frame shoulderLiftLink_;
        Frame upperarmRollJointOrigin_;
        Frame upperarmRollLink_;
        Frame elbowFlexJointOrigin_;
        Frame elbowFlexLink_;
        Frame forearmRollJointOrigin_;
        Frame forearmRollLink_;
        Frame wristFlexJointOrigin_;
        Frame wristFlexLink_;
        Frame wristRollJointOrigin_;
        Frame wristRollLink_;
        Frame gripperAxis_;

        void fk() {
            using T = Eigen::Translation<Scalar, 3>;
            using R = Eigen::AngleAxis<Scalar>;
            
            torsoLiftJointOrigin_ = baseLink_ * T(-0.086875, 0, 0.37743);
            torsoLiftLink_ = torsoLiftJointOrigin_ * T(0, 0, config_[kTorsoLiftJoint]);

            shoulderPanJointOrigin_ = torsoLiftLink_ * T(0.119525, 0, 0.34858);
            shoulderPanLink_ = shoulderPanJointOrigin_ * R(config_[kShoulderPanJoint], Vec3::UnitZ());
            
            shoulderLiftJointOrigin_ = shoulderPanLink_ * T(0.117, 0, 0.0599999999999999);
            shoulderLiftLink_ = shoulderLiftJointOrigin_ * R(config_[kShoulderLiftJoint], Vec3::UnitY());
            
            upperarmRollJointOrigin_ = shoulderLiftLink_ * T(0.219, 0, 0);
            upperarmRollLink_ = upperarmRollJointOrigin_ * R(config_[kUpperarmRollJoint], Vec3::UnitX());

            elbowFlexJointOrigin_ = upperarmRollLink_ * T(0.133, 0, 0);
            elbowFlexLink_ = elbowFlexJointOrigin_ * R(config_[kElbowFlexJoint], Vec3::UnitY());
            
            forearmRollJointOrigin_ = elbowFlexLink_ * T(0.197, 0, 0);
            forearmRollLink_ = forearmRollJointOrigin_ * R(config_[kForearmRollJoint], Vec3::UnitX());
            
            wristFlexJointOrigin_ = forearmRollLink_ * T(0.1245, 0, 0);
            wristFlexLink_ = wristFlexJointOrigin_ * R(config_[kWristFlexJoint], Vec3::UnitY());
            
            wristRollJointOrigin_ = wristFlexLink_ * T(0.1385, 0, 0);
            wristRollLink_ = wristRollJointOrigin_ * R(config_[kWristRollJoint], Vec3::UnitX());

            gripperAxis_ = wristRollLink_ * T(0.16645, 0, 0);


            // headPanJointOrigin_ = torsoLiftLink_ * T(0.053125, 0, 0.603001417713939);
            // headPanLink_ = headPanJointOrigin_ * R(config_[kHeadPanJoint], Vec3::UnitZ());

            // headTiltJointOrigin_ = headPanLink_ * T(0.14253, 0, 0.057999);
            // headTiltLink_ = headTiltJointOrigin_ * R(config_[kHeadTiltJoint], Vec3::UnitY());

            // rGripperFingerJointOrigin_ = gripperAxis_ * T(0, 0.015425, 0);
            // lGripperFingerJointOrigin_ = gripperAxis_ * T(0,-0.015425, 0);
        }

    public:
        FetchRobot() {
        }

        FetchRobot(const Config& q) {
            setConfig(q);
        }

        const Config& config() const {
            return config_;
        }

        void setConfig(const Config& q) {
            config_ = q;
            fk();
        }

        const Frame& baseLink() const { return baseLink_; }
        const Frame& torsoLiftJointOrigin() const { return torsoLiftJointOrigin_; }
        const Frame& torsoLiftLink() const { return torsoLiftLink_; }
        // const Frame& headPanJointOrigin() const { return headPanJointOrigin_; }
        // const Frame& headPanLink() const { return headPanLink_; }
        // const Frame& headTiltJointOrigin() const { return headTiltJointOrigin_; }
        // const Frame& headTiltLink() const { return headTiltLink_; }
        const Frame& shoulderPanJointOrigin() const { return shoulderPanJointOrigin_; }
        const Frame& shoulderPanLink() const { return shoulderPanLink_; }
        const Frame& shoulderLiftJointOrigin() const { return shoulderLiftJointOrigin_; }
        const Frame& shoulderLiftLink() const { return shoulderLiftLink_; }
        const Frame& upperarmRollJointOrigin() const { return upperarmRollJointOrigin_; }
        const Frame& upperarmRollLink() const { return upperarmRollLink_; }
        const Frame& elbowFlexJointOrigin() const { return elbowFlexJointOrigin_; }
        const Frame& elbowFlexLink() const { return elbowFlexLink_; }
        const Frame& forearmRollJointOrigin() const { return forearmRollJointOrigin_; }
        const Frame& forearmRollLink() const { return forearmRollLink_; }
        const Frame& wristFlexJointOrigin() const { return wristFlexJointOrigin_; }
        const Frame& wristFlexLink() const { return wristFlexLink_; }
        const Frame& wristRollJointOrigin() const { return wristRollJointOrigin_; }
        const Frame& wristRollLink() const { return wristRollLink_; }
        const Frame& gripperAxis() const { return gripperAxis_; }

        Jacobian jacobian() const {
            using Twist = mpl::demo::Twist<S>;

            // it might make sense to make `end` be a parameter or
            // modified by a parameter.
            const Frame& end = gripperAxis_;
            
            Jacobian J;
            
            J.col(kTorsoLiftJoint) = (torsoLiftJointOrigin_.linear() * Twist::translation(Vec3::UnitZ()))
                .refPoint(end.translation() - torsoLiftLink_.translation()).matrix();
                
            J.col(kShoulderPanJoint) = (shoulderPanJointOrigin_.linear() * Twist::rotation(Vec3::UnitZ()))
                .refPoint(end.translation() - shoulderPanLink_.translation()).matrix();
                
            J.col(kShoulderLiftJoint) = (shoulderLiftJointOrigin_.linear() * Twist::rotation(Vec3::UnitY()))
                .refPoint(end.translation() - shoulderLiftLink_.translation()).matrix();
            
            J.col(kUpperarmRollJoint) = (upperarmRollJointOrigin_.linear() * Twist::rotation(Vec3::UnitX()))
                .refPoint(end.translation() - upperarmRollLink_.translation()).matrix();
                
            J.col(kElbowFlexJoint) = (elbowFlexJointOrigin_.linear() * Twist::rotation(Vec3::UnitY()))
                .refPoint(end.translation() - elbowFlexLink_.translation()).matrix();
            
            J.col(kForearmRollJoint) = (forearmRollJointOrigin_.linear() * Twist::rotation(Vec3::UnitX()))
                .refPoint(end.translation() - forearmRollLink_.translation()).matrix();
            
            J.col(kWristFlexJoint) = (wristFlexJointOrigin_.linear() * Twist::rotation(Vec3::UnitY()))
                .refPoint(end.translation() - wristFlexLink_.translation()).matrix();
            
            J.col(kWristRollJoint) = (wristRollJointOrigin_.linear() * Twist::rotation(Vec3::UnitX()))
                .refPoint(end.translation() - wristRollLink_.translation()).matrix();
            
            return J;
        }
        
        // Compute an IK solution using LMA based upon the current
        // configuration.  This method modifies the current
        // configuration of the robot regardless of success of
        // failure.  If successful, the IK solution can be obtained
        // from the `config()` method, and the robot's joint frames
        // will be set to their FK solution.  If unsuccessful, the
        // `config()` of the robot will be the closest configuration
        // that the LMA solver could find.  It may or may not be a
        // useful configuration.
        bool ik(
            const Frame& target,
            const Eigen::Matrix<S, 6, 1>& L,
            S eps = 1e-5,
            unsigned maxIters = 500,
            S epsJoints = std::numeric_limits<S>::epsilon()*4)
        {
            using Twist = mpl::demo::Twist<S>;
            using Delta = Eigen::Matrix<S, 6, 1>;
            const S epsSqr = eps*eps;
            Config q = config_;

            Delta deltaPos = L.asDiagonal() * Twist::diff(gripperAxis_, target).matrix();
            S deltaPosNormSquared = deltaPos.squaredNorm();
            if (deltaPosNormSquared < epsSqr) {
                // already in a solution configuration
                return true;
            }

            Jacobian J = L.asDiagonal() * jacobian();
            S v = 2;
            S lambda = 10;

            for (unsigned iter = 1 ; iter <= maxIters ; ++iter) {
                Config grad = J.transpose() * deltaPos;
                Eigen::Matrix<S, kDOF, kDOF> jTj = (J.transpose() * J).eval();
                jTj.diagonal() *= 1 + 1/lambda;
                Config diffq = jTj.colPivHouseholderQr().solve(grad);
                S dNorm = diffq.cwiseAbs().maxCoeff();

                if (dNorm < epsJoints) {
                    // failed: joint increment is too small
                    return false;
                }

                if (grad.transpose() * grad < epsJoints*epsJoints) {
                    // failed: joint gradient is too small
                    return false;
                }

                // TODO: jointMin and jointMax are recomputed every
                // time (unless the compiler is smart enough to cache
                // it).  These should be cached in static variables
                // somewhere
                setConfig(
                    (q + diffq)
                    .cwiseMax(jointMin())
                    .cwiseMin(jointMax()));

                Delta deltaPosNew = L.asDiagonal() * Twist::diff(gripperAxis_, target).matrix();
                S deltaPosNewNormSquared = deltaPosNew.squaredNorm();

                S rho = (deltaPosNormSquared - deltaPosNewNormSquared)
                    / (diffq.transpose() * (lambda * diffq + grad));

                if (rho <= 0) {
                    lambda *= v;
                    v *= 2;
                    if (lambda == std::numeric_limits<Scalar>::infinity()) {
                        // failed: lambda overflow
                        return false;
                    }
                } else {
                    q = config_;
                    deltaPos = deltaPosNew;
                    deltaPosNormSquared = deltaPosNewNormSquared;
                    if (deltaPosNewNormSquared < epsSqr)
                        return true; // solved!

                    J = L.asDiagonal() * jacobian();
                    S x = 2*rho - 1;
                    lambda *= std::max(1/3.0, 1 - x*x*x);
                    v = 2;
                }                     
            }

            // failed: too many iterations
            return false;
        }
    };
}

#endif
