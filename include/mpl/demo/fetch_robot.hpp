#pragma once
#ifndef MPL_FEMO_FETCH_ROBOT_HPP
#define MPL_FEMO_FETCH_ROBOT_HPP

#include "twist.hpp"
#include <Eigen/Dense>
#include <fcl/narrowphase/collision.h>

namespace mpl::demo {

    template <class S>
    class Origin {
        using Scalar = S;
        using Transform = fcl::Transform3<S>; // Eigen::Transform<Scalar, 3, Eigen::AffineCompact>;
        using Vec3 = Eigen::Matrix<Scalar, 3, 1>;
        
        Transform m_;

    public:
        Origin(Scalar tx, Scalar ty, Scalar tz, Scalar roll, Scalar pitch, Scalar yaw) {
            using R = Eigen::AngleAxis<Scalar>;
            
            m_.linear() =
                (R(yaw, Vec3::UnitZ()) * R(pitch, Vec3::UnitY()) * R(roll, Vec3::UnitX()))
                .toRotationMatrix();
            m_.translation() = Vec3{tx, ty, tz};
        }

        operator const Transform& () const {
            return m_;
        }
    };

    template <class S>
    struct FetchCollisionGeometry {
        using Scalar = S;
        
        fcl::Cylinder<S> base_{0.29, 0.32 - 0.02};
        fcl::Box<S> torsoLift_{0.18 + 0.08, 0.34, 1.5};
        fcl::Capsule<S> shoulderPan_{0.0525, 0.29};
        fcl::Cylinder<S> shoulderLift_{0.06, 0.145};
        fcl::Capsule<S> upperarmRoll_{0.0525, 0.24};
        fcl::Cylinder<S> elbowFlex_{0.0525, 0.14};
        fcl::Capsule<S> forearmRoll_{0.0525, 0.215};
        fcl::Cylinder<S> wristFlex_{0.0525, 0.155};
        fcl::Capsule<S> wristRoll_{0.0525, 0.09};
        fcl::Box<S> gripper_{0.12 + 0.0625 * 2, 0.125, 0.075};
        fcl::Capsule<S> neck_{0.085 * 1.1, 0.04 * 1.1};
        fcl::Cylinder<S> head_{0.24 * 1.05, 0.115 * 1.05};
    };
    
    template <class S>
    class FetchRobot {
    public:
        using Scalar = S;
        using Transform = fcl::Transform3<S>;
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

        // keep the gripper from hitting the floor
        static constexpr S floorClearance_ = 0.30;
        
        using Config = Eigen::Matrix<S, kDOF, 1>;
        using Jacobian = Eigen::Matrix<S, 6, kDOF>;

        // Collision geometry does not change from one robot to the
        // next, so we keep around a static copy of it.
        static const auto& cg() {
            static FetchCollisionGeometry<S> collisionGeometry_;
            return collisionGeometry_;
        }

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

        Frame tfBaseLink() const {
            const static Origin<Scalar> originBaseLink{0, 0, 0.1975, 0, 0, 0};
            return baseLink_ * originBaseLink;
        }

        Frame tfTorsoLiftLink() const {
            const static Origin<Scalar> originTorsoLiftLink{-0.09, 0, 0.31, 0, 0, 0};
            return torsoLiftLink_ * originTorsoLiftLink;
        }

        Frame tfShoulderPanLink() const {
            const static Origin<Scalar> originShoulderPanLink{0, 0, -0.025, 0, 0, 0};
            return shoulderPanLink_ * originShoulderPanLink;
        }

        Frame tfShoulderLiftLink() const {
            const static Origin<Scalar> originShoulderLiftLink{0, -0.0025, 0, 1.5708, 0, 0};
            return shoulderLiftLink_ * originShoulderLiftLink;
        }

        Frame tfUpperarmRollLink() const {
            const static Origin<Scalar> originUpperarmRollLink{-0.04, 0, 0, 0, 1.5708, 0};
            return upperarmRollLink_ * originUpperarmRollLink;
        }

        Frame tfElbowFlexLink() const {
            const static Origin<Scalar> originElbowFlexLink{0, 0, 0, 1.5708, 0, 0};
            return elbowFlexLink_ * originElbowFlexLink;
        }
        
        Frame tfForearmRollLink() const {
            const static Origin<Scalar> originForearmRollLink{-0.04, 0, 0, 0, 1.5708, 0};
            return forearmRollLink_ * originForearmRollLink;
        }

        Frame tfWristFlexLink() const {
            const static Origin<Scalar> originWristFlexLink{0, -0.01, 0, 1.5708, 0, 0};
            return wristFlexLink_ * originWristFlexLink;
        }

        Frame tfWristRollLink() const {
            const static Origin<Scalar> originWristRollLink{-0.045, 0, 0, 0, 1.5708, 0};
            return wristRollLink_ * originWristRollLink;
        }

        Frame tfGripperLink() const {
            const static Origin<Scalar> originGripperLink{-0.09, 0, 0.0025, 0, 0, 0};
            return gripperAxis_ * originGripperLink;
        }

        Frame tfNeckLink() const {
            const static Origin<Scalar> originNeckLink{0.053125, 0, 0.603001417713939, 0, 0, 0};
            return torsoLiftLink_ * originNeckLink;
        }

        Frame tfHeadLink() const {
            const static Origin<Scalar> originHeadLink{0.053125, 0, 0.603001417713939 + 0.115/2, 0, 0, 0};
            return torsoLiftLink_ * originHeadLink;
        }
        
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
        // configuration.  This method returns true if successful,
        // false if not.  This method modifies the current
        // configuration of the robot regardless of success or
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

        bool selfCollision() const {
            fcl::CollisionRequest<S> req;
            fcl::CollisionResult<S> res;

            if (fcl::collide(&cg().base_, tfBaseLink(), &cg().forearmRoll_, tfForearmRollLink(), req, res))
                return true;

            if (fcl::collide(&cg().base_, tfBaseLink(), &cg().wristFlex_, tfWristFlexLink(), req, res))
                return true;

            if (fcl::collide(&cg().base_, tfBaseLink(), &cg().wristRoll_, tfWristRollLink(), req, res))
                return true;

            if (fcl::collide(&cg().base_, tfBaseLink(), &cg().gripper_, tfGripperLink(), req, res))
                return true;

            if (fcl::collide(&cg().torsoLift_, tfTorsoLiftLink(), &cg().forearmRoll_, tfForearmRollLink(), req, res))
                return true;
            
            if (fcl::collide(&cg().torsoLift_, tfTorsoLiftLink(), &cg().wristFlex_, tfWristFlexLink(), req, res))
                return true;
            
            if (fcl::collide(&cg().torsoLift_, tfTorsoLiftLink(), &cg().wristRoll_, tfWristRollLink(), req, res))
                return true;
            
            if (fcl::collide(&cg().torsoLift_, tfTorsoLiftLink(), &cg().gripper_, tfGripperLink(), req, res))
                return true;
            
            if (fcl::collide(&cg().shoulderPan_, tfShoulderPanLink(), &cg().forearmRoll_, tfForearmRollLink(), req, res))
                return true;
            
            if (fcl::collide(&cg().shoulderPan_, tfShoulderPanLink(), &cg().wristFlex_, tfWristFlexLink(), req, res))
                return true;
            
            if (fcl::collide(&cg().shoulderPan_, tfShoulderPanLink(), &cg().wristRoll_, tfWristRollLink(), req, res))
                return true;
            
            if (fcl::collide(&cg().shoulderPan_, tfShoulderPanLink(), &cg().gripper_, tfGripperLink(), req, res))
                return true;
            
            if (fcl::collide(&cg().shoulderLift_, tfShoulderLiftLink(), &cg().wristRoll_, tfWristRollLink(), req, res))
                return true;
            
            if (fcl::collide(&cg().upperarmRoll_, tfUpperarmRollLink(), &cg().wristRoll_, tfWristRollLink(), req, res))
                return true;
            
            if (fcl::collide(&cg().upperarmRoll_, tfUpperarmRollLink(), &cg().gripper_, tfGripperLink(), req, res))
                return true;
            
            if (fcl::collide(&cg().upperarmRoll_, tfUpperarmRollLink(), &cg().head_, tfHeadLink(), req, res))
                return true;
            
            if (fcl::collide(&cg().elbowFlex_, tfElbowFlexLink(), &cg().head_, tfHeadLink(), req, res))
                return true;
            
            if (fcl::collide(&cg().forearmRoll_, tfForearmRollLink(), &cg().head_, tfHeadLink(), req, res))
                return true;
            
            if (fcl::collide(&cg().wristFlex_, tfWristFlexLink(), &cg().neck_, tfNeckLink(), req, res))
                return true;
            
            if (fcl::collide(&cg().wristFlex_, tfWristFlexLink(), &cg().head_, tfHeadLink(), req, res))
                return true;
            
            if (fcl::collide(&cg().wristRoll_, tfWristRollLink(), &cg().neck_, tfNeckLink(), req, res))
                return true;
            
            if (fcl::collide(&cg().wristRoll_, tfWristRollLink(), &cg().head_, tfHeadLink(), req, res))
                return true;
            
            if (fcl::collide(&cg().gripper_, tfGripperLink(), &cg().neck_, tfNeckLink(), req, res))
                return true;
            
            if (fcl::collide(&cg().gripper_, tfGripperLink(), &cg().head_, tfHeadLink(), req, res))
                return true;

            if (gripperAxis_.translation()[2] < floorClearance_)
                return true;
            
            return false;
        }

        bool inCollisionWith(
            const fcl::CollisionGeometry<S>* geom,
            const Frame& frame)
        {
            fcl::CollisionRequest<S> req;
            fcl::CollisionResult<S> res;

            if (fcl::collide(geom, frame, &cg().base_, tfBaseLink(), req, res))
                return true;

            if (fcl::collide(geom, frame, &cg().torsoLift_, tfTorsoLiftLink(), req, res))
                return true;

            if (fcl::collide(geom, frame, &cg().shoulderPan_, tfShoulderPanLink(), req, res))
                return true;

            if (fcl::collide(geom, frame, &cg().shoulderLift_, tfShoulderLiftLink(), req, res))
                return true;

            if (fcl::collide(geom, frame, &cg().upperarmRoll_, tfUpperarmRollLink(), req, res))
                return true;
            
            if (fcl::collide(geom, frame, &cg().elbowFlex_, tfElbowFlexLink(), req, res))
                return true;

            if (fcl::collide(geom, frame, &cg().forearmRoll_, tfForearmRollLink(), req, res))
                return true;
            
            if (fcl::collide(geom, frame, &cg().wristFlex_, tfWristFlexLink(), req, res))
                return true;

            if (fcl::collide(geom, frame, &cg().wristRoll_, tfWristRollLink(), req, res))
                return true;

            if (fcl::collide(geom, frame, &cg().gripper_, tfGripperLink(), req, res))
                return true;

            if (fcl::collide(geom, frame, &cg().wristFlex_, tfWristFlexLink(), req, res))
                return true;
            
            if (fcl::collide(geom, frame, &cg().head_, tfHeadLink(), req, res))
                return true;

            return false;
        }
    };
}

#endif
