#pragma once

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include "common/dog_data_bridge.hpp"
#include <ocs2_core/Types.h>
#include <ocs2_legged_robot/common/Types.h>
#include <ocs2_pinocchio_interface/PinocchioInterface.h>
#include <ocs2_centroidal_model/CentroidalModelInfo.h>
#include <ocs2_pinocchio_interface/PinocchioEndEffectorKinematics.h>
#include <ocs2_mpc/SystemObservation.h>
#include <ocs2_centroidal_model/CentroidalModelRbdConversions.h>
#include <ocs2_robotic_tools/common/RotationTransforms.h>
#include <pinocchio/algorithm/centroidal.hpp>
#include <pinocchio/algorithm/crba.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/rnea.hpp>

namespace dog_controllers
{
    using namespace ocs2;
    using namespace ocs2::legged_robot;
    struct EstimatorResults
    {
        /**
         * 36维全状态向量 (rbdState_36) 索引定义：
         * 0-2  : 基座姿态 - 欧拉角 (Euler Angles: Z,Y,X )
         * 3-5  : 基座位置 - 世界坐标系 (Base Position: x, y, z)
         * 6-17 : 12个关节角度 (Joint Angles: 顺序通常为 LF, RF, LH, RH 的 HAA, HFE, KFE)
         * 18-20: 基座角速度 - 机体坐标系/世界系 (Angular Velocity: wx, wy, wz)
         * 21-23: 基座线速度 - 世界坐标系 (Linear Velocity: vx, vy, vz)
         * 24-35: 12个关节速度 (Joint Velocities)
         */
        vector_t rbdState_36;
        contact_flag_t contactFlags_WBC; // 4位布尔数组 (WBC用)
        size_t contactFlags_MPC = 15;    // 整数索引 (MPC用)
        std::array<matrix3_t, 4> legJacobians;
    };
    class StateEstimatorBase
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        StateEstimatorBase(
            const PinocchioInterface &pinocchioInterface,
            const CentroidalModelInfo &info,
            const PinocchioEndEffectorKinematics &eeKinematics,
            rclcpp_lifecycle::LifecycleNode::SharedPtr &node);
        virtual ~StateEstimatorBase() = default;

        virtual const vector_t &estimate(const std::array<LegData, 4> &legsPtr, const ImuData &imuData, const rclcpp::Duration &period) = 0;
        EstimatorResults results;
        SystemObservation currentObservation_;

    protected:
        vector3_t zyxOffset_ = vector3_t::Zero();
        void updateObservationFromResults(const rclcpp::Duration &period);
        void updateGenericResults(const double &qw, const double &qx, const double &qy, const double &qz, const std::array<LegData, 4> &legsPtr);
        void updateJacobians();

        PinocchioInterface pinocchioInterface_;
        CentroidalModelInfo info_;
        std::unique_ptr<PinocchioEndEffectorKinematics> eeKinematics_;
        std::unique_ptr<CentroidalModelRbdConversions> rbdConversions_;

        rclcpp_lifecycle::LifecycleNode::SharedPtr node_;

        inline vector3_t quatToZyx(const Eigen::Quaternion<scalar_t> &q)
        {
            vector3_t zyx;
            scalar_t w = q.w(), x = q.x(), y = q.y(), z = q.z();

            scalar_t sinP = 2.0 * (w * y - z * x);
            if (std::abs(sinP) >= 1.0)
            {
                zyx(1) = std::copysign(M_PI / 2.0, sinP);
                zyx(0) = std::atan2(2.0 * (x * y + w * z), 1.0 - 2.0 * (y * y + z * z));
                zyx(2) = 0.0;
            }
            else
            {
                zyx(1) = std::asin(sinP);
                zyx(0) = std::atan2(2.0 * (x * y + w * z), 1.0 - 2.0 * (y * y + z * z));
                zyx(2) = std::atan2(2.0 * (y * z + w * x), 1.0 - 2.0 * (x * x + y * y));
            }
            return zyx;
        }
    };
}