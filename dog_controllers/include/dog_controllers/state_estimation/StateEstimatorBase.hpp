#pragma once

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include "common/dog_data_bridge.hpp"
#include <ocs2_core/Types.h>
#include <ocs2_pinocchio_interface/PinocchioInterface.h>
#include <ocs2_centroidal_model/CentroidalModelInfo.h>
#include <ocs2_pinocchio_interface/PinocchioEndEffectorKinematics.h>
#include <ocs2_robotic_tools/common/RotationTransforms.h>

namespace dog_controllers
{
    using namespace ocs2;
    using quaternion_t = Eigen::Quaternion<scalar_t>;
    using vector3_t = Eigen::Matrix<scalar_t, 3, 1>;
    using matrix3_t = Eigen::Matrix<scalar_t, 3, 3>;

    struct EstimatorResults
    {
        /**
         * 36维全状态向量 (rbdState_36) 索引定义：
         * 0-2  : 基座姿态 - 欧拉角 (Euler Angles: Roll, Pitch, Yaw)
         * 3-5  : 基座位置 - 世界坐标系 (Base Position: x, y, z)
         * 6-17 : 12个关节角度 (Joint Angles: 顺序通常为 LF, RF, LH, RH 的 HAA, HFE, KFE)
         * 18-20: 基座角速度 - 机体坐标系/世界系 (Angular Velocity: wx, wy, wz)
         * 21-23: 基座线速度 - 世界坐标系 (Linear Velocity: vx, vy, vz)
         * 24-35: 12个关节速度 (Joint Velocities)
         */
        vector_t rbdState_36;
        std::array<bool, 4> contactFlags_WBC; // 4位布尔数组 (WBC用)
        size_t contactFlags_MPC = 15;         // 整数索引 (MPC用)
    };
    class StateEstimatorBase
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        StateEstimatorBase(
            const PinocchioInterface &pinocchioInterface,
            const PinocchioEndEffectorKinematics &eeKinematics,
            rclcpp_lifecycle::LifecycleNode::SharedPtr &node);
        virtual ~StateEstimatorBase() = default;

        virtual const vector_t &estimate(const std::array<LegData, 4> &legsPtr, const ImuData &imuData) = 0;
        EstimatorResults results;

    protected:
        vector3_t zyxOffset_ = vector3_t::Zero(); // 欧拉角偏移量（用于校准）

        PinocchioInterface pinocchioInterface_;
        std::unique_ptr<PinocchioEndEffectorKinematics> eeKinematics_;

        rclcpp_lifecycle::LifecycleNode::SharedPtr node_;

        void updateGenericResults(const double &qw, const double &qx, const double &qy, const double &qz, const std::array<LegData, 4> &legsPtr);

        inline vector3_t quatToZyx(const Eigen::Quaternion<scalar_t> &q)
        {
            vector3_t zyx;

            const scalar_t w = q.w();
            const scalar_t x = q.x();
            const scalar_t y = q.y();
            const scalar_t z = q.z();

            scalar_t sinP = -2.0 * (x * z - w * y);

            if (std::abs(sinP) >= 1.0)
            {
                zyx(1) = std::copysign(M_PI / 2.0, sinP);
                zyx(2) = 0.0;
                zyx(0) = std::atan2(2.0 * (x * y + w * z), 1.0 - 2.0 * (y * y + z * z));
            }
            else
            {
                zyx(1) = std::asin(sinP);
                zyx(0) = std::atan2(2.0 * (x * y + w * z), 1.0 - 2.0 * (x * x + y * y));
                zyx(2) = std::atan2(2.0 * (y * z + w * x), 1.0 - 2.0 * (x * x + y * y));
            }
            return zyx;
        }
    };
}