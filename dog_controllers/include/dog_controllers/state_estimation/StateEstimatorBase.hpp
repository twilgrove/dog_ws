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
        double time = 0.0; // 时间戳
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
        StateEstimatorBase(const LegData *legsPtr_,
                           const ImuData *imuPtr_,
                           PinocchioInterface pinocchioInterface,
                           CentroidalModelInfo info,
                           const PinocchioEndEffectorKinematics &eeKinematics, rclcpp_lifecycle::LifecycleNode::SharedPtr &node);
        virtual ~StateEstimatorBase() = default;

        virtual const vector_t &estimate() = 0;
        EstimatorResults results;

    protected:
        const LegData *legsPtr_ = nullptr;
        const ImuData *imuPtr_ = nullptr;

        PinocchioInterface pinocchioInterface_;
        CentroidalModelInfo centroidalModelInfo_;
        std::unique_ptr<PinocchioEndEffectorKinematics> eeKinematics_;

        rclcpp_lifecycle::LifecycleNode::SharedPtr node_;

        void updateGenericResults();

        template <typename SCALAR_T>
        inline Eigen::Matrix<SCALAR_T, 3, 1> quatToZyx(const Eigen::Quaternion<SCALAR_T> &q)
        {
            using vector3_t = Eigen::Matrix<SCALAR_T, 3, 1>;
            vector3_t zyx;

            const SCALAR_T w = q.w();
            const SCALAR_T x = q.x();
            const SCALAR_T y = q.y();
            const SCALAR_T z = q.z();

            SCALAR_T sinP = -2.0 * (x * z - w * y);

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