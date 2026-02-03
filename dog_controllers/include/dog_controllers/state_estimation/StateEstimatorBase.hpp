#pragma once

#include <rclcpp/rclcpp.hpp>
#include "dog_data_bridge.hpp"
#include <ocs2_core/Types.h>
#include <ocs2_pinocchio_interface/PinocchioInterface.h>
#include <ocs2_centroidal_model/CentroidalModelInfo.h>
#include <ocs2_pinocchio_interface/PinocchioEndEffectorKinematics.h>

namespace dog_controllers
{
    using namespace ocs2;

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
        // 构造函数
        StateEstimatorBase() = default;
        ~StateEstimatorBase() = default;

        void init(const DogDataBridge *bridge,
                  PinocchioInterface pinocchioInterface,
                  CentroidalModelInfo info,
                  PinocchioEndEffectorKinematics &eeKinematics);

        virtual const vector_t &estimate() = 0;

        const DogDataBridge *bridgePtr_;

        PinocchioInterface pinocchioInterface_;
        CentroidalModelInfo centroidalModelInfo_;
        std::unique_ptr<PinocchioEndEffectorKinematics> eeKinematics_;

        EstimatorResults results;
    };
}