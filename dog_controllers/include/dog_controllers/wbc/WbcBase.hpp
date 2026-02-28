#pragma once
#include <Eigen/Core>
#include <pinocchio/fwd.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include "wbc/Task.h"
#include <ocs2_core/Types.h>
#include <ocs2_core/misc/Benchmark.h>
#include <ocs2_legged_robot/common/Types.h>
#include <ocs2_centroidal_model/CentroidalModelInfo.h>
#include <ocs2_centroidal_model/CentroidalModelPinocchioMapping.h>
#include <ocs2_pinocchio_interface/PinocchioInterface.h>
#include <ocs2_pinocchio_interface/PinocchioEndEffectorKinematics.h>
#include <ocs2_legged_robot/gait/MotionPhaseDefinition.h>
namespace dog_controllers
{
    using namespace ocs2;
    using namespace ocs2::legged_robot;
    class WbcBase
    {
        using Vector6 = Eigen::Matrix<scalar_t, 6, 1>;
        using Matrix6 = Eigen::Matrix<scalar_t, 6, 6>;

    public:
        WbcBase(
            const PinocchioInterface &pinocchioInterface,
            const CentroidalModelInfo &info,
            const PinocchioEndEffectorKinematics &eeKinematics,
            rclcpp_lifecycle::LifecycleNode::SharedPtr &node);

        virtual ~WbcBase() = default;

        virtual vector_t update(const vector_t &stateDesired,
                                const vector_t &inputDesired,
                                const vector_t &rbdStateMeasured,
                                size_t mode,
                                scalar_t period);

        // 更新测量状态
        void updateMeasured(const vector_t &rbdStateMeasured);

        // 更新期望状态
        void updateDesired(const vector_t &stateDesired, const vector_t &inputDesired);

        // 构建浮动基座动力学方程任务
        Task formulateFloatingBaseEomTask();

        // 构建关节力矩限制任务
        Task formulateTorqueLimitsTask();

        // 构建无接触运动任务
        Task formulateNoContactMotionTask();

        // 构建摩擦锥约束任务
        Task formulateFrictionConeTask();

        // 构建基座加速度跟踪任务
        Task formulateBaseAccelTask(const vector_t &stateDesired, const vector_t &inputDesired, scalar_t period);

        // 构建摆动腿轨迹跟踪任务
        Task formulateSwingLegTask();

        // 构建接触力跟踪任务
        Task formulateContactForceTask(const vector_t &inputDesired) const;

        Task formulateBaseAccelTask2(const vector_t &rbdStateMeasured);
        Task formulateContactForceTask2();
        Task formulateJointRegularizationTask();

        // --- 核心模型与映射 ---
        CentroidalModelInfo info_;                // 机器人模型参数
        CentroidalModelPinocchioMapping mapping_; // 质心到全身状态映射

        // --- 动力学/运动学接口 ---
        PinocchioInterface pinocchioInterfaceMeasured_;                // 测量值计算接口
        PinocchioInterface pinocchioInterfaceDesired_;                 // 期望值计算接口
        std::unique_ptr<PinocchioEndEffectorKinematics> eeKinematics_; // 足端运动学

        // --- 状态缓存 ---
        vector_t qMeasured_{}, vMeasured_{};       // 关节位置/速度观测
        vector_t inputLast_{};                     // 上一时刻控制量
        matrix_t j_{}, dj_{};                      // 接触雅可比及其导数
        Eigen::Matrix<scalar_t, 6, 18> jac;        // 足端雅可比矩阵（6x18，包含位置和旋转部分）
        contact_flag_t contactFlag_{};             // 接触状态标志
        size_t numDecisionVars_{}, numContacts_{}; // 决策变量与接触点数
        rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
        benchmark::RepeatedTimer wbcTimer_;
        int wbcnWSR_;
        bool firstRun_ = true;
        // --- 任务增益与物理限制 ---
        vector_t torqueLimits_{};        // 关节力矩限制
        scalar_t frictionCoeff_{};       // 摩擦系数
        scalar_t swingKp_{}, swingKd_{}; // 摆动腿 PD 增益
    };
}