/******************************************************************************
Copyright (c) 2021, Farbod Farshidian. All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

 * Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

 * Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

 * Neither the name of the copyright holder nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
******************************************************************************/

#pragma once

// ocs2
#include <ocs2_centroidal_model/FactoryFunctions.h>
#include <ocs2_core/Types.h>
#include <ocs2_core/penalties/Penalties.h>
#include <ocs2_ddp/DDP_Settings.h>
#include <ocs2_ipm/IpmSettings.h>
#include <ocs2_mpc/MPC_Settings.h>
#include <ocs2_oc/rollout/TimeTriggeredRollout.h>
#include <ocs2_pinocchio_interface/PinocchioInterface.h>
#include <ocs2_pinocchio_interface/PinocchioEndEffectorKinematics.h>
#include <ocs2_robotic_tools/common/RobotInterface.h>
#include <ocs2_robotic_tools/end_effector/EndEffectorKinematics.h>
#include <ocs2_sqp/SqpSettings.h>

#include "nmpc_ocs2_legged_robot/common/ModelSettings.h"
#include "nmpc_ocs2_legged_robot/initialization/LeggedRobotInitializer.h"
#include "nmpc_ocs2_legged_robot/reference_manager/SwitchedModelReferenceManager.h"

namespace ocs2
{
  namespace legged_robot
  {

    class LeggedRobotInterface final : public RobotInterface
    {
    public:
      /**
       * 构造函数：初始化机器人模型、加载参数并构建优化问题
       * @param taskFile: MPC 配置文件路径 (task.info)
       * @param urdfFile: 机器人模型路径 (robot.urdf)
       * @param referenceFile: 步态/参考轨迹文件路径 (reference.info)
       * @param useHardFrictionConeConstraint: 是否使用硬约束形式的摩擦锥
       */
      LeggedRobotInterface(const std::string &taskFile, const std::string &urdfFile, const std::string &referenceFile,
                           bool useHardFrictionConeConstraint = false);

      ~LeggedRobotInterface() override = default;

      // 获取当前构建好的最优控制问题 (OCP)
      const OptimalControlProblem &getOptimalControlProblem() const override { return *problemPtr_; }

      // 各类算法设置的获取接口
      const ModelSettings &modelSettings() const { return modelSettings_; }         // 获取物理模型设置
      const ddp::Settings &ddpSettings() const { return ddpSettings_; }             // 获取 DDP 求解器参数
      const mpc::Settings &mpcSettings() const { return mpcSettings_; }             // 获取 MPC 运行参数
      const rollout::Settings &rolloutSettings() const { return rolloutSettings_; } // 获取正向模拟参数
      const sqp::Settings &sqpSettings() { return sqpSettings_; }                   // 获取 SQP 求解器参数
      const ipm::Settings &ipmSettings() { return ipmSettings_; }                   // 获取 IPM 求解器参数

      // 状态与动力学获取接口
      const vector_t &getInitialState() const { return initialState_; }                                                           // 获取默认初始状态
      const RolloutBase &getRollout() const { return *rolloutPtr_; }                                                              // 获取动力学积分器
      PinocchioInterface &getPinocchioInterface() { return *pinocchioInterfacePtr_; }                                             // 获取 Pinocchio 动力学句柄
      const CentroidalModelInfo &getCentroidalModelInfo() const { return centroidalModelInfo_; }                                  // 获取质心模型参数
      const PinocchioEndEffectorKinematics &getEndEffectorKinematics() const { return *eeKinematicsPtrs_; }                       // 获取足端运动学
      std::shared_ptr<SwitchedModelReferenceManager> getSwitchedModelReferenceManagerPtr() const { return referenceManagerPtr_; } // 获取参考管理器

      // 接口基类要求的重写函数
      const LeggedRobotInitializer &getInitializer() const override { return *initializerPtr_; }                          // 获取初始化策略
      std::shared_ptr<ReferenceManagerInterface> getReferenceManagerPtr() const override { return referenceManagerPtr_; } // 获取通用参考管理器

    private:
      // 设置和构建最优控制问题
      void setupOptimalControlProblem(const std::string &taskFile, const std::string &urdfFile, const std::string &referenceFile, bool verbose);

      // 从 info 文件加载预定义的步态安排
      std::shared_ptr<GaitSchedule> loadGaitSchedule(const std::string &file, bool verbose) const;

      // 创建躯干（Base）的 6-DOF 轨迹跟踪代价函数
      std::unique_ptr<StateInputCost> getBaseTrackingCost(const std::string &taskFile, const CentroidalModelInfo &info, bool verbose);
      // 初始化足端力（Input）的惩罚权重矩阵
      matrix_t initializeInputCostWeight(const std::string &taskFile, const CentroidalModelInfo &info);

      // 加载摩擦锥的相关系数和惩罚参数
      std::pair<scalar_t, RelaxedBarrierPenalty::Config> loadFrictionConeSettings(const std::string &taskFile, bool verbose) const;
      // 创建摩擦锥不等式硬约束
      std::unique_ptr<StateInputConstraint> getFrictionConeConstraint(size_t contactPointIndex, scalar_t frictionCoefficient);
      // 创建摩擦锥软约束（惩罚项）
      std::unique_ptr<StateInputCost> getFrictionConeSoftConstraint(size_t contactPointIndex, scalar_t frictionCoefficient,
                                                                    const RelaxedBarrierPenalty::Config &barrierPenaltyConfig);
      // 创建零力约束（用于摆动腿）
      std::unique_ptr<StateInputConstraint> getZeroForceConstraint(size_t contactPointIndex);
      // 创建足端零速度约束（用于支撑腿，全方位不滑动）
      std::unique_ptr<StateInputConstraint> getZeroVelocityConstraint(const EndEffectorKinematics<scalar_t> &eeKinematics,
                                                                      size_t contactPointIndex, bool useAnalyticalGradients);
      // 创建足端法向速度约束（用于控制抬腿落脚）
      std::unique_ptr<StateInputConstraint> getNormalVelocityConstraint(const EndEffectorKinematics<scalar_t> &eeKinematics,
                                                                        size_t contactPointIndex, bool useAnalyticalGradients);

      // 成员变量
      ModelSettings modelSettings_;              // 物理模型配置
      ddp::Settings ddpSettings_;                // DDP 参数
      mpc::Settings mpcSettings_;                // MPC 参数
      sqp::Settings sqpSettings_;                // SQP 参数
      ipm::Settings ipmSettings_;                // IPM 参数
      const bool useHardFrictionConeConstraint_; // 是否使用硬约束

      std::unique_ptr<PinocchioInterface> pinocchioInterfacePtr_;        // Pinocchio 动力学接口指针
      std::unique_ptr<PinocchioEndEffectorKinematics> eeKinematicsPtrs_; // 足端运动学接口指针
      CentroidalModelInfo centroidalModelInfo_;                          // 质心动力学元数据

      std::unique_ptr<OptimalControlProblem> problemPtr_;                  // 核心 OCP 问题对象
      std::shared_ptr<SwitchedModelReferenceManager> referenceManagerPtr_; // 参考轨迹管理器指针

      rollout::Settings rolloutSettings_;                      // 数值积分参数
      std::unique_ptr<RolloutBase> rolloutPtr_;                // 数值积分执行器
      std::unique_ptr<LeggedRobotInitializer> initializerPtr_; // MPC 初始化器

      vector_t initialState_; // 系统初始状态向量
    };

  } // namespace legged_robot
} // namespace ocs2
  /* CentroidalModelInfo内参数：
   // ------ 模型基础设置 ---
   CentroidalModelType centroidalModelType;
   [模型类型]：FullCentroidalDynamics(全质心动力学) 或 SingleRigidBodyDynamics(SRBD, 单刚体动力学)
  
   // ------ 接触点定义 ---
   size_t numThreeDofContacts = 4;
   [3DOF接触点数]：指只产生线外力(Fx, Fy, Fz)的脚
  
   size_t numSixDofContacts = 0;
   [6DOF接触点数]：指能产生线外力+转动力矩的脚（如人脚）
  
   std::vector<size_t> endEffectorFrameIndices;
   [末端序号]：四个脚在 Pinocchio 动力学模型里的 Frame 编号，用于索引位置和雅可比
  
   // ------ 维度定义 (决定矩阵大小) ---
   size_t generalizedCoordinatesNum;
   [广义坐标数]：身体的6维位姿 + 12个关节 = 18
  
   size_t actuatedDofNum = 12;
   [驱动自由度数]：电机的个数
  
   size_t stateDim = 24;
   [状态维度]：OCS2算法里的状态向量长度，通常包含身体动量、姿态和关节角度等信息。
  
   size_t inputDim = 24;
   [输入维度]：控制算法输出的变量长度，通常包含12维足端力和12维关节速度等信息。
  
   // ------ 物理参数 ---
   scalar_t robotMass;
   [总质量]：机器人的整机重量 (kg)
  
   // ------ SRBD 模型专用标称值 (用于预测) ---
   vector_t qPinocchioNominal;
   [标称关节角度]：机器人“标准站姿”时的关节配置
  
   matrix3_t centroidalInertiaNominal;
   [标称转动惯量]：在标准站姿下，身体相对于质心的 3x3 惯性矩阵
  
   vector3_t comToBasePositionNominal;
   [重心偏移量]：质心(CoM)相对于机器人几何中心(Base)的 3D 偏移
   */