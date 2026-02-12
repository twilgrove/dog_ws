//
// Created by qiayuan on 2022/7/1.
//

#pragma once

#include "legged_wbc/Task.h"

#include <ocs2_centroidal_model/PinocchioCentroidalDynamics.h>
#include <ocs2_legged_robot/gait/MotionPhaseDefinition.h>
#include <ocs2_pinocchio_interface/PinocchioEndEffectorKinematics.h>

namespace legged {
using namespace ocs2;
using namespace legged_robot;

// 决策变量: x = [\dot u^T, F^T, \tau^T]^T
// 其中: \dot u = 基座加速度, F = 接触力, \tau = 关节力矩
class WbcBase {
  using Vector6 = Eigen::Matrix<scalar_t, 6, 1>;
  using Matrix6 = Eigen::Matrix<scalar_t, 6, 6>;

 public:
  // 构造函数: 初始化WBC基类
  // pinocchioInterface: Pinocchio机器人模型接口
  // info: 质心动力学模型信息
  // eeKinematics: 末端执行器运动学
  WbcBase(const PinocchioInterface& pinocchioInterface, CentroidalModelInfo info, const PinocchioEndEffectorKinematics& eeKinematics);

  // 从配置文件加载任务参数
  // taskFile: 任务配置文件路径
  // verbose: 是否输出详细加载信息
  virtual void loadTasksSetting(const std::string& taskFile, bool verbose);

  // 主更新函数: 计算WBC控制输出
  // stateDesired: 期望状态
  // inputDesired: 期望输入
  // rbdStateMeasured: 测量的刚体状态
  // mode: 当前运动模式
  // period: 控制周期
  // 返回: 优化后的控制输入
  virtual vector_t update(const vector_t& stateDesired, const vector_t& inputDesired, const vector_t& rbdStateMeasured, size_t mode,
                          scalar_t period);

 protected:
  // 更新测量状态
  // rbdStateMeasured: 测量的刚体状态
  void updateMeasured(const vector_t& rbdStateMeasured);
  
  // 更新期望状态
  // stateDesired: 期望状态
  // inputDesired: 期望输入
  void updateDesired(const vector_t& stateDesired, const vector_t& inputDesired);

  // 获取决策变量数量
  size_t getNumDecisionVars() const { return numDecisionVars_; }

  // 构建浮动基座动力学方程任务
  Task formulateFloatingBaseEomTask();
  
  // 构建关节力矩限制任务
  Task formulateTorqueLimitsTask();
  
  // 构建无接触运动任务
  Task formulateNoContactMotionTask();
  
  // 构建摩擦锥约束任务
  Task formulateFrictionConeTask();
  
  // 构建基座加速度跟踪任务
  // stateDesired: 期望状态
  // inputDesired: 期望输入
  // period: 控制周期
  Task formulateBaseAccelTask(const vector_t& stateDesired, const vector_t& inputDesired, scalar_t period);
  
  // 构建摆动腿轨迹跟踪任务
  Task formulateSwingLegTask();
  
  // 构建接触力跟踪任务
  // inputDesired: 期望输入
  Task formulateContactForceTask(const vector_t& inputDesired) const;

  // 决策变量数量
  size_t numDecisionVars_;
  // 测量和期望状态对应的Pinocchio接口
  PinocchioInterface pinocchioInterfaceMeasured_, pinocchioInterfaceDesired_;
  // 质心动力学模型信息
  CentroidalModelInfo info_;

  // 末端执行器运动学计算器
  std::unique_ptr<PinocchioEndEffectorKinematics> eeKinematics_;
  // 质心模型到Pinocchio的映射
  CentroidalModelPinocchioMapping mapping_;

  // 测量状态: 关节位置、速度
  vector_t qMeasured_, vMeasured_;
  // 上一次的控制输入
  vector_t inputLast_;
  // 雅可比矩阵及其导数
  matrix_t j_, dj_;
  // 接触标志位
  contact_flag_t contactFlag_{};
  // 接触点数量
  size_t numContacts_{};

  // 任务参数:
  vector_t torqueLimits_;      // 关节力矩限制
  scalar_t frictionCoeff_{};   // 摩擦系数
  scalar_t swingKp_{};         // 摆动腿位置控制P增益
  scalar_t swingKd_{};         // 摆动腿速度控制D增益
};

}  // namespace legged