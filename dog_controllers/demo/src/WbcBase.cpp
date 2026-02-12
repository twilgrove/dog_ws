//
// Created by qiayuan on 2022/7/1.
//
#include <pinocchio/fwd.hpp> // 前向声明必须首先包含

#include "legged_wbc/WbcBase.h"

#include <ocs2_centroidal_model/AccessHelperFunctions.h>
#include <ocs2_centroidal_model/ModelHelperFunctions.h>
#include <pinocchio/algorithm/centroidal.hpp>
#include <pinocchio/algorithm/crba.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <utility>

namespace legged
{
  // 构造函数：初始化WBC基类，设置Pinocchio接口、质心模型信息、末端执行器运动学
  WbcBase::WbcBase(const PinocchioInterface &pinocchioInterface, CentroidalModelInfo info, const PinocchioEndEffectorKinematics &eeKinematics)
      : pinocchioInterfaceMeasured_(pinocchioInterface), // 测量用的Pinocchio接口
        pinocchioInterfaceDesired_(pinocchioInterface),  // 期望用的Pinocchio接口
        info_(std::move(info)),                          // 质心模型信息
        mapping_(info_),                                 // 状态映射
        inputLast_(vector_t::Zero(info_.inputDim)),      // 上一时刻输入初始化为零
        eeKinematics_(eeKinematics.clone())
  { // 末端执行器运动学克隆
    // 决策变量数量 = 广义坐标数 + 3*接触点数 + 驱动关节数
    numDecisionVars_ = info_.generalizedCoordinatesNum + 3 * info_.numThreeDofContacts + info_.actuatedDofNum;
    // 初始化测量位置和速度向量
    qMeasured_ = vector_t(info_.generalizedCoordinatesNum);
    vMeasured_ = vector_t(info_.generalizedCoordinatesNum);
  }

  // 更新函数：根据期望状态、输入和测量状态更新WBC，返回空向量（需子类实现具体WBC求解）
  vector_t WbcBase::update(const vector_t &stateDesired, const vector_t &inputDesired, const vector_t &rbdStateMeasured, size_t mode,
                           scalar_t /*period*/)
  {
    // 根据模式计算接触标志
    contactFlag_ = modeNumber2StanceLeg(mode);
    numContacts_ = 0;
    for (bool flag : contactFlag_)
    {
      if (flag)
      {
        numContacts_++; // 统计接触点数量
      }
    }

    // 更新测量和期望相关量
    updateMeasured(rbdStateMeasured);
    updateDesired(stateDesired, inputDesired);

    return {}; // 基类返回空，子类应重写以返回实际控制量
  }

  // 更新测量状态：从测量状态提取位置和速度，并计算运动学量
  void WbcBase::updateMeasured(const vector_t &rbdStateMeasured)
  {
    // 位置：前3个为平移，接着3个为欧拉角，最后为关节角
    qMeasured_.head<3>() = rbdStateMeasured.segment<3>(3);
    qMeasured_.segment<3>(3) = rbdStateMeasured.head<3>();
    qMeasured_.tail(info_.actuatedDofNum) = rbdStateMeasured.segment(6, info_.actuatedDofNum);
    // 速度：前3个为线速度，接着3个为角速度（从全局角速度转换），最后为关节速度
    vMeasured_.head<3>() = rbdStateMeasured.segment<3>(info_.generalizedCoordinatesNum + 3);
    vMeasured_.segment<3>(3) = getEulerAnglesZyxDerivativesFromGlobalAngularVelocity<scalar_t>(
        qMeasured_.segment<3>(3), rbdStateMeasured.segment<3>(info_.generalizedCoordinatesNum));
    vMeasured_.tail(info_.actuatedDofNum) = rbdStateMeasured.segment(info_.generalizedCoordinatesNum + 6, info_.actuatedDofNum);

    const auto &model = pinocchioInterfaceMeasured_.getModel();
    auto &data = pinocchioInterfaceMeasured_.getData();

    // 为浮基动力学任务准备：前向运动学、雅可比、质量矩阵、非线性项
    pinocchio::forwardKinematics(model, data, qMeasured_, vMeasured_);
    pinocchio::computeJointJacobians(model, data);
    pinocchio::updateFramePlacements(model, data);
    pinocchio::crba(model, data, qMeasured_);                                                                  // 计算复合刚体算法（质量矩阵）
    data.M.triangularView<Eigen::StrictlyLower>() = data.M.transpose().triangularView<Eigen::StrictlyLower>(); // 对称化质量矩阵
    pinocchio::nonLinearEffects(model, data, qMeasured_, vMeasured_);                                          // 计算非线性项（科里奥利+重力）
    // 计算接触点雅可比矩阵（位置部分）
    j_ = matrix_t(3 * info_.numThreeDofContacts, info_.generalizedCoordinatesNum);
    for (size_t i = 0; i < info_.numThreeDofContacts; ++i)
    {
      Eigen::Matrix<scalar_t, 6, Eigen::Dynamic> jac;
      jac.setZero(6, info_.generalizedCoordinatesNum);
      pinocchio::getFrameJacobian(model, data, info_.endEffectorFrameIndices[i], pinocchio::LOCAL_WORLD_ALIGNED, jac);
      j_.block(3 * i, 0, 3, info_.generalizedCoordinatesNum) = jac.template topRows<3>(); // 取位置雅可比
    }

    // 为无接触运动任务准备：计算雅可比时间导数
    pinocchio::computeJointJacobiansTimeVariation(model, data, qMeasured_, vMeasured_);
    dj_ = matrix_t(3 * info_.numThreeDofContacts, info_.generalizedCoordinatesNum);
    for (size_t i = 0; i < info_.numThreeDofContacts; ++i)
    {
      Eigen::Matrix<scalar_t, 6, Eigen::Dynamic> jac;
      jac.setZero(6, info_.generalizedCoordinatesNum);
      pinocchio::getFrameJacobianTimeVariation(model, data, info_.endEffectorFrameIndices[i], pinocchio::LOCAL_WORLD_ALIGNED, jac);
      dj_.block(3 * i, 0, 3, info_.generalizedCoordinatesNum) = jac.template topRows<3>(); // 取位置雅可比时间导数
    }
  }

  // 更新期望状态：计算期望运动学量
  void WbcBase::updateDesired(const vector_t &stateDesired, const vector_t &inputDesired)
  {
    const auto &model = pinocchioInterfaceDesired_.getModel();
    auto &data = pinocchioInterfaceDesired_.getData();

    // 设置映射并获取期望关节位置和速度
    mapping_.setPinocchioInterface(pinocchioInterfaceDesired_);
    const auto qDesired = mapping_.getPinocchioJointPosition(stateDesired);
    pinocchio::forwardKinematics(model, data, qDesired);
    pinocchio::computeJointJacobians(model, data, qDesired);
    pinocchio::updateFramePlacements(model, data);
    updateCentroidalDynamics(pinocchioInterfaceDesired_, info_, qDesired); // 更新质心动力学
    const vector_t vDesired = mapping_.getPinocchioJointVelocity(stateDesired, inputDesired);
    pinocchio::forwardKinematics(model, data, qDesired, vDesired); // 带速度的前向运动学
  }

  //约束
  // 构建浮基动力学方程任务：M * accel - J^T * force - S^T * torque = -nle
  Task WbcBase::formulateFloatingBaseEomTask()
  {
    auto &data = pinocchioInterfaceMeasured_.getData();

    // 选择矩阵S：前6个为浮基，后为驱动关节
    matrix_t s(info_.actuatedDofNum, info_.generalizedCoordinatesNum);
    s.block(0, 0, info_.actuatedDofNum, 6).setZero();
    s.block(0, 6, info_.actuatedDofNum, info_.actuatedDofNum).setIdentity();

    // 构建任务矩阵A和向量b：A = [M, -J^T, -S^T], b = -nle
    matrix_t a = (matrix_t(info_.generalizedCoordinatesNum, numDecisionVars_) << data.M, -j_.transpose(), -s.transpose()).finished();
    vector_t b = -data.nle;

    return {a, b, matrix_t(), vector_t()}; // 返回等式任务（无不等式约束）
  }

  // 构建力矩限制任务：-limit <= torque <= limit
  Task WbcBase::formulateTorqueLimitsTask()
  {
    matrix_t d(2 * info_.actuatedDofNum, numDecisionVars_);
    d.setZero();
    matrix_t i = matrix_t::Identity(info_.actuatedDofNum, info_.actuatedDofNum);
    // 设置不等式矩阵D：正负单位矩阵对应力矩上下限
    d.block(0, info_.generalizedCoordinatesNum + 3 * info_.numThreeDofContacts, info_.actuatedDofNum, info_.actuatedDofNum) = i;
    d.block(info_.actuatedDofNum, info_.generalizedCoordinatesNum + 3 * info_.numThreeDofContacts, info_.actuatedDofNum,
            info_.actuatedDofNum) = -i;
    vector_t f(2 * info_.actuatedDofNum);
    // 设置力矩限制向量：每个关节3个自由度重复限制值
    for (size_t l = 0; l < 2 * info_.actuatedDofNum / 3; ++l)
    {
      f.segment<3>(3 * l) = torqueLimits_;
    }

    return {matrix_t(), vector_t(), d, f}; // 返回不等式任务（无等式约束）
  }

  // 构建无接触运动任务：接触点加速度为零（保持接触）
  Task WbcBase::formulateNoContactMotionTask()
  {
    matrix_t a(3 * numContacts_, numDecisionVars_);
    vector_t b(a.rows());
    a.setZero();
    b.setZero();
    size_t j = 0;
    for (size_t i = 0; i < info_.numThreeDofContacts; i++)
    {
      if (contactFlag_[i])
      { // 仅对接触点
        // 等式：J * accel + dJ * vel = 0
        a.block(3 * j, 0, 3, info_.generalizedCoordinatesNum) = j_.block(3 * i, 0, 3, info_.generalizedCoordinatesNum);
        b.segment(3 * j, 3) = -dj_.block(3 * i, 0, 3, info_.generalizedCoordinatesNum) * vMeasured_;
        j++;
      }
    }

    return {a, b, matrix_t(), vector_t()}; // 返回等式任务
  }

  // 构建摩擦锥任务：接触力在摩擦锥内，非接触点力为零
  Task WbcBase::formulateFrictionConeTask()
  {
    // 等式部分：非接触点力为零
    matrix_t a(3 * (info_.numThreeDofContacts - numContacts_), numDecisionVars_);
    a.setZero();
    size_t j = 0;
    for (size_t i = 0; i < info_.numThreeDofContacts; ++i)
    {
      if (!contactFlag_[i])
      {
        a.block(3 * j++, info_.generalizedCoordinatesNum + 3 * i, 3, 3) = matrix_t::Identity(3, 3); // 力变量直接设为单位
      }
    }
    vector_t b(a.rows());
    b.setZero();

    // 摩擦锥不等式：5个平面约束（法向+4个切向）
    matrix_t frictionPyramic(5, 3); // clang-format off
  frictionPyramic << 0, 0, -1,     // 法向力向下为正
                     1, 0, -frictionCoeff_,   // x方向摩擦
                    -1, 0, -frictionCoeff_,   // -x方向摩擦
                     0, 1, -frictionCoeff_,   // y方向摩擦
                     0,-1, -frictionCoeff_;   // -y方向摩擦
    // clang-format on

    matrix_t d(5 * numContacts_ + 3 * (info_.numThreeDofContacts - numContacts_), numDecisionVars_);
    d.setZero();
    j = 0;
    // 对接触点施加摩擦锥约束
    for (size_t i = 0; i < info_.numThreeDofContacts; ++i)
    {
      if (contactFlag_[i])
      {
        d.block(5 * j++, info_.generalizedCoordinatesNum + 3 * i, 5, 3) = frictionPyramic;
      }
    }
    vector_t f = Eigen::VectorXd::Zero(d.rows()); // 不等式上界为零（锥内）

    return {a, b, d, f}; // 返回混合任务
  }

  //加权任务
  // 构建基座加速度任务：跟踪期望的基座加速度
  Task WbcBase::formulateBaseAccelTask(const vector_t &stateDesired, const vector_t &inputDesired, scalar_t period)
  {
    matrix_t a(6, numDecisionVars_);
    a.setZero();
    a.block(0, 0, 6, 6) = matrix_t::Identity(6, 6); // 直接作用于基座加速度变量

    // 计算关节加速度：通过输入差分估计
    vector_t jointAccel = centroidal_model::getJointVelocities(inputDesired - inputLast_, info_) / period;
    inputLast_ = inputDesired;
    mapping_.setPinocchioInterface(pinocchioInterfaceDesired_);

    const auto &model = pinocchioInterfaceDesired_.getModel();
    auto &data = pinocchioInterfaceDesired_.getData();
    const auto qDesired = mapping_.getPinocchioJointPosition(stateDesired);
    const vector_t vDesired = mapping_.getPinocchioJointVelocity(stateDesired, inputDesired);

    // 质心动力学计算：A_b * accel_b + A_j * accel_j + A_dot * v = 质心动量变化率
    const auto &A = getCentroidalMomentumMatrix(pinocchioInterfaceDesired_);
    const Matrix6 Ab = A.template leftCols<6>();                               // 基座部分
    const auto AbInv = computeFloatingBaseCentroidalMomentumMatrixInverse(Ab); // 基座逆
    const auto Aj = A.rightCols(info_.actuatedDofNum);                         // 关节部分
    const auto ADot = pinocchio::dccrba(model, data, qDesired, vDesired);      // A的时间导数
    Vector6 centroidalMomentumRate = info_.robotMass * getNormalizedCentroidalMomentumRate(pinocchioInterfaceDesired_, info_, inputDesired);
    centroidalMomentumRate.noalias() -= ADot * vDesired; // 减去速度相关项
    centroidalMomentumRate.noalias() -= Aj * jointAccel; // 减去关节加速度项

    // 计算期望基座加速度
    Vector6 b = AbInv * centroidalMomentumRate;

    return {a, b, matrix_t(), vector_t()}; // 返回等式任务
  }

  // 构建摆动腿任务：PD控制跟踪期望末端轨迹
  Task WbcBase::formulateSwingLegTask()
  {
    // 测量末端位置和速度
    eeKinematics_->setPinocchioInterface(pinocchioInterfaceMeasured_);
    std::vector<vector3_t> posMeasured = eeKinematics_->getPosition(vector_t());
    std::vector<vector3_t> velMeasured = eeKinematics_->getVelocity(vector_t(), vector_t());
    // 期望末端位置和速度
    eeKinematics_->setPinocchioInterface(pinocchioInterfaceDesired_);
    std::vector<vector3_t> posDesired = eeKinematics_->getPosition(vector_t());
    std::vector<vector3_t> velDesired = eeKinematics_->getVelocity(vector_t(), vector_t());

    matrix_t a(3 * (info_.numThreeDofContacts - numContacts_), numDecisionVars_);
    vector_t b(a.rows());
    a.setZero();
    b.setZero();
    size_t j = 0;
    for (size_t i = 0; i < info_.numThreeDofContacts; ++i)
    {
      if (!contactFlag_[i])
      { // 仅对摆动腿
        // PD加速度：accel = Kp*(pos_d - pos_m) + Kd*(vel_d - vel_m)
        vector3_t accel = swingKp_ * (posDesired[i] - posMeasured[i]) + swingKd_ * (velDesired[i] - velMeasured[i]);
        // 任务：J * accel + dJ * vel = accel_desired
        a.block(3 * j, 0, 3, info_.generalizedCoordinatesNum) = j_.block(3 * i, 0, 3, info_.generalizedCoordinatesNum);
        b.segment(3 * j, 3) = accel - dj_.block(3 * i, 0, 3, info_.generalizedCoordinatesNum) * vMeasured_;
        j++;
      }
    }

    return {a, b, matrix_t(), vector_t()}; // 返回等式任务
  }

  // 构建接触力任务：跟踪期望接触力
Task WbcBase::formulateContactForceTask(const vector_t& inputDesired