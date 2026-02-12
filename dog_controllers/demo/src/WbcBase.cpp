//
// Created by qiayuan on 2022/7/1.
//
#include <pinocchio/fwd.hpp> // 必须首先包含前向声明

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
  /**
   * WBC 基类构造函数
   * @param pinocchioInterface Pinocchio 模型接口
   * @param info 质心模型信息（包含机器人质量、自由度等）
   * @param eeKinematics 末端执行器（足端）运动学接口
   */
  WbcBase::WbcBase(const PinocchioInterface &pinocchioInterface, CentroidalModelInfo info, const PinocchioEndEffectorKinematics &eeKinematics)
      : pinocchioInterfaceMeasured_(pinocchioInterface), // 观测模型接口（用于实测状态）
        pinocchioInterfaceDesired_(pinocchioInterface),  // 期望模型接口（用于 NMPC 目标）
        info_(std::move(info)),
        mapping_(info_), // 状态/输入映射工具
        inputLast_(vector_t::Zero(info_.inputDim)),
        eeKinematics_(eeKinematics.clone())
  {
    // 定义 QP 决策变量的数量：广义坐标(q_acc) + 足端接触力(3*4) + 关节力矩(12)
    numDecisionVars_ = info_.generalizedCoordinatesNum + 3 * info_.numThreeDofContacts + info_.actuatedDofNum;
    qMeasured_ = vector_t(info_.generalizedCoordinatesNum);
    vMeasured_ = vector_t(info_.generalizedCoordinatesNum);
  }

  /**
   * WBC 更新主循环
   * @param stateDesired NMPC 输出的期望状态
   * @param inputDesired NMPC 输出的期望输入（主要包含期望足端力）
   * @param rbdStateMeasured 状态估计器输出的实测状态
   * @param mode 当前步态模式（支撑情况）
   */
  vector_t WbcBase::update(const vector_t &stateDesired, const vector_t &inputDesired, const vector_t &rbdStateMeasured, size_t mode,
                           scalar_t /*period*/)
  {
    // 根据当前模式确定哪些腿是支撑腿
    contactFlag_ = modeNumber2StanceLeg(mode);
    numContacts_ = 0;
    for (bool flag : contactFlag_)
    {
      if (flag)
      {
        numContacts_++;
      }
    }

    // 更新实测数据的动力学（Jacobian, Mass Matrix 等）
    updateMeasured(rbdStateMeasured);
    // 更新期望数据的动力学
    updateDesired(stateDesired, inputDesired);

    return {}; // 基类返回空，具体求解在子类 HoPmcWbc 中实现
  }

  /**
   * 更新实测状态相关的 Pinocchio 数据
   */
  void WbcBase::updateMeasured(const vector_t &rbdStateMeasured)
  {
    // 将状态估计器的输出转换为 Pinocchio 的广义坐标 q 和速度 v
    qMeasured_.head<3>() = rbdStateMeasured.segment<3>(3);                                     // 位置
    qMeasured_.segment<3>(3) = rbdStateMeasured.head<3>();                                     // 欧拉角
    qMeasured_.tail(info_.actuatedDofNum) = rbdStateMeasured.segment(6, info_.actuatedDofNum); // 关节角

    vMeasured_.head<3>() = rbdStateMeasured.segment<3>(info_.generalizedCoordinatesNum + 3); // 线速度
    vMeasured_.segment<3>(3) = getEulerAnglesZyxDerivativesFromGlobalAngularVelocity<scalar_t>(
        qMeasured_.segment<3>(3), rbdStateMeasured.segment<3>(info_.generalizedCoordinatesNum));                                 // 角速度转欧拉角变化率
    vMeasured_.tail(info_.actuatedDofNum) = rbdStateMeasured.segment(info_.generalizedCoordinatesNum + 6, info_.actuatedDofNum); // 关节速度

    const auto &model = pinocchioInterfaceMeasured_.getModel();
    auto &data = pinocchioInterfaceMeasured_.getData();

    // 执行前向运动学计算
    pinocchio::forwardKinematics(model, data, qMeasured_, vMeasured_);
    pinocchio::computeJointJacobians(model, data);
    pinocchio::updateFramePlacements(model, data);
    // 计算质量矩阵 M (Composite Rigid Body Algorithm)
    pinocchio::crba(model, data, qMeasured_);
    data.M.triangularView<Eigen::StrictlyLower>() = data.M.transpose().triangularView<Eigen::StrictlyLower>();
    // 计算非线性效应 nle (Coriolis + Gravity)
    pinocchio::nonLinearEffects(model, data, qMeasured_, vMeasured_);

    // 计算足端雅可比矩阵 J
    j_ = matrix_t(3 * info_.numThreeDofContacts, info_.generalizedCoordinatesNum);
    for (size_t i = 0; i < info_.numThreeDofContacts; ++i)
    {
      Eigen::Matrix<scalar_t, 6, Eigen::Dynamic> jac;
      jac.setZero(6, info_.generalizedCoordinatesNum);
      pinocchio::getFrameJacobian(model, data, info_.endEffectorFrameIndices[i], pinocchio::LOCAL_WORLD_ALIGNED, jac);
      j_.block(3 * i, 0, 3, info_.generalizedCoordinatesNum) = jac.template topRows<3>();
    }

    // 计算雅可比矩阵的时间导数 dJ
    pinocchio::computeJointJacobiansTimeVariation(model, data, qMeasured_, vMeasured_);
    dj_ = matrix_t(3 * info_.numThreeDofContacts, info_.generalizedCoordinatesNum);
    for (size_t i = 0; i < info_.numThreeDofContacts; ++i)
    {
      Eigen::Matrix<scalar_t, 6, Eigen::Dynamic> jac;
      jac.setZero(6, info_.generalizedCoordinatesNum);
      pinocchio::getFrameJacobianTimeVariation(model, data, info_.endEffectorFrameIndices[i], pinocchio::LOCAL_WORLD_ALIGNED, jac);
      dj_.block(3 * i, 0, 3, info_.generalizedCoordinatesNum) = jac.template topRows<3>();
    }
  }

  /**
   * 构造浮基座运动学方程 (Floating Base Equation of Motion)
   * M*q_acc + h = J^T * f + S^T * tau
   */
  Task WbcBase::formulateFloatingBaseEomTask()
  {
    auto &data = pinocchioInterfaceMeasured_.getData();

    // 这里的 S 是选择矩阵，用于提取被驱动的关节（非浮基座部分）
    matrix_t s(info_.actuatedDofNum, info_.generalizedCoordinatesNum);
    s.block(0, 0, info_.actuatedDofNum, 6).setZero();
    s.block(0, 6, info_.actuatedDofNum, info_.actuatedDofNum).setIdentity();

    // 方程变为: M*q_acc - J^T*f - S^T*tau = -nle
    matrix_t a = (matrix_t(info_.generalizedCoordinatesNum, numDecisionVars_) << data.M, -j_.transpose(), -s.transpose()).finished();
    vector_t b = -data.nle;

    return {a, b, matrix_t(), vector_t()};
  }

  /**
   * 构造关节力矩限制任务 (Torque Limits)
   */
  Task WbcBase::formulateTorqueLimitsTask()
  {
    matrix_t d(2 * info_.actuatedDofNum, numDecisionVars_);
    d.setZero();
    matrix_t i = matrix_t::Identity(info_.actuatedDofNum, info_.actuatedDofNum);
    // 提取决策变量中的力矩部分 tau
    d.block(0, info_.generalizedCoordinatesNum + 3 * info_.numThreeDofContacts, info_.actuatedDofNum, info_.actuatedDofNum) = i;
    d.block(info_.actuatedDofNum, info_.generalizedCoordinatesNum + 3 * info_.numThreeDofContacts, info_.actuatedDofNum,
            info_.actuatedDofNum) = -i;
    vector_t f(2 * info_.actuatedDofNum);
    for (size_t l = 0; l < 2 * info_.actuatedDofNum / 3; ++l)
    {
      f.segment<3>(3 * l) = torqueLimits_; // 设置上下限
    }

    return {matrix_t(), vector_t(), d, f};
  }

  /**
   * 构造支撑腿不动的任务 (No Contact Motion)
   * 约束：J*q_acc + dJ*q_vel = 0 (即足端加速度为0)
   */
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
      {
        a.block(3 * j, 0, 3, info_.generalizedCoordinatesNum) = j_.block(3 * i, 0, 3, info_.generalizedCoordinatesNum);
        b.segment(3 * j, 3) = -dj_.block(3 * i, 0, 3, info_.generalizedCoordinatesNum) * vMeasured_;
        j++;
      }
    }

    return {a, b, matrix_t(), vector_t()};
  }

  /**
   * 构造摩擦锥约束任务 (Friction Cone)
   * 摆动腿：接触力必须为 0；支撑腿：接触力必须在摩擦金字塔内
   */
  Task WbcBase::formulateFrictionConeTask()
  {
    // 摆动腿约束：Force = 0
    matrix_t a(3 * (info_.numThreeDofContacts - numContacts_), numDecisionVars_);
    a.setZero();
    size_t j = 0;
    for (size_t i = 0; i < info_.numThreeDofContacts; ++i)
    {
      if (!contactFlag_[i])
      {
        a.block(3 * j++, info_.generalizedCoordinatesNum + 3 * i, 3, 3) = matrix_t::Identity(3, 3);
      }
    }
    vector_t b(a.rows());
    b.setZero();

    // 支撑腿摩擦金字塔约束 (近似摩擦锥)
    matrix_t frictionPyramic(5, 3);
    frictionPyramic << 0, 0, -1, // Fz > 0 (不能吸在地上)
        1, 0, -frictionCoeff_,   // |Fx| < mu*Fz
        -1, 0, -frictionCoeff_,
        0, 1, -frictionCoeff_, // |Fy| < mu*Fz
        0, -1, -frictionCoeff_;

    matrix_t d(5 * numContacts_ + 3 * (info_.numThreeDofContacts - numContacts_), numDecisionVars_);
    d.setZero();
    j = 0;
    for (size_t i = 0; i < info_.numThreeDofContacts; ++i)
    {
      if (contactFlag_[i])
      {
        d.block(5 * j++, info_.generalizedCoordinatesNum + 3 * i, 5, 3) = frictionPyramic;
      }
    }
    vector_t f = Eigen::VectorXd::Zero(d.rows());

    return {a, b, d, f};
  }

  /**
   * 构造机身加速度任务 (Base Acceleration)
   * 根据质心动力学，将 NMPC 期望的质心动量变化率转化为机身加速度
   */
  Task WbcBase::formulateBaseAccelTask(const vector_t &stateDesired, const vector_t &inputDesired, scalar_t period)
  {
    matrix_t a(6, numDecisionVars_);
    a.setZero();
    // 提取决策变量中的前 6 维 (浮基座加速度)
    a.block(0, 0, 6, 6) = matrix_t::Identity(6, 6);

    // 计算期望的关节加速度
    vector_t jointAccel = centroidal_model::getJointVelocities(inputDesired - inputLast_, info_) / period;
    inputLast_ = inputDesired;

    // 基于质心动量矩阵 (CMM) 计算机身加速度目标
    const auto &A = getCentroidalMomentumMatrix(pinocchioInterfaceDesired_);
    const Matrix6 Ab = A.template leftCols<6>();
    const auto AbInv = computeFloatingBaseCentroidalMomentumMatrixInverse(Ab);
    const auto Aj = A.rightCols(info_.actuatedDofNum);

    const auto &model = pinocchioInterfaceDesired_.getModel();
    auto &data = pinocchioInterfaceDesired_.getData();
    const auto qDesired = mapping_.getPinocchioJointPosition(stateDesired);
    const vector_t vDesired = mapping_.getPinocchioJointVelocity(stateDesired, inputDesired);
    const auto ADot = pinocchio::dccrba(model, data, qDesired, vDesired);

    // 质心动量变化率 = Ab*v_base_acc + Aj*v_joint_acc + ADot*v_all
    Vector6 centroidalMomentumRate = info_.robotMass * getNormalizedCentroidalMomentumRate(pinocchioInterfaceDesired_, info_, inputDesired);
    centroidalMomentumRate.noalias() -= ADot * vDesired;
    centroidalMomentumRate.noalias() -= Aj * jointAccel;

    Vector6 b = AbInv * centroidalMomentumRate;

    return {a, b, matrix_t(), vector_t()};
  }

  /**
   * 构造摆动腿任务 (Swing Leg Tracking)
   * 使用 PD 控制器跟踪 NMPC 给出的足端轨迹位置和速度
   */
  Task WbcBase::formulateSwingLegTask()
  {
    eeKinematics_->setPinocchioInterface(pinocchioInterfaceMeasured_);
    std::vector<vector3_t> posMeasured = eeKinematics_->getPosition(vector_t());
    std::vector<vector3_t> velMeasured = eeKinematics_->getVelocity(vector_t(), vector_t());

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
      {
        // 计算目标加速度: b = Kp*(pos_err) + Kd*(vel_err) - dJ*v
        vector3_t accel = swingKp_ * (posDesired[i] - posMeasured[i]) + swingKd_ * (velDesired[i] - velMeasured[i]);
        a.block(3 * j, 0, 3, info_.generalizedCoordinatesNum) = j_.block(3 * i, 0, 3, info_.generalizedCoordinatesNum);
        b.segment(3 * j, 3) = accel - dj_.block(3 * i, 0, 3, info_.generalizedCoordinatesNum) * vMeasured_;
        j++;
      }
    }

    return {a, b, matrix_t(), vector_t()};
  }

  /**
   * 构造接触力任务 (Contact Force)
   * 让 WBC 计算出的力尽量贴合 NMPC 优化出来的期望力 (Feedforward)
   */
  Task WbcBase::formulateContactForceTask(const vector_t &inputDesired) const
  {
    matrix_t a(3 * info_.numThreeDofContacts, numDecisionVars_);
    vector_t b(a.rows());
    a.setZero();

    for (size_t i = 0; i < info_.numThreeDofContacts; ++i)
    {
      a.block(3 * i, info_.generalizedCoordinatesNum + 3 * i, 3, 3) = matrix_t::Identity(3, 3);
    }
    // NMPC 的 inputDesired 前部分通常就是期望接触力
    b = inputDesired.head(a.rows());

    return {a, b, matrix_t(), vector_t()};
  }