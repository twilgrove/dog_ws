#include "wbc/WbcBase.hpp"
#include <pinocchio/fwd.hpp>

#include <ocs2_centroidal_model/AccessHelperFunctions.h>
#include <ocs2_centroidal_model/ModelHelperFunctions.h>
#include <pinocchio/algorithm/centroidal.hpp>
#include <pinocchio/algorithm/crba.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <utility>
#include <ocs2_legged_robot/gait/MotionPhaseDefinition.h>
namespace dog_controllers
{
    WbcBase::WbcBase(
        const PinocchioInterface &pinocchioInterface,
        const CentroidalModelInfo &info,
        const PinocchioEndEffectorKinematics &eeKinematics,
        rclcpp_lifecycle::LifecycleNode::SharedPtr &node)
        : info_(info),
          mapping_(info_),
          pinocchioInterfaceMeasured_(pinocchioInterface),
          pinocchioInterfaceDesired_(pinocchioInterface),
          eeKinematics_(eeKinematics.clone()),
          inputLast_(vector_t::Zero(info_.inputDim)),
          node_(node)

    {
        // 决策变量数量 = 18 + 3*4 + 12 = 42
        numDecisionVars_ = info_.generalizedCoordinatesNum + 3 * info_.numThreeDofContacts + info_.actuatedDofNum;
        // 初始化测量位置和速度向量
        qMeasured_ = vector_t(info_.generalizedCoordinatesNum);
        vMeasured_ = vector_t(info_.generalizedCoordinatesNum);
    }
    vector_t WbcBase::update(const vector_t &stateDesired,
                             const vector_t &inputDesired,
                             const vector_t &rbdStateMeasured,
                             size_t mode,
                             scalar_t /*period*/)
    {
        // 更新接触状态
        contactFlag_ = modeNumber2StanceLeg(mode);

        numContacts_ = std::count(contactFlag_.begin(), contactFlag_.end(), true);

        // 更新实测数据的动力学
        updateMeasured(rbdStateMeasured);
        // 更新期望数据的动力学
        // updateDesired(stateDesired, inputDesired);

        return {}; // 基类返回空，具体求解在子类 HoPmcWbc 中实现
    }

    /**
     * 更新实测状态相关的 Pinocchio 数据
     */
    void WbcBase::updateMeasured(const vector_t &rbdStateMeasured)
    {
        qMeasured_.head<3>() = rbdStateMeasured.segment<3>(3);
        qMeasured_.segment<3>(3) = rbdStateMeasured.head<3>();
        qMeasured_.tail(info_.actuatedDofNum) = rbdStateMeasured.segment(6, info_.actuatedDofNum);

        vMeasured_.head<3>() = rbdStateMeasured.segment<3>(info_.generalizedCoordinatesNum + 3);
        vMeasured_.segment<3>(3) = getEulerAnglesZyxDerivativesFromGlobalAngularVelocity<scalar_t>(
            qMeasured_.segment<3>(3),
            rbdStateMeasured.segment<3>(info_.generalizedCoordinatesNum));
        vMeasured_.tail(info_.actuatedDofNum) = rbdStateMeasured.segment(info_.generalizedCoordinatesNum + 6, info_.actuatedDofNum);

        const auto &model = pinocchioInterfaceMeasured_.getModel();
        auto &data = pinocchioInterfaceMeasured_.getData();

        // 执行前向运动学计算
        pinocchio::forwardKinematics(model, data, qMeasured_, vMeasured_);
        pinocchio::computeJointJacobians(model, data);
        pinocchio::computeJointJacobiansTimeVariation(model, data, qMeasured_, vMeasured_);
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
            jac.setZero(6, info_.generalizedCoordinatesNum);
            pinocchio::getFrameJacobian(model, data, info_.endEffectorFrameIndices[i], pinocchio::LOCAL_WORLD_ALIGNED, jac);
            j_.block(3 * i, 0, 3, info_.generalizedCoordinatesNum) = jac.template topRows<3>();
        }

        // 计算雅可比矩阵的时间导数 dJ
        dj_ = matrix_t(3 * info_.numThreeDofContacts, info_.generalizedCoordinatesNum);
        for (size_t i = 0; i < info_.numThreeDofContacts; ++i)
        {
            jac.setZero(6, info_.generalizedCoordinatesNum);
            pinocchio::getFrameJacobianTimeVariation(model, data, info_.endEffectorFrameIndices[i], pinocchio::LOCAL_WORLD_ALIGNED, jac);
            dj_.block(3 * i, 0, 3, info_.generalizedCoordinatesNum) = jac.template topRows<3>();
        }
    }

    // 更新期望状态：计算期望运动学量
    void WbcBase::updateDesired(const vector_t &stateDesired, const vector_t &inputDesired)
    {
        const auto &model = pinocchioInterfaceDesired_.getModel();
        auto &data = pinocchioInterfaceDesired_.getData();

        mapping_.setPinocchioInterface(pinocchioInterfaceDesired_);
        const auto qDesired = mapping_.getPinocchioJointPosition(stateDesired);
        pinocchio::forwardKinematics(model, data, qDesired);
        pinocchio::computeJointJacobians(model, data, qDesired);
        pinocchio::updateFramePlacements(model, data);
        updateCentroidalDynamics(pinocchioInterfaceDesired_, info_, qDesired);
        const vector_t vDesired = mapping_.getPinocchioJointVelocity(stateDesired, inputDesired);
        pinocchio::forwardKinematics(model, data, qDesired, vDesired);
    }

    /**
     * 构造全身运动学方程
     */
    Task WbcBase::formulateFloatingBaseEomTask()
    {
        auto &data = pinocchioInterfaceMeasured_.getData();

        matrix_t s(info_.actuatedDofNum, info_.generalizedCoordinatesNum);
        s.block(0, 0, info_.actuatedDofNum, 6).setZero();
        s.block(0, 6, info_.actuatedDofNum, info_.actuatedDofNum).setIdentity();

        // 方程变为: M*q_acc - J^T*f - S^T*tau = -nle
        matrix_t a = (matrix_t(info_.generalizedCoordinatesNum, numDecisionVars_) << data.M, -j_.transpose(), -s.transpose()).finished();
        vector_t b = -data.nle;

        return {a, b, matrix_t(), vector_t()};
    }

    /**
     * 构造关节力矩限制任务
     */
    Task WbcBase::formulateTorqueLimitsTask()
    {
        matrix_t d(2 * info_.actuatedDofNum, numDecisionVars_);
        d.setZero();
        matrix_t i = matrix_t::Identity(info_.actuatedDofNum, info_.actuatedDofNum);

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
     * 构造支撑腿不动的任务
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
        frictionPyramic << 0, 0, -1,
            1, 0, -frictionCoeff_,
            -1, 0, -frictionCoeff_,
            0, 1, -frictionCoeff_,
            0, -1, -frictionCoeff_;

        matrix_t d(5 * numContacts_, numDecisionVars_);
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

        Vector6 centroidalMomentumRate = info_.robotMass * getNormalizedCentroidalMomentumRate(pinocchioInterfaceDesired_, info_, inputDesired);
        centroidalMomentumRate.noalias() -= ADot * vDesired;
        centroidalMomentumRate.noalias() -= Aj * jointAccel;

        Vector6 b = AbInv * centroidalMomentumRate;

        return {a, b, matrix_t(), vector_t()};
    }

    /**
     * 构造摆动腿任务 (Swing Leg Tracking)
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
     * 构造接触力任务
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

        b = inputDesired.head(a.rows());

        return {a, b, matrix_t(), vector_t()};
    }
    //-----------------------------------------------单WBC站立-----------------------------------------------
    /**
     * 构造机身加速度任务
     */
    Task WbcBase::formulateBaseAccelTask2(const vector_t &rbdStateMeasured)
    {
        // 1. 初始化 A 矩阵：6行（3平移+3旋转），numDecisionVars_ 列
        matrix_t a(6, numDecisionVars_);
        a.setZero();

        // 2. 锁定决策变量中的基座 6 自由度加速度 (索引 0-5)
        // 这是核心约束：让优化器计算出的 q_acc.head(6) 等于我们算出的参考加速度 b
        a.block(0, 0, 6, 6) = matrix_t::Identity(6, 6);

        // 3. --- 手动设定目标值 (无 Bug 核心：确保目标与测量量坐标系一致) ---
        // 目标高度 (Z轴)
        const scalar_t targetHeight = 0.32;

        // 4. --- 设定 PD 增益 (这里是抗干扰强度的来源) ---
        // 5. --- 计算 PD 加速度 b ---
        vector_t b = vector_t::Zero(6);

        // [平移部分] 索引 0, 1, 2 (X, Y, Z)
        b(0) = baseKpw_ * (-qMeasured_(0)) + baseKdw_ * (-vMeasured_(0));               // X 轴防漂移
        b(1) = baseKpw_ * (-qMeasured_(1)) + baseKdw_ * (-vMeasured_(1));               // Y 轴防漂移
        b(2) = baseKpw_ * (targetHeight - qMeasured_(2)) + baseKdw_ * (-vMeasured_(2)); // Z 轴高度控制

        // [旋转部分] 索引 3, 4, 5(Yaw, Pitch, Roll)
        scalar_t cur_yaw = qMeasured_(3);
        scalar_t cur_pitch = qMeasured_(4);
        scalar_t cur_roll = qMeasured_(5);

        scalar_t vel_yaw = rbdStateMeasured(20);
        scalar_t vel_pitch = rbdStateMeasured(19);
        scalar_t vel_roll = rbdStateMeasured(18);

        b(3) = baseKpj_ * (-cur_roll) + baseKdj_ * (-vel_roll);   // Roll 放在索引 3
        b(4) = baseKpj_ * (-cur_pitch) + baseKdj_ * (-vel_pitch); // Pitch 放在索引 4
        b(5) = baseKpj_ * (-cur_yaw) + baseKdj_ * (-vel_yaw);     // Yaw 放在索引 5
        return {a, b, matrix_t(), vector_t()};
    }
    /**
     * 构造接触力任务：站立时每条腿都要承担四分之一的体重
     */
    Task WbcBase::formulateContactForceTask2()
    {
        matrix_t a(3 * info_.numThreeDofContacts, numDecisionVars_);
        vector_t b(a.rows());
        a.setZero();
        b.setZero();

        for (size_t i = 0; i < info_.numThreeDofContacts; ++i)
        {
            a.block(3 * i, info_.generalizedCoordinatesNum + 3 * i, 3, 3) = matrix_t::Identity(3, 3);
        }

        scalar_t totalMass = info_.robotMass;
        scalar_t forcePerLeg = (totalMass * 9.81) / 4.0;

        for (size_t i = 0; i < info_.numThreeDofContacts; ++i)
        {
            if (contactFlag_[i])
            {
                b(3 * i + 2) = forcePerLeg;
            }
            else
            {
                b.segment<3>(3 * i).setZero();
            }
        }

        return {a, b, matrix_t(), vector_t()};
    }
    /**
     * 构造关节正则化任务 (Joint Regularization)
     * 作用：给关节一个 PD 指令，使其倾向于保持在站立姿态，防止冗余自由度漂移。
     */
    Task WbcBase::formulateJointRegularizationTask()
    {
        matrix_t a(info_.actuatedDofNum, numDecisionVars_);
        a.setZero();

        // [0:5]基座加速度, [6:17]关节加速度, [18:29]接触力, [30:41]力矩
        a.block(0, 6, info_.actuatedDofNum, info_.actuatedDofNum) =
            matrix_t::Identity(info_.actuatedDofNum, info_.actuatedDofNum);

        vector_t qNom(info_.actuatedDofNum);
        for (int i = 0; i < 4; ++i)
        {
            qNom(i * 3 + 0) = 0.0;  // Hip
            qNom(i * 3 + 1) = -0.8; // Thigh
            qNom(i * 3 + 2) = 1.5;  // Calf
        }

        vector_t b = swingKp_ * (qNom - qMeasured_.tail(info_.actuatedDofNum)) - swingKd_ * vMeasured_.tail(info_.actuatedDofNum);

        return {a, b, matrix_t(), vector_t()};
    }
}
