#include "state_estimation/KalmanFilterEstimator.hpp"
#include <ocs2_robotic_tools/common/RotationDerivativesTransforms.h>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/kinematics.hpp>

namespace dog_controllers
{

    KalmanFilterEstimator::KalmanFilterEstimator(const LegData *legsPtr, const ImuData *imuPtr,
                                                 PinocchioInterface pinocchioInterface,
                                                 CentroidalModelInfo info,
                                                 const PinocchioEndEffectorKinematics &eeKinematics,
                                                 rclcpp_lifecycle::LifecycleNode::SharedPtr &node)
        : StateEstimatorBase(legsPtr, imuPtr, std::move(pinocchioInterface), std::move(info), eeKinematics, node)
    {

        // 1. 初始化矩阵 A, B
        xHat_.setZero(numState_);
        a_.setIdentity(numState_, numState_);
        b_.setZero(numState_, 3);

        // 构造 C 矩阵
        c_.setZero(numObserve_, numState_);
        for (size_t i = 0; i < numContacts_; ++i)
        {
            c_.block(3 * i, 0, 3, 3) << matrix3_t::Identity();                  // pos观测中的基座部分
            c_.block(3 * (numContacts_ + i), 3, 3, 3) << matrix3_t::Identity(); // vel观测中的基座部分
            c_(2 * dimContacts_ + i, 6 + 3 * i + 2) = 1.0;                      // 高度观测: 提取脚的世界坐标 Z
        }
        c_.block(0, 6, dimContacts_, dimContacts_) = -matrix_t::Identity(dimContacts_, dimContacts_);
        ct_ = c_.transpose();
        // 2. 初始化协方差
        p_.setIdentity(numState_, numState_);
        p_ *= 100.0;
        q_.setZero(numState_, numState_);
        r_.setZero(numObserve_, numObserve_);

        // 3. 缓存初始化
        ps_.setZero(dimContacts_);
        vs_.setZero(dimContacts_);
        feetHeights_.setZero(numContacts_);
        accelWorld_.setZero();
        lastaccelWorld_.setZero();

        eeKinematics_->setPinocchioInterface(pinocchioInterface_);

        lastTime_ = node_->now();
    }

    const vector_t &KalmanFilterEstimator::estimate()
    {

        updateKinematics();

        prepareMatrices();

        compute();

        results.rbdState_36.segment<3>(3) = xHat_.segment<3>(0);  // 世界系位置
        results.rbdState_36.segment<3>(21) = xHat_.segment<3>(3); // 世界系线速度

        return results.rbdState_36;
    }

    void KalmanFilterEstimator::updateKinematics()
    {
        updateGenericResults(
            imuPtr_->ori[3], imuPtr_->ori[0], imuPtr_->ori[1], imuPtr_->ori[2],
            imuPtr_->ang_vel[0], imuPtr_->ang_vel[1], imuPtr_->ang_vel[2]);

        const auto &model = pinocchioInterface_.getModel();
        auto &data = pinocchioInterface_.getData();
        size_t actuatedDofNum = centroidalModelInfo_.actuatedDofNum;
        size_t generalizedCoordinatesNum = centroidalModelInfo_.generalizedCoordinatesNum;

        // 构建 Pinocchio 状态向量 (位置设为0，防止自相关误差)
        vector_t qPino = vector_t::Zero(generalizedCoordinatesNum);
        vector_t vPino = vector_t::Zero(generalizedCoordinatesNum);

        // 获取欧拉角姿态
        qPino.segment<3>(3) = results.rbdState_36.head<3>();
        qPino.tail(actuatedDofNum) = results.rbdState_36.segment(6, actuatedDofNum);

        // 角速度转换: Global Angular Velocity -> Euler Zyx Derivatives
        vPino.segment<3>(3) = getEulerAnglesZyxDerivativesFromGlobalAngularVelocity<scalar_t>(
            qPino.segment<3>(3),
            results.rbdState_36.segment<3>(generalizedCoordinatesNum));
        vPino.tail(actuatedDofNum) = results.rbdState_36.segment(6 + generalizedCoordinatesNum, actuatedDofNum);

        // 正运动学更新
        pinocchio::forwardKinematics(model, data, qPino, vPino);
        pinocchio::updateFramePlacements(model, data);

        // 获取足端相对于机体的位置和速度 (运动学观测值)
        const auto eePos = eeKinematics_->getPosition(vector_t());
        const auto eeVel = eeKinematics_->getVelocity(vector_t(), vector_t());

        for (size_t i = 0; i < numContacts_; ++i)
        {
            ps_.segment(3 * i, 3) = -eePos[i];
            ps_.segment(3 * i, 3)[2] += footRadius_; // 补偿半径
            vs_.segment(3 * i, 3) = -eeVel[i];
        }
    }

    void KalmanFilterEstimator::prepareMatrices()
    {
        // 获取当前时间和时间步长
        rclcpp::Time currentTime = node_->now();
        scalar_t dt = (currentTime - lastTime_).seconds();
        lastTime_ = currentTime;

        // 1. 更新系统矩阵 A, B
        a_.block<3, 3>(0, 3).diagonal().fill(dt);
        b_.block<3, 3>(0, 0).diagonal().fill(0.5 * dt * dt);
        b_.block<3, 3>(3, 0).diagonal().fill(dt);

        // 2. 更新过程噪声 Q
        q_.diagonal().segment<3>(0).fill(dt / 20.f * imuProcessNoisePosition_);
        q_.diagonal().segment<3>(3).fill(dt * 9.81f / 20.f * imuProcessNoiseVelocity_);

        // 3. 更新加速度 (输入 u)
        quaternion_t quat(imuPtr_->ori[3], imuPtr_->ori[0], imuPtr_->ori[1], imuPtr_->ori[2]);
        vector3_t accelLocal(imuPtr_->lin_acc[0], imuPtr_->lin_acc[1], imuPtr_->lin_acc[2]);
        accelWorld_ = lastaccelWorld_ * (1 - accelFilterAlpha_) + accelFilterAlpha_ * quat._transformVector(accelLocal);
        lastaccelWorld_ = accelWorld_;
        accelWorld_.z() -= 9.81;

        // 4. 动态调整 Q 和 R
        const scalar_t suspect_q = 100.0;
        const scalar_t suspect_r = 1e4;

        for (size_t i = 0; i < numContacts_; ++i)
        {
            // 判断触地状态
            bool isContact = (legsPtr_[i].contact > 0.5);

            // 更新 Q
            scalar_t q_m = isContact ? 1.0 : suspect_q;
            q_.diagonal().segment<3>(6 + 3 * i).fill(dt * footProcessNoisePosition_ * q_m);

            // 更新 R
            scalar_t r_m = isContact ? 1.0 : suspect_r;
            // R_pos (0 ~ dimContacts_)
            r_.diagonal().segment<3>(3 * i).fill(footSensorNoisePosition_ * r_m);
            // R_vel (dimContacts_ ~ 2*dimContacts_)
            r_.diagonal().segment<3>(dimContacts_ + 3 * i).fill(footSensorNoiseVelocity_ * r_m);
            // R_height (2*dimContacts_ ~ numObserve_)
            r_(2 * dimContacts_ + i) = footHeightSensorNoise_ * r_m;
        }
    }

    void KalmanFilterEstimator::compute()
    {
        // --- 1. 预测步 (Predict) ---
        xHat_ = a_ * xHat_ + b_ * accelWorld_; // 状态预测：x = A*x + B*u
        p_ = a_ * p_ * a_.transpose() + q_;    // 协方差预测：P = A*P*A' + Q

        // --- 2. 准备修正步 (Prepare Correct) ---
        vector_t y_obs(numObserve_);
        y_obs << ps_, vs_, feetHeights_;

        matrix_t tmp_pct;
        tmp_pct.noalias() = p_ * ct_;

        matrix_t s;
        s.noalias() = c_ * tmp_pct + r_; // 创新协方差：S = C*P*C' + R

        auto ldlt = s.ldlt();

        // --- 3. 状态更新 (Update State) ---
        vector_t e = y_obs - c_ * xHat_; // 创新/残差：e = y - C*x

        xHat_ += tmp_pct * ldlt.solve(e); // x = x + (P*Ct) * (S^-1 * e)

        // --- 4. 协方差更新 (Update Covariance) ---

        p_ -= tmp_pct * ldlt.solve(tmp_pct.transpose()); // 协方差更新：P = P - (P*Ct) * S^-1 * (C*P)

        // --- 5. 数值稳定性 ---
        p_ = (p_ + p_.transpose()) * 0.5;
    }

} // namespace dog_controllers
