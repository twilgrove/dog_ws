#include "state_estimation/KalmanFilterEstimator.hpp"
#include <ocs2_robotic_tools/common/RotationDerivativesTransforms.h>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/info_parser.hpp>
#include <ocs2_core/misc/LoadData.h>
namespace dog_controllers
{
    KalmanFilterEstimator::KalmanFilterEstimator(
        const std::string &taskFile,
        const PinocchioInterface &pinocchioInterface,
        const CentroidalModelInfo &info,
        const PinocchioEndEffectorKinematics &eeKinematics,
        rclcpp_lifecycle::LifecycleNode::SharedPtr &node)
        : StateEstimatorBase(pinocchioInterface, info, eeKinematics, node)
    {

        RCLCPP_INFO(node_->get_logger(), "\033[1;36m====================================================\033[0m");
        RCLCPP_INFO(node_->get_logger(), "\033[1;36m[ 初始化开始 ] 🚀 KalmanFilterEstimator\033[0m");

        sub_ = node->create_subscription<nav_msgs::msg::Odometry>(
            "/ground_truth", rclcpp::SensorDataQoS().keep_last(1),
            [this](const nav_msgs::msg::Odometry::SharedPtr msg)
            { this->odomCallback(msg); });

        boost::property_tree::ptree pt;
        boost::property_tree::read_info(taskFile, pt);
        std::string prefix = "kalmanFilter.";

        loadData::loadPtreeValue(pt, accelFilterAlpha_, prefix + "accelFilterAlpha", false);
        loadData::loadPtreeValue(pt, footRadius_, prefix + "footRadius", false);
        loadData::loadPtreeValue(pt, imuProcessNoisePosition_, prefix + "imuProcessNoisePosition", false);
        loadData::loadPtreeValue(pt, imuProcessNoiseVelocity_, prefix + "imuProcessNoiseVelocity", false);
        loadData::loadPtreeValue(pt, footProcessNoisePosition_, prefix + "footProcessNoisePosition", false);
        loadData::loadPtreeValue(pt, footSensorNoisePosition_, prefix + "footSensorNoisePosition", false);
        loadData::loadPtreeValue(pt, footSensorNoiseVelocity_, prefix + "footSensorNoiseVelocity", false);
        loadData::loadPtreeValue(pt, footHeightSensorNoise_, prefix + "footHeightSensorNoise", false);
        RCLCPP_INFO(node_->get_logger(), "\033[1;33m📊 [PARAM] 已加载卡尔曼参数配置清单:\033[0m");
        RCLCPP_INFO(node_->get_logger(), "\033[1;33m  ├─ 加速度计滤波系数        : \033[0m%.3f", accelFilterAlpha_);
        RCLCPP_INFO(node_->get_logger(), "\033[1;33m  ├─ 足端碰撞球半径          : \033[0m%.3f", footRadius_);
        RCLCPP_INFO(node_->get_logger(), "\033[1;33m  ├─ IMU 位置过程噪声        : \033[0m%.3f", imuProcessNoisePosition_);
        RCLCPP_INFO(node_->get_logger(), "\033[1;33m  ├─ IMU 速度过程噪声        : \033[0m%.3f", imuProcessNoiseVelocity_);
        RCLCPP_INFO(node_->get_logger(), "\033[1;33m  ├─ 足端位置过程噪声        : \033[0m%.3f", footProcessNoisePosition_);
        RCLCPP_INFO(node_->get_logger(), "\033[1;33m  ├─ 足端位置观测噪声        : \033[0m%.3f", footSensorNoisePosition_);
        RCLCPP_INFO(node_->get_logger(), "\033[1;33m  ├─ 足端速度观测噪声        : \033[0m%.3f", footSensorNoiseVelocity_);
        RCLCPP_INFO(node_->get_logger(), "\033[1;33m  └─ 足端高度观测噪声        : \033[0m%.3f", footHeightSensorNoise_);

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

        RCLCPP_INFO(node_->get_logger(), "\033[1;32m[ 初始化完成 ] ✅ KalmanFilterEstimator\033[0m");
        RCLCPP_INFO(node_->get_logger(), "\033[1;32m====================================================\033[0m");
    }

    const vector_t &KalmanFilterEstimator::estimate(const std::array<LegData, 4> &legsPtr, const ImuData &imuData, const rclcpp::Duration &period)
    {

        updateKinematics(legsPtr, imuData);

        prepareMatrices(legsPtr, imuData, period);

        compute();

        results.rbdState_36.segment<3>(3) = xHat_.segment<3>(0);  // 世界系位置
        results.rbdState_36.segment<3>(21) = xHat_.segment<3>(3); // 世界系线速度

        if (msgReceived_)
        {
            nav_msgs::msg::Odometry::SharedPtr currentMsg;
            {
                std::lock_guard<std::mutex> lock(mutex_);
                currentMsg = lastOdomPtr_;
            }
            const auto &pose = currentMsg->pose.pose;
            results.rbdState_36.segment<3>(3) << pose.position.x, pose.position.y, pose.position.z;
        }

        updateObservationFromResults(period);

        return results.rbdState_36;
    }

    void KalmanFilterEstimator::updateKinematics(const std::array<LegData, 4> &legsPtr, const ImuData &imuData)
    {
        updateGenericResults(imuData.ori[3], imuData.ori[0], imuData.ori[1], imuData.ori[2], legsPtr);
        const vector3_t zyx = results.rbdState_36.head<3>();
        results.rbdState_36.segment<3>(18) = getGlobalAngularVelocityFromEulerAnglesZyxDerivatives<scalar_t>(
            zyx, getEulerAnglesZyxDerivativesFromLocalAngularVelocity<scalar_t>(
                     zyx,
                     Eigen::Map<const vector3_t>(imuData.ang_vel)));

        // 构建 Pinocchio 状态向量 (位置设为0，防止自相关误差)
        vector_t qPino = vector_t::Zero(18);
        vector_t vPino = vector_t::Zero(18);

        // 获取欧拉角姿态
        qPino.segment<3>(3) = results.rbdState_36.head<3>();
        // 获取关节位置
        qPino.tail(12) = results.rbdState_36.segment(6, 12);

        // 获取角速度变化率
        vPino.segment<3>(3) = getEulerAnglesZyxDerivativesFromGlobalAngularVelocity<scalar_t>(
            qPino.segment<3>(3),
            results.rbdState_36.segment<3>(18));

        // 获取关节速度
        vPino.tail(12) = results.rbdState_36.segment(24, 12);

        // 正运动学更新
        const auto &model = pinocchioInterface_.getModel();
        auto &data = pinocchioInterface_.getData();
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

    void KalmanFilterEstimator::prepareMatrices(const std::array<LegData, 4> &legsPtr, const ImuData &imuData, const rclcpp::Duration &period)
    {
        scalar_t dt = period.seconds();

        // 1. 更新系统矩阵 A, B
        a_.block<3, 3>(0, 3).diagonal().fill(dt);
        b_.block<3, 3>(0, 0).diagonal().fill(0.5 * dt * dt);
        b_.block<3, 3>(3, 0).diagonal().fill(dt);

        // 2. 更新过程噪声 Q
        q_.diagonal().segment<3>(0).fill(dt / 20.f * imuProcessNoisePosition_);
        q_.diagonal().segment<3>(3).fill(dt * 9.81f / 20.f * imuProcessNoiseVelocity_);

        // 3. 更新加速度 (输入 u)
        quaternion_t quat(imuData.ori[3], imuData.ori[0], imuData.ori[1], imuData.ori[2]);
        vector3_t accelLocal(imuData.lin_acc[0], imuData.lin_acc[1], imuData.lin_acc[2]);

        accelWorld_ = lastaccelWorld_ * (1 - accelFilterAlpha_) + accelFilterAlpha_ * quat._transformVector(accelLocal);
        lastaccelWorld_ = accelWorld_;
        accelWorld_.z() -= 9.81;

        // 4. 动态调整 Q 和 R
        const scalar_t suspect_q = 100.0;
        const scalar_t suspect_r = 100.0;

        for (size_t i = 0; i < numContacts_; ++i)
        {
            // 判断触地状态
            bool isContact = results.contactFlags_WBC[i];

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

    void KalmanFilterEstimator::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        lastOdomPtr_ = msg;
        msgReceived_ = true;
    }
} // namespace dog_controllers
