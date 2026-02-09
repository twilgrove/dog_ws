#pragma once

#include "state_estimation/StateEstimatorBase.hpp"

namespace dog_controllers
{

    class KalmanFilterEstimator : public StateEstimatorBase
    {
    public:
        KalmanFilterEstimator(const LegData *legsPtr,
                              const ImuData *imuPtr,
                              const std::string &taskFile,
                              PinocchioInterface pinocchioInterface,
                              const PinocchioEndEffectorKinematics &eeKinematics,
                              rclcpp_lifecycle::LifecycleNode::SharedPtr &node);

        const vector_t &estimate() override;

        void loadSettings(const std::string &taskFile, bool verbose);

    private:
        // --- 核心算法模块 ---
        void updateKinematics();
        void prepareMatrices();
        void compute();
        // 维度变量：接触点数、接触总维度、状态向量维度(18)、观测向量维度(28)
        constexpr static size_t numContacts_ = 4;
        constexpr static size_t dimContacts_ = 12;
        constexpr static size_t numState_ = 18;
        constexpr static size_t numObserve_ = 28;

        // --- 卡尔曼滤波核心矩阵 ---
        matrix_t a_;  // 状态转移矩阵 (F): x_k = A * x_{k-1} + B * u
        matrix_t b_;  // 控制输入矩阵 (G): 映射加速度到位置/速度
        matrix_t c_;  // 观测矩阵 (H): y = C * x
        matrix_t ct_; // 观测矩阵转置 (H')

        matrix_t q_; // 过程噪声协方差矩阵 (物理模型的不信任度)
        matrix_t p_; // 后验估计协方差矩阵 (状态估计的不确定性)
        matrix_t r_; // 测量噪声协方差矩阵 (传感器测量的不信任度)

        vector_t xHat_;                         // 状态量: [base_pos(3), base_vel(3), foot_pos(3*n)]
        vector_t ps_, vs_, feetHeights_;        // 观测缓存
        vector3_t accelWorld_, lastaccelWorld_; // 世界系下的加速度 (从 IMU 投影并扣除重力)
        scalar_t accelFilterAlpha_ = 0.2;       // 加速度低通滤波系数：0~1 之间

        // --- 配置参数 ---
        scalar_t footRadius_ = 0.02;
        scalar_t imuProcessNoisePosition_ = 0.02;
        scalar_t imuProcessNoiseVelocity_ = 0.02;
        scalar_t footProcessNoisePosition_ = 0.002;
        scalar_t footSensorNoisePosition_ = 0.005;
        scalar_t footSensorNoiseVelocity_ = 0.1;
        scalar_t footHeightSensorNoise_ = 0.01;

        // 时间管理
        rclcpp::Time lastTime_;
    };

} // namespace dog_controllers