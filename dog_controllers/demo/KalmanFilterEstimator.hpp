namespace dog_controllers
{

    class KalmanFilterEstimator final : public StateEstimatorBase
    {
    public:
        using StateEstimatorBase::StateEstimatorBase;

        // 同时更新 IMU 和 关节，存入缓存
        void updateData(const vector_t &jointPos,
                        const vector_t &jointVel,
                        const Eigen::Quaternion<scalar_t> &quat,
                        const vector3_t &angVel,
                        const vector3_t &linAccel) override
        {
            lastJointPos_ = jointPos;
            lastJointVel_ = jointVel;
            lastQuat_ = quat;
            lastAngVel_ = angVel;
            lastLinAccel_ = linAccel;
        }

        vector_t estimate(const rclcpp::Time &time, const rclcpp::Duration &period) override;
    };
}