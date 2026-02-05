#include "state_estimation/StateEstimatorBase.hpp"

namespace dog_controllers
{

    StateEstimatorBase::StateEstimatorBase(const DogDataBridge *bridge,
                                           PinocchioInterface pinocchioInterface,
                                           CentroidalModelInfo info,
                                           const PinocchioEndEffectorKinematics &eeKinematics, rclcpp_lifecycle::LifecycleNode::SharedPtr &node) : bridgePtr_(bridge),
                                                                                                                                                   pinocchioInterface_(std::move(pinocchioInterface)),
                                                                                                                                                   centroidalModelInfo_(std::move(info)), eeKinematics_(eeKinematics.clone()), node_(node)
    {
        if (bridgePtr_)
        {
            legsPtr_ = bridgePtr_->legs.data();
            imuPtr_ = &(bridgePtr_->imu);
        }

        results.rbdState_36 = vector_t::Zero(2 * centroidalModelInfo_.generalizedCoordinatesNum);
        results.contactFlags_WBC.fill(true);
        results.contactFlags_MPC = 15;
    }

    void StateEstimatorBase::updateGenericResults()
    {
        // 1. 关节角度与速度 (6-17, 24-35)
        for (int i = 0; i < 4; ++i)
        {
            const auto &leg = legsPtr_[i];
            const int idx = 6 + i * 3;
            results.rbdState_36(idx) = leg.hip.pos;
            results.rbdState_36(idx + 1) = leg.thigh.pos;
            results.rbdState_36(idx + 2) = leg.calf.pos;

            const int v_idx = 24 + i * 3;
            results.rbdState_36(v_idx) = leg.hip.vel;
            results.rbdState_36(v_idx + 1) = leg.thigh.vel;
            results.rbdState_36(v_idx + 2) = leg.calf.vel;
        }

        // 2. 触地逻辑与 MPC 模式映射
        for (int i = 0; i < 4; ++i)
        {
            results.contactFlags_WBC[i] = (legsPtr_[i].contact > 0.5f);
        }
        results.contactFlags_MPC = (static_cast<size_t>(results.contactFlags_WBC[0])) |
                                   (static_cast<size_t>(results.contactFlags_WBC[1]) << 1) |
                                   (static_cast<size_t>(results.contactFlags_WBC[2]) << 2) |
                                   (static_cast<size_t>(results.contactFlags_WBC[3]) << 3);
    }

}