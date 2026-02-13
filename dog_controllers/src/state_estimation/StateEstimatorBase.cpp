#include "state_estimation/StateEstimatorBase.hpp"

namespace dog_controllers
{

    StateEstimatorBase::StateEstimatorBase(
        const PinocchioInterface &pinocchioInterface,
        const PinocchioEndEffectorKinematics &eeKinematics,
        rclcpp_lifecycle::LifecycleNode::SharedPtr &node)
        : pinocchioInterface_(pinocchioInterface),
          eeKinematics_(eeKinematics.clone()), node_(node)
    {
        results.rbdState_36 = vector_t::Zero(36);
        results.contactFlags_WBC.fill(true);
        results.contactFlags_MPC = 15;
    }

    void StateEstimatorBase::updateGenericResults(const double &qw, const double &qx, const double &qy, const double &qz, const std::array<LegData, 4> &legsPtr_)
    {
        const quaternion_t quat(qw, qx, qy, qz);
        results.rbdState_36.head<3>() = quatToZyx(quat) - zyxOffset_;
        // LF, LH, RF, RH
        for (int i = 0; i < 4; ++i)
        {
            const auto &leg = legsPtr_[i];
            results.rbdState_36.segment<3>(6 + i * 3) << leg.hip.pos, leg.thigh.pos, leg.calf.pos;
            results.rbdState_36.segment<3>(24 + i * 3) << leg.hip.vel, leg.thigh.vel, leg.calf.vel;
        }
        // LF, RF, LH, RH
        results.contactFlags_WBC[0] = (legsPtr_[0].contact > 0.5f);
        results.contactFlags_WBC[1] = (legsPtr_[2].contact > 0.5f);
        results.contactFlags_WBC[2] = (legsPtr_[1].contact > 0.5f);
        results.contactFlags_WBC[3] = (legsPtr_[3].contact > 0.5f);

        results.contactFlags_MPC = (static_cast<size_t>(results.contactFlags_WBC[0]) << 3) |
                                   (static_cast<size_t>(results.contactFlags_WBC[1]) << 2) |
                                   (static_cast<size_t>(results.contactFlags_WBC[2]) << 1) |
                                   (static_cast<size_t>(results.contactFlags_WBC[3]));
    }

}