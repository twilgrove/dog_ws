#include "state_estimation/StateEstimatorBase.hpp"

namespace dog_controllers
{

    StateEstimatorBase::StateEstimatorBase(const LegData *legsPtr_,
                                           const ImuData *imuPtr_,
                                           PinocchioInterface pinocchioInterface,
                                           CentroidalModelInfo info,
                                           const PinocchioEndEffectorKinematics &eeKinematics,
                                           rclcpp_lifecycle::LifecycleNode::SharedPtr &node)
        : legsPtr_(legsPtr_),
          imuPtr_(imuPtr_),
          pinocchioInterface_(std::move(pinocchioInterface)),
          centroidalModelInfo_(std::move(info)), eeKinematics_(eeKinematics.clone()), node_(node)
    {
        results.rbdState_36 = vector_t::Zero(2 * centroidalModelInfo_.generalizedCoordinatesNum);
        results.contactFlags_WBC.fill(true);
        results.contactFlags_MPC = 15;
    }

    void StateEstimatorBase::updateGenericResults(const double &qw, const double &qx, const double &qy, const double &qz,
                                                  const double &ang_vel_x, const double &ang_vel_y, const double &ang_vel_z)
    {
        const quaternion_t quat(qw, qx, qy, qz);

        results.rbdState_36.head<3>() = quatToZyx(quat);

        results.rbdState_36.segment<3>(18) << ang_vel_x, ang_vel_y, ang_vel_z;

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

        for (int i = 0; i < 4; ++i)
        {
            results.contactFlags_WBC[i] = (legsPtr_[i].contact > 0.5f);
        }
        results.contactFlags_MPC = (static_cast<size_t>(results.contactFlags_WBC[0]) << 3) |
                                   (static_cast<size_t>(results.contactFlags_WBC[1]) << 2) |
                                   (static_cast<size_t>(results.contactFlags_WBC[2]) << 1) |
                                   (static_cast<size_t>(results.contactFlags_WBC[3]));
    }

}