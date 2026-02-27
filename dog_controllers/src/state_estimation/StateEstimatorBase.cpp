#include "state_estimation/StateEstimatorBase.hpp"
#include <angles/angles.h>
namespace dog_controllers
{

    StateEstimatorBase::StateEstimatorBase(
        const PinocchioInterface &pinocchioInterface,
        const CentroidalModelInfo &info,
        const PinocchioEndEffectorKinematics &eeKinematics,
        rclcpp_lifecycle::LifecycleNode::SharedPtr &node)
        : pinocchioInterface_(pinocchioInterface),
          info_(info),
          eeKinematics_(eeKinematics.clone()),
          node_(node)
    {
        results.rbdState_36 = vector_t::Zero(2 * info.generalizedCoordinatesNum);
        results.contactFlags_WBC.fill(true);
        results.contactFlags_MPC = 15;

        rbdConversions_ = std::make_unique<CentroidalModelRbdConversions>(pinocchioInterface, info);

        currentObservation_.time = 0.0;
        currentObservation_.state.setZero(info.stateDim);
        currentObservation_.input.setZero(info.inputDim);
        currentObservation_.mode = 15;
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

    void StateEstimatorBase::updateObservationFromResults(const rclcpp::Duration &period)
    {

        currentObservation_.time += period.seconds();

        scalar_t yawLast = currentObservation_.state(9);
        currentObservation_.state = rbdConversions_->computeCentroidalStateFromRbdModel(results.rbdState_36);
        currentObservation_.state(9) = yawLast + angles::shortest_angular_distance(yawLast, currentObservation_.state(9));
        currentObservation_.mode = results.contactFlags_MPC;
    }

    void StateEstimatorBase::updateJacobians()
    {
        const auto &model = pinocchioInterface_.getModel();
        auto &data = pinocchioInterface_.getData();

        vector_t qPino = vector_t::Zero(info_.generalizedCoordinatesNum);
        qPino.segment<3>(3) = results.rbdState_36.head<3>();
        qPino.tail(12) = results.rbdState_36.segment(6, 12);

        pinocchio::forwardKinematics(model, data, qPino);
        pinocchio::updateFramePlacements(model, data);

        // OCS2 顺序: 0:LF, 1:RF, 2:LH, 3:RH
        // Pinocchio 顺序: LF:6, LH:9, RF:12, RH:15
        for (size_t i = 0; i < 4; ++i)
        {
            matrix_t J_full = matrix_t::Zero(6, info_.generalizedCoordinatesNum);
            pinocchio::getFrameJacobian(model, data, info_.endEffectorFrameIndices[i], pinocchio::LOCAL_WORLD_ALIGNED, J_full);

            int startCol = 6; // 默认偏移
            if (i == 0)
                startCol = 6; // LF 对应 URDF 的第 1 组关节
            if (i == 1)
                startCol = 12; // RF 对应 URDF 的第 3 组关节
            if (i == 2)
                startCol = 9; // LH 对应 URDF 的第 2 组关节
            if (i == 3)
                startCol = 15; // RH 对应 URDF 的第 4 组关节

            results.legJacobians[i] = J_full.block<3, 3>(0, startCol);
        }
    }
}