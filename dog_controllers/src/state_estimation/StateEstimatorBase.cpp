#include "state_estimation/StateEstimatorBase.hpp"

namespace dog_controllers
{

    void StateEstimatorBase::init(const DogDataBridge *bridge,
                                  PinocchioInterface pinocchioInterface,
                                  CentroidalModelInfo info,
                                  PinocchioEndEffectorKinematics &eeKinematics)
    {
        bridgePtr_ = bridge;
        pinocchioInterface_ = std::move(pinocchioInterface);
        centroidalModelInfo_ = std::move(info);
        eeKinematics_.reset(eeKinematics.clone());

        results.rbdState_36 = vector_t::Zero(2 * centroidalModelInfo_.generalizedCoordinatesNum);
        results.contactFlags_WBC.fill(true);
        results.contactFlags_MPC = 15;
    }

}