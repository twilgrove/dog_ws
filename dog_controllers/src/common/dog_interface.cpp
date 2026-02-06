#include "common/dog_interface.hpp"
#include <iostream>
#include <string>

#include <pinocchio/fwd.hpp> // forward declarations must be included first.

#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/kinematics.hpp>

#include "ocs2_legged_robot/LeggedRobotInterface.h"

#include <ocs2_centroidal_model/AccessHelperFunctions.h>
#include <ocs2_centroidal_model/CentroidalModelPinocchioMapping.h>
#include <ocs2_centroidal_model/ModelHelperFunctions.h>
#include <ocs2_core/misc/Display.h>
#include <ocs2_core/soft_constraint/StateInputSoftConstraint.h>
#include <ocs2_oc/synchronized_module/SolverSynchronizedModule.h>
#include <ocs2_pinocchio_interface/PinocchioEndEffectorKinematicsCppAd.h>

#include "ocs2_legged_robot/LeggedRobotPreComputation.h"
#include "ocs2_legged_robot/constraint/FrictionConeConstraint.h"
#include "ocs2_legged_robot/constraint/NormalVelocityConstraintCppAd.h"
#include "ocs2_legged_robot/constraint/ZeroForceConstraint.h"
#include "ocs2_legged_robot/constraint/ZeroVelocityConstraintCppAd.h"
#include "ocs2_legged_robot/cost/LeggedRobotQuadraticTrackingCost.h"
#include "ocs2_legged_robot/dynamics/LeggedRobotDynamicsAD.h"

namespace dog_controllers
{

    DogInterface::DogInterface(const std::string &taskFile, const std::string &urdfFile, const std::string &referenceFile)
    {
        bool verbose = false;
        ocs2::loadData::loadCppDataType(taskFile, "legged_robot_interface.verbose", verbose);

        modelSettings_ = ocs2::legged_robot::loadModelSettings(taskFile, "model_settings", verbose);

        pinocchioInterfacePtr_.reset(new ocs2::PinocchioInterface(
            ocs2::centroidal_model::createPinocchioInterface(urdfFile, modelSettings_.jointNames)));

        vector_t defaultJointState = ocs2::centroidal_model::loadDefaultJointState(
            pinocchioInterfacePtr_->getModel().nq - 6, referenceFile);

        centroidalModelInfo_ = ocs2::centroidal_model::createCentroidalModelInfo(
            *pinocchioInterfacePtr_,
            ocs2::centroidal_model::loadCentroidalType(taskFile),
            defaultJointState,
            modelSettings_.contactNames3DoF,
            modelSettings_.contactNames6DoF);

        const auto &model = pinocchioInterfacePtr_->getModel();
        ocs2::CentroidalModelPinocchioMapping pinocchioMapping(centroidalModelInfo_);
        eeKinematicsPtr_.reset(new ocs2::PinocchioEndEffectorKinematics(
            *pinocchioInterfacePtr_, pinocchioMapping, modelSettings_.contactNames3DoF));
    }

} // namespace dog_controllers