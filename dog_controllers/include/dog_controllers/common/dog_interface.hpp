#pragma once

#include <ocs2_legged_robot/LeggedRobotInterface.h>
#include <ocs2_pinocchio_interface/PinocchioEndEffectorKinematics.h>
namespace dog_controllers
{
    using namespace ocs2;
    class DogInterface : public ocs2::RobotInterface
    {
    public:
        DogInterface(const std::string &taskFile, const std::string &urdfFile, const std::string &referenceFile);

        const ocs2::OptimalControlProblem &getOptimalControlProblem() const override { return problem_; }
        const ocs2::Initializer &getInitializer() const override { return *initializerPtr_; }

        // 核心成员
        ocs2::legged_robot::ModelSettings modelSettings_;
        ocs2::CentroidalModelInfo centroidalModelInfo_;
        std::unique_ptr<ocs2::PinocchioInterface> pinocchioInterfacePtr_;
        std::unique_ptr<ocs2::PinocchioEndEffectorKinematics> eeKinematicsPtr_;

        std::unique_ptr<ocs2::Initializer> initializerPtr_;
        ocs2::OptimalControlProblem problem_;
    };

} // namespace dog_controllers