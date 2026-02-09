#pragma once

#include <ocs2_legged_robot/LeggedRobotInterface.h>
#include <ocs2_pinocchio_interface/PinocchioEndEffectorKinematics.h>
namespace dog_controllers
{
    using namespace ocs2;
    class DogInterface final : public RobotInterface ////ocs2::legged_robot::LeggedRobotInterface
    {
    public:
        DogInterface(const std::string &taskFile,
                     const std::string &urdfFile,
                     const std::string &referenceFile);

        const OptimalControlProblem &getOptimalControlProblem() const override { return problem_; }
        const Initializer &getInitializer() const override { return *initializerPtr_; }

        const PinocchioInterface &getPinocchioInterface() const { return *pinocchioInterfacePtr_; }
        const PinocchioEndEffectorKinematics &getEndEffectorKinematics() const { return *eeKinematicsPtr_; }
        const CentroidalModelInfo &getCentroidalModelInfo() const { return centroidalModelInfo_; }
        // 核心成员
        legged_robot::ModelSettings modelSettings_;
        CentroidalModelInfo centroidalModelInfo_;
        std::unique_ptr<PinocchioInterface> pinocchioInterfacePtr_;
        std::unique_ptr<PinocchioEndEffectorKinematics> eeKinematicsPtr_;

        std::unique_ptr<Initializer> initializerPtr_;
        OptimalControlProblem problem_;
    };

} // namespace dog_controllers