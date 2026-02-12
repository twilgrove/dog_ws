#pragma once
#include <Eigen/Core>
#include <pinocchio/fwd.hpp>
#include "wbc/Task.h"
#include <ocs2_centroidal_model/CentroidalModelInfo.h>
namespace dog_controllers
{
    using namespace ocs2;
    using namespace legged_robot;
    class WbcBase
    {
        using Vector6 = Eigen::Matrix<scalar_t, 6, 1>;
        using Matrix6 = Eigen::Matrix<scalar_t, 6, 6>;

        WbcBase(const std::string &taskFile,
                const PinocchioInterface &pinocchioInterface,
                const CentroidalModelInfo &info,
                const PinocchioEndEffectorKinematics &eeKinematics);

        virtual ~WbcBase() = default;

        PinocchioInterface pinocchioInterfaceMeasured_, pinocchioInterfaceDesired_;
        CentroidalModelInfo info_;
        std::unique_ptr<PinocchioEndEffectorKinematics> eeKinematics_;
    }
}