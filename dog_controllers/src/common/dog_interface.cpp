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

        // 1. 模型设置 (modelSettings_): 【静态只读】
        // 存储关节和足端名称。初始化后不再改变，是后续索引的依据。
        modelSettings_ = ocs2::legged_robot::loadModelSettings(taskFile, "model_settings", verbose);

        // 2. 物理本体 (pinocchioInterfacePtr_): 【动态更新】
        // 机器人的“虚拟骨架”。每毫秒(1kHz)都要向其写入最新的电机角度。
        pinocchioInterfacePtr_ = std::make_unique<ocs2::PinocchioInterface>(
            ocs2::centroidal_model::createPinocchioInterface(urdfFile, modelSettings_.jointNames));

        // 3. 数学规格 (centroidalModelInfo_): 【静态只读】
        // 存储总质量、状态维度等数学常数。由“本体”计算而来，但算完后即固定，不随姿态改变。
        centroidalModelInfo_ = ocs2::centroidal_model::createCentroidalModelInfo(
            *pinocchioInterfacePtr_,
            ocs2::centroidal_model::loadCentroidalType(taskFile),
            ocs2::centroidal_model::loadDefaultJointState(pinocchioInterfacePtr_->getModel().nq - 6, referenceFile),
            modelSettings_.contactNames3DoF,
            modelSettings_.contactNames6DoF);

        // 4. 足端工具 (eeKinematicsPtr_): 【随动更新】
        // 专门算“脚在哪”的计算器。它不存数据，但由于它盯着“pinocchioInterfacePtr_”，本体一动，它算出的坐标就跟着变。
        eeKinematicsPtr_ = std::make_unique<ocs2::PinocchioEndEffectorKinematics>(
            *pinocchioInterfacePtr_,
            ocs2::CentroidalModelPinocchioMapping(centroidalModelInfo_),
            modelSettings_.contactNames3DoF);
    }

} // namespace dog_controllers