/******************************************************************************
Copyright (c) 2021, Farbod Farshidian. All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

 * Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

 * Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

 * Neither the name of the copyright holder nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
******************************************************************************/

#include <iostream>
#include <string>

#include <rclcpp/rclcpp.hpp>

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

// Boost
#include <boost/filesystem/operations.hpp>
#include <boost/filesystem/path.hpp>

namespace ocs2
{
  namespace legged_robot
  {

    /******************************************************************************************************/
    LeggedRobotInterface::LeggedRobotInterface(const std::string &taskFile,
                                               const std::string &urdfFile,
                                               const std::string &referenceFile,
                                               bool useHardFrictionConeConstraint)
        : useHardFrictionConeConstraint_(useHardFrictionConeConstraint)
    {
      RCLCPP_INFO(rclcpp::get_logger("DogNmpcWbcController"), "\033[1;36m====================================================\033[0m");
      RCLCPP_INFO(rclcpp::get_logger("DogNmpcWbcController"), "\033[1;36m[ 初始化开始 ] 🚀 LeggedRobotInterface\033[0m");

      bool verbose;
      loadData::loadCppDataType(taskFile, "legged_robot_interface.verbose", verbose);

      // load setting from loading file
      modelSettings_ = loadModelSettings(taskFile, "model_settings", verbose);
      mpcSettings_ = mpc::loadSettings(taskFile, "mpc", verbose);
      ddpSettings_ = ddp::loadSettings(taskFile, "ddp", verbose);
      sqpSettings_ = sqp::loadSettings(taskFile, "sqp", verbose);
      ipmSettings_ = ipm::loadSettings(taskFile, "ipm", verbose);
      rolloutSettings_ = rollout::loadSettings(taskFile, "rollout", verbose);

      // OptimalControlProblem
      setupOptimalControlProblem(taskFile, urdfFile, referenceFile, verbose);

      // initial state
      initialState_.setZero(centroidalModelInfo_.stateDim);
      loadData::loadEigenMatrix(taskFile, "initialState", initialState_);

      RCLCPP_INFO(rclcpp::get_logger("DogNmpcWbcController"), "\033[1;32m[ 初始化完成 ] ✅ LeggedRobotInterface\033[0m");
      RCLCPP_INFO(rclcpp::get_logger("DogNmpcWbcController"), "\033[1;32m====================================================\033[0m");
    }

    /******************************************************************************************************/
    void LeggedRobotInterface::setupOptimalControlProblem(
        const std::string &taskFile,
        const std::string &urdfFile,
        const std::string &referenceFile,
        bool verbose)
    {
      // ===========================================================================
      // 1. 【物理引擎模块】初始化 Pinocchio 接口
      // 作用：加载 URDF 模型，建立机器人的运动学树结构，它是所有动力学计算的基础。
      // ===========================================================================
      pinocchioInterfacePtr_ = std::make_unique<ocs2::PinocchioInterface>(
          ocs2::centroidal_model::createPinocchioInterface(urdfFile, modelSettings_.jointNames));

      // ===========================================================================
      // 2. 【数学模型模块】创建质心模型信息 (Centroidal Model Info)
      // 作用：定义机器人的状态维度（位置、姿态、动量）和输入维度（足端力）。
      // 它决定了 MPC 求解时的矩阵大小。
      // ===========================================================================
      centroidalModelInfo_ = centroidal_model::createCentroidalModelInfo(
          *pinocchioInterfacePtr_,
          centroidal_model::loadCentroidalType(taskFile),
          centroidal_model::loadDefaultJointState(pinocchioInterfacePtr_->getModel().nq - 6, referenceFile),
          modelSettings_.contactNames3DoF,
          modelSettings_.contactNames6DoF);

      // ===========================================================================
      // 3. 【运动学模块】末端执行器（足端）运动学句柄
      // 作用：用于计算足端在空间中的位置和雅可比矩阵，主要用于后续的约束计算。
      // ===========================================================================
      eeKinematicsPtrs_ = std::make_unique<ocs2::PinocchioEndEffectorKinematics>(
          *pinocchioInterfacePtr_,
          ocs2::CentroidalModelPinocchioMapping(centroidalModelInfo_),
          modelSettings_.contactNames3DoF);

      // ===========================================================================
      // 4. 【任务规划模块】摆动腿轨迹规划与步态管理器
      // 作用：SwingTrajectoryPlanner 负责算脚抬多高；ReferenceManager 负责管什么时候迈哪只脚。
      // ===========================================================================
      auto swingTrajectoryPlanner = std::make_unique<SwingTrajectoryPlanner>(
          loadSwingTrajectorySettings(taskFile, "swing_trajectory_config", verbose), 4);

      referenceManagerPtr_ = std::make_shared<SwitchedModelReferenceManager>(
          loadGaitSchedule(referenceFile, verbose),
          std::move(swingTrajectoryPlanner));

      // ===========================================================================
      // 5. 【容器初始化】创建 OCP（最优控制问题）对象
      // 作用：这是所有 Cost（代价）和 Constraints（约束）的“篮子”。
      // ===========================================================================
      problemPtr_.reset(new OptimalControlProblem);

      // ===========================================================================
      // 6. 【动力学模块】系统动力学定义 (AD 自动微分版)
      // 作用：告诉 MPC 机器人运动遵循的物理规律（F=ma）。
      // 这里使用了 CppAD 自动求导，免去了手写导数矩阵的痛苦。
      // ===========================================================================
      bool useAnalyticalGradientsDynamics = false;
      loadData::loadCppDataType(
          taskFile,
          "legged_robot_interface.useAnalyticalGradientsDynamics",
          useAnalyticalGradientsDynamics);
      std::unique_ptr<SystemDynamicsBase> dynamicsPtr;
      if (useAnalyticalGradientsDynamics)
      {
        throw std::runtime_error("解析导数版本尚未实现！");
      }
      else
      {
        const std::string modelName = "dynamics";
        dynamicsPtr.reset(new LeggedRobotDynamicsAD(*pinocchioInterfacePtr_,
                                                    centroidalModelInfo_,
                                                    modelName,
                                                    modelSettings_));
      }
      problemPtr_->dynamicsPtr = std::move(dynamicsPtr);

      // ===========================================================================
      // 7. 【代价函数模块】基础跟踪代价 (Cost)
      // 作用：告诉机器人“你该怎么动”。比如：身子要稳、跟着参考路径走。
      // ===========================================================================
      problemPtr_->costPtr->add(
          "baseTrackingCost",
          getBaseTrackingCost(taskFile, centroidalModelInfo_, true));

      // ===========================================================================
      // 8. 【约束条件模块】循环为每只腿添加安全规则
      // 作用：遍历四条腿，添加摩擦锥、零力（摆动腿）、零速度（支撑腿）等约束。
      // ===========================================================================
      scalar_t frictionCoefficient = 0.7;
      RelaxedBarrierPenalty::Config barrierPenaltyConfig;
      std::tie(frictionCoefficient, barrierPenaltyConfig) = loadFrictionConeSettings(taskFile, verbose);

      bool useAnalyticalGradientsConstraints = false;
      loadData::loadCppDataType(
          taskFile, "legged_robot_interface.useAnalyticalGradientsConstraints",
          useAnalyticalGradientsConstraints);

      for (size_t i = 0; i < centroidalModelInfo_.numThreeDofContacts; i++)
      {
        const std::string &footName = modelSettings_.contactNames3DoF[i];

        std::unique_ptr<EndEffectorKinematics<scalar_t>> eeKinematicsPtr;

        // 为每只脚生成自动微分的运动学代码（CppAD），用于实时计算雅可比
        if (useAnalyticalGradientsConstraints)
        {
          throw std::runtime_error(
              "[LeggedRobotInterface::setupOptimalControlProblem] The analytical "
              "end-effector linear constraint is not implemented!");
        }
        else
        {
          const auto infoCppAd = centroidalModelInfo_.toCppAd();
          const CentroidalModelPinocchioMappingCppAd pinocchioMappingCppAd(infoCppAd);
          auto velocityUpdateCallback = [&infoCppAd](const ad_vector_t &state, PinocchioInterfaceCppAd &pinocchioInterfaceAd)
          {
            const ad_vector_t q = centroidal_model::getGeneralizedCoordinates(state, infoCppAd);
            updateCentroidalDynamics(pinocchioInterfaceAd, infoCppAd, q);
          };
          eeKinematicsPtr.reset(new PinocchioEndEffectorKinematicsCppAd(
              *pinocchioInterfacePtr_, pinocchioMappingCppAd, {footName},
              centroidalModelInfo_.stateDim, centroidalModelInfo_.inputDim,
              velocityUpdateCallback, footName, modelSettings_.modelFolderCppAd,
              modelSettings_.recompileLibrariesCppAd, modelSettings_.verboseCppAd));
        }

        // A. 摩擦锥约束：防止脚底打滑
        if (useHardFrictionConeConstraint_) // 选择硬约束还是软约束
        {
          problemPtr_->inequalityConstraintPtr->add(footName + "_frictionCone", getFrictionConeConstraint(i, frictionCoefficient));
        }
        else
        {
          problemPtr_->softConstraintPtr->add(footName + "_frictionCone", getFrictionConeSoftConstraint(i, frictionCoefficient, barrierPenaltyConfig));
        }
        // B. 零力约束：摆动时脚不能使劲
        problemPtr_->equalityConstraintPtr->add(footName + "_zeroForce", getZeroForceConstraint(i));

        // C. 零速度约束：踩地时脚不能乱动
        problemPtr_->equalityConstraintPtr->add(footName + "_zeroVelocity", getZeroVelocityConstraint(*eeKinematicsPtr, i, useAnalyticalGradientsConstraints));

        // D. 法向速度约束：控制抬腿和落脚的速度
        problemPtr_->equalityConstraintPtr->add(footName + "_normalVelocity", getNormalVelocityConstraint(*eeKinematicsPtr, i, useAnalyticalGradientsConstraints));
      }

      // ===========================================================================
      // 9. 【优化加速模块】预计算 (Pre-Computation)
      // 作用：这是改版的精髓。一次性算出这一帧需要的所有中间变量并缓存，大幅降低 CPU 负载。
      // ===========================================================================
      problemPtr_->preComputationPtr.reset(new LeggedRobotPreComputation(
          *pinocchioInterfacePtr_,
          centroidalModelInfo_,
          *referenceManagerPtr_->getSwingTrajectoryPlanner(),
          modelSettings_));

      // ===========================================================================
      // 10. 【预测模拟模块】数值积分 (Rollout)
      // 作用：MPC 需要“预知未来”，Rollout 负责在给定控制量下模拟出未来的运动轨迹。
      // ===========================================================================
      rolloutPtr_.reset(new TimeTriggeredRollout(*problemPtr_->dynamicsPtr, rolloutSettings_));

      // ===========================================================================
      // 11. 【求解启动模块】初始化策略 (Initializer)
      // 作用：为 MPC 求解器提供一个“靠谱”的初始猜测值，让它收敛得更快。
      // ===========================================================================
      constexpr bool extendNormalizedMomentum = true;
      initializerPtr_.reset(new LeggedRobotInitializer(
          centroidalModelInfo_,
          *referenceManagerPtr_,
          extendNormalizedMomentum));
    }

    /******************************************************************************************************/
    std::shared_ptr<GaitSchedule> LeggedRobotInterface::loadGaitSchedule(
        const std::string &file,
        bool verbose) const
    {
      const auto initModeSchedule =
          loadModeSchedule(file, "initialModeSchedule", false);
      const auto defaultModeSequenceTemplate =
          loadModeSequenceTemplate(file, "defaultModeSequenceTemplate", false);

      const auto defaultGait = [&]
      {
        Gait gait{};
        gait.duration = defaultModeSequenceTemplate.switchingTimes.back();
        // Events: from time -> phase
        std::for_each(defaultModeSequenceTemplate.switchingTimes.begin() + 1,
                      defaultModeSequenceTemplate.switchingTimes.end() - 1,
                      [&](double eventTime)
                      {
                        gait.eventPhases.push_back(eventTime / gait.duration);
                      });
        // Modes:
        gait.modeSequence = defaultModeSequenceTemplate.modeSequence;
        return gait;
      }();

      // display
      if (verbose)
      {
        std::cerr << "\n#### Modes Schedule: ";
        std::cerr << "\n#### "
                     "============================================================="
                     "================\n";
        std::cerr << "Initial Modes Schedule: \n"
                  << initModeSchedule;
        std::cerr << "Default Modes Sequence Template: \n"
                  << defaultModeSequenceTemplate;
        std::cerr << "#### "
                     "============================================================="
                     "================\n";
      }

      return std::make_shared<GaitSchedule>(
          initModeSchedule, defaultModeSequenceTemplate,
          modelSettings_.phaseTransitionStanceTime);
    }

    /******************************************************************************************************/
    matrix_t LeggedRobotInterface::initializeInputCostWeight(
        const std::string &taskFile,
        const CentroidalModelInfo &info)
    {
      const size_t totalContactDim = 3 * info.numThreeDofContacts;

      vector_t initialState(centroidalModelInfo_.stateDim);
      loadData::loadEigenMatrix(taskFile, "initialState", initialState);

      const auto &model = pinocchioInterfacePtr_->getModel();
      auto &data = pinocchioInterfacePtr_->getData();
      const auto q = centroidal_model::getGeneralizedCoordinates(
          initialState, centroidalModelInfo_);
      pinocchio::computeJointJacobians(model, data, q);
      pinocchio::updateFramePlacements(model, data);

      matrix_t baseToFeetJacobians(totalContactDim, info.actuatedDofNum);
      for (size_t i = 0; i < info.numThreeDofContacts; i++)
      {
        matrix_t jacobianWorldToContactPointInWorldFrame =
            matrix_t::Zero(6, info.generalizedCoordinatesNum);
        pinocchio::getFrameJacobian(
            model, data, model.getBodyId(modelSettings_.contactNames3DoF[i]),
            pinocchio::LOCAL_WORLD_ALIGNED,
            jacobianWorldToContactPointInWorldFrame);

        baseToFeetJacobians.block(3 * i, 0, 3, info.actuatedDofNum) =
            jacobianWorldToContactPointInWorldFrame.block(0, 6, 3,
                                                          info.actuatedDofNum);
      }

      matrix_t R_taskspace(totalContactDim + totalContactDim,
                           totalContactDim + totalContactDim);
      loadData::loadEigenMatrix(taskFile, "R", R_taskspace);

      matrix_t R = matrix_t::Zero(info.inputDim, info.inputDim);
      // Contact Forces
      R.topLeftCorner(totalContactDim, totalContactDim) =
          R_taskspace.topLeftCorner(totalContactDim, totalContactDim);
      // Joint velocities
      R.bottomRightCorner(info.actuatedDofNum, info.actuatedDofNum) =
          baseToFeetJacobians.transpose() *
          R_taskspace.bottomRightCorner(totalContactDim, totalContactDim) *
          baseToFeetJacobians;
      return R;
    }

    /******************************************************************************************************/
    std::unique_ptr<StateInputCost> LeggedRobotInterface::getBaseTrackingCost(
        const std::string &taskFile,
        const CentroidalModelInfo &info,
        bool verbose)
    {
      matrix_t Q(info.stateDim, info.stateDim);
      loadData::loadEigenMatrix(taskFile, "Q", Q);
      matrix_t R = initializeInputCostWeight(taskFile, info);

      if (verbose)
      {
        std::cerr << "\n #### Base Tracking Cost Coefficients: ";
        std::cerr << "\n #### "
                     "============================================================="
                     "================\n";
        std::cerr << "Q:\n"
                  << Q << "\n";
        std::cerr << "R:\n"
                  << R << "\n";
        std::cerr << " #### "
                     "============================================================="
                     "================\n";
      }

      return std::make_unique<LeggedRobotStateInputQuadraticCost>(
          std::move(Q), std::move(R), info, *referenceManagerPtr_);
    }

    /******************************************************************************************************/
    std::pair<scalar_t, RelaxedBarrierPenalty::Config>
    LeggedRobotInterface::loadFrictionConeSettings(const std::string &taskFile,
                                                   bool verbose) const
    {
      boost::property_tree::ptree pt;
      boost::property_tree::read_info(taskFile, pt);
      const std::string prefix = "frictionConeSoftConstraint.";

      scalar_t frictionCoefficient = 1.0;
      RelaxedBarrierPenalty::Config barrierPenaltyConfig;
      if (verbose)
      {
        std::cerr << "\n #### Friction Cone Settings: ";
        std::cerr << "\n #### "
                     "============================================================="
                     "================\n";
      }
      loadData::loadPtreeValue(pt, frictionCoefficient,
                               prefix + "frictionCoefficient", verbose);
      loadData::loadPtreeValue(pt, barrierPenaltyConfig.mu, prefix + "mu", verbose);
      loadData::loadPtreeValue(pt, barrierPenaltyConfig.delta, prefix + "delta",
                               verbose);
      if (verbose)
      {
        std::cerr << " #### "
                     "============================================================="
                     "================\n";
      }

      return {frictionCoefficient, std::move(barrierPenaltyConfig)};
    }

    /******************************************************************************************************/
    std::unique_ptr<StateInputConstraint>
    LeggedRobotInterface::getFrictionConeConstraint(size_t contactPointIndex,
                                                    scalar_t frictionCoefficient)
    {
      FrictionConeConstraint::Config frictionConeConConfig(frictionCoefficient);
      return std::make_unique<FrictionConeConstraint>(
          *referenceManagerPtr_, std::move(frictionConeConConfig),
          contactPointIndex, centroidalModelInfo_);
    }

    /******************************************************************************************************/
    std::unique_ptr<StateInputCost>
    LeggedRobotInterface::getFrictionConeSoftConstraint(
        size_t contactPointIndex,
        scalar_t frictionCoefficient,
        const RelaxedBarrierPenalty::Config &barrierPenaltyConfig)
    {
      return std::make_unique<StateInputSoftConstraint>(
          getFrictionConeConstraint(contactPointIndex, frictionCoefficient),
          std::make_unique<RelaxedBarrierPenalty>(barrierPenaltyConfig));
    }

    /******************************************************************************************************/
    std::unique_ptr<StateInputConstraint>
    LeggedRobotInterface::getZeroForceConstraint(size_t contactPointIndex)
    {
      return std::make_unique<ZeroForceConstraint>(
          *referenceManagerPtr_, contactPointIndex, centroidalModelInfo_);
    }

    /******************************************************************************************************/
    std::unique_ptr<StateInputConstraint>
    LeggedRobotInterface::getZeroVelocityConstraint(
        const EndEffectorKinematics<scalar_t> &eeKinematics,
        size_t contactPointIndex,
        bool useAnalyticalGradients)
    {
      auto eeZeroVelConConfig = [](scalar_t positionErrorGain)
      {
        EndEffectorLinearConstraint::Config config;
        config.b.setZero(3);
        config.Av.setIdentity(3, 3);
        if (!numerics::almost_eq(positionErrorGain, 0.0))
        {
          config.Ax.setZero(3, 3);
          config.Ax(2, 2) = positionErrorGain;
        }
        return config;
      };

      if (useAnalyticalGradients)
      {
        throw std::runtime_error(
            "[LeggedRobotInterface::getZeroVelocityConstraint] The analytical "
            "end-effector zero velocity constraint is not implemented!");
      }
      else
      {
        return std::make_unique<ZeroVelocityConstraintCppAd>(
            *referenceManagerPtr_, eeKinematics, contactPointIndex,
            eeZeroVelConConfig(modelSettings_.positionErrorGain));
      }
    }

    /******************************************************************************************************/
    std::unique_ptr<StateInputConstraint>
    LeggedRobotInterface::getNormalVelocityConstraint(
        const EndEffectorKinematics<scalar_t> &eeKinematics,
        size_t contactPointIndex,
        bool useAnalyticalGradients)
    {
      if (useAnalyticalGradients)
      {
        throw std::runtime_error(
            "[LeggedRobotInterface::getNormalVelocityConstraint] The analytical "
            "end-effector normal velocity constraint is not implemented!");
      }
      else
      {
        return std::make_unique<NormalVelocityConstraintCppAd>(
            *referenceManagerPtr_, eeKinematics, contactPointIndex);
      }
    }

  } // namespace legged_robot
} // namespace ocs2
