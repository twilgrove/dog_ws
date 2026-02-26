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

// Pinocchio forward declarations must be included first
#include <pinocchio/fwd.hpp>

// Pinocchio
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/kinematics.hpp>

#include "ocs2_legged_robot/visualization/LeggedRobotVisualizer.h"

// OCS2
#include <ocs2_centroidal_model/AccessHelperFunctions.h>
#include <ocs2_core/misc/LinearInterpolation.h>
#include <ocs2_robotic_tools/common/RotationTransforms.h>
#include <ocs2_ros_interfaces/visualization/VisualizationHelpers.h>

#include "ocs2_legged_robot/gait/MotionPhaseDefinition.h"

// Additional messages not in the helpers file
#include <geometry_msgs/msg/pose_array.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

// URDF related
#include <urdf/model.h>

#include <kdl_parser/kdl_parser.hpp>

namespace ocs2
{
  namespace legged_robot
  {

    /******************************************************************************************************/
    // 构造函数：初始化发布者、动力学接口和频率控制参数
    LeggedRobotVisualizer::LeggedRobotVisualizer(
        PinocchioInterface pinocchioInterface,
        CentroidalModelInfo centroidalModelInfo,
        const PinocchioEndEffectorKinematics &endEffectorKinematics,
        const rclcpp::Node::SharedPtr &node,
        scalar_t maxUpdateFrequency)
        : node_(node),
          pinocchioInterface_(std::move(pinocchioInterface)),
          centroidalModelInfo_(std::move(centroidalModelInfo)),
          endEffectorKinematicsPtr_(endEffectorKinematics.clone()),
          tfBroadcaster_(node),
          lastTime_(std::numeric_limits<scalar_t>::lowest()),
          minPublishTimeDifference_(1.0 / maxUpdateFrequency)
    {

      endEffectorKinematicsPtr_->setPinocchioInterface(pinocchioInterface_);

      // 创建基座（质心）期望轨迹的发布者
      costDesiredBasePositionPublisher_ =
          node->create_publisher<visualization_msgs::msg::Marker>(
              "/legged_robot/desiredBaseTrajectory", 1);

      // 为每只脚创建足端期望轨迹的发布者
      costDesiredFeetPositionPublishers_.resize(
          centroidalModelInfo_.numThreeDofContacts);
      costDesiredFeetPositionPublishers_[0] =
          node->create_publisher<visualization_msgs::msg::Marker>(
              "/legged_robot/desiredFeetTrajectory/LF", 1); // 左前腿
      costDesiredFeetPositionPublishers_[1] =
          node->create_publisher<visualization_msgs::msg::Marker>(
              "/legged_robot/desiredFeetTrajectory/RF", 1); // 右前腿
      costDesiredFeetPositionPublishers_[2] =
          node->create_publisher<visualization_msgs::msg::Marker>(
              "/legged_robot/desiredFeetTrajectory/LH", 1); // 左后腿
      costDesiredFeetPositionPublishers_[3] =
          node->create_publisher<visualization_msgs::msg::Marker>(
              "/legged_robot/desiredFeetTrajectory/RH", 1); // 右后腿

      // 创建 MPC 优化后的状态轨迹发布者
      stateOptimizedPublisher_ =
          node->create_publisher<visualization_msgs::msg::MarkerArray>(
              "/legged_robot/optimizedStateTrajectory", 1);

      // 创建当前实时状态的发布者
      currentStatePublisher_ =
          node->create_publisher<visualization_msgs::msg::MarkerArray>(
              "/legged_robot/currentState", 1);

      // 创建标准的 ROS 关节状态发布者，用于驱动 URDF 模型显示
      jointPublisher_ =
          node_->create_publisher<sensor_msgs::msg::JointState>("joint_states", 1);
    };

    /******************************************************************************************************/
    // 核心更新函数：决定何时发布可视化数据
    void LeggedRobotVisualizer::update(const SystemObservation &observation,
                                       const PrimalSolution &primalSolution,
                                       const CommandData &command)
    {
      if (observation.time - lastTime_ > minPublishTimeDifference_)
      {
        const auto &model = pinocchioInterface_.getModel();
        auto &data = pinocchioInterface_.getData();

        // 核心动力学计算：利用 Pinocchio 执行正运动学（Forward Kinematics）
        // 将状态向量（observation.state）转换为广义坐标（位置、姿态、关节角）
        pinocchio::forwardKinematics(model, data,
                                     centroidal_model::getGeneralizedCoordinates(
                                         observation.state, centroidalModelInfo_));
        // 更新每一个 Link（连杆）在 3D 空间中的位置
        pinocchio::updateFramePlacements(model, data);

        const auto timeStamp = node_->get_clock()->now(); // 获取当前 ROS 时间戳

        // 分别发布：当前观察值、期望指令、以及优化后的预测轨迹
        publishObservation(timeStamp, observation);
        publishDesiredTrajectory(timeStamp, command.mpcTargetTrajectories_);
        publishOptimizedStateTrajectory(timeStamp, primalSolution.timeTrajectory_,
                                        primalSolution.stateTrajectory_,
                                        primalSolution.modeSchedule_);

        lastTime_ = observation.time; // 更新上次发布时间
      }
    }

    /******************************************************************************************************/
    // 发布实时观测值：将机器人当前的状态（位置、受力等）发送到 RViz
    void LeggedRobotVisualizer::publishObservation(
        rclcpp::Time timeStamp, const SystemObservation &observation)
    {
      // 1. 从状态向量中提取基座（Base）的 6 自由度位姿（位置+四元数）
      const auto basePose =
          centroidal_model::getBasePose(observation.state, centroidalModelInfo_);

      // 2. 从状态向量中提取 12 个关节的当前角度
      const auto qJoints =
          centroidal_model::getJointAngles(observation.state, centroidalModelInfo_);

      // 3. 计算足端在笛卡尔空间的位置
      const auto feetPositions =
          endEffectorKinematicsPtr_->getPosition(observation.state);

      // 4. 提取足端的地面反作用力（GRF），用于在 RViz 里画受力箭头
      std::vector<vector3_t> feetForces(centroidalModelInfo_.numThreeDofContacts);
      for (size_t i = 0; i < centroidalModelInfo_.numThreeDofContacts; i++)
      {
        feetForces[i] = centroidal_model::getContactForces(observation.input, i,
                                                           centroidalModelInfo_);
      }

      // 5. 执行具体的发布动作：关节角、基座坐标系变换、以及脚部标记点
      publishJointTransforms(timeStamp, qJoints);
      publishBaseTransform(timeStamp, basePose);
      publishCartesianMarkers(timeStamp, modeNumber2StanceLeg(observation.mode),
                              feetPositions, feetForces);
    }

    /******************************************************************************************************/
    // 发布关节变换：构造并发送 JointState 消息
    void LeggedRobotVisualizer::publishJointTransforms(
        rclcpp::Time timeStamp, const vector_t &jointAngles) const
    {
      if (jointPublisher_ != nullptr) // 确保发布者已初始化
      {
        sensor_msgs::msg::JointState joint_state;
        joint_state.header.stamp = node_->get_clock()->now(); // 设置时间戳

        // 针对 12 自由度四足机器人初始化名称和位置数组
        joint_state.name.resize(12);
        joint_state.position.resize(12);

        // 关节命名（需与 URDF 文件中的名称完全一致）
        // 顺序为：左前(LF), 左后(LH), 右前(RF), 右后(RH)
        joint_state.name[0] = "LF_HAA"; // 左前侧摆
        joint_state.name[1] = "LF_HFE"; // 左前航向
        joint_state.name[2] = "LF_KFE"; // 左前膝盖
        joint_state.name[3] = "LH_HAA";
        joint_state.name[4] = "LH_HFE";
        joint_state.name[5] = "LH_KFE";
        joint_state.name[6] = "RF_HAA";
        joint_state.name[7] = "RF_HFE";
        joint_state.name[8] = "RF_KFE";
        joint_state.name[9] = "RH_HAA";
        joint_state.name[10] = "RH_HFE";
        joint_state.name[11] = "RH_KFE";

        // 填充对应的关节角度值
        joint_state.position[0] = jointAngles[0];
        joint_state.position[1] = jointAngles[1];
        joint_state.position[2] = jointAngles[2];
        joint_state.position[3] = jointAngles[3];
        joint_state.position[4] = jointAngles[4];
        joint_state.position[5] = jointAngles[5];
        joint_state.position[6] = jointAngles[6];
        joint_state.position[7] = jointAngles[7];
        joint_state.position[8] = jointAngles[8];
        joint_state.position[9] = jointAngles[9];
        joint_state.position[10] = jointAngles[10];
        joint_state.position[11] = jointAngles[11];

        // 发布关节消息，RViz 中的机器人模型会根据此消息转动关节
        jointPublisher_->publish(joint_state);
      }
    }

    /******************************************************************************************************/
    // 发布基座（Base）的坐标变换：将机器人在世界坐标系中的位姿通过 TF 广播出去
    void LeggedRobotVisualizer::publishBaseTransform(rclcpp::Time timeStamp,
                                                     const vector_t &basePose)
    {
      if (jointPublisher_ != nullptr) // 这里的判断其实是为了确保节点环境正常
      {
        geometry_msgs::msg::TransformStamped baseToWorldTransform;
        // 设置 Header：包括 frame_id（通常是 odom）和时间戳
        baseToWorldTransform.header = getHeaderMsg(frameId_, timeStamp);
        // 子坐标系设为 base（需与 URDF 中的根 link 对应）
        baseToWorldTransform.child_frame_id = "base_link";

        // 将欧拉角（ZYX顺序，取自状态向量末尾3位）转换为 Eigen 四元数
        const Eigen::Quaternion<scalar_t> q_world_base =
            getQuaternionFromEulerAnglesZyx(vector3_t(basePose.tail<3>()));

        // 填充旋转数据（四元数）
        baseToWorldTransform.transform.rotation = getOrientationMsg(q_world_base);
        // 填充平移数据（位置 X, Y, Z，取自状态向量前3位）
        baseToWorldTransform.transform.translation =
            getVectorMsg(basePose.head<3>());

        // 广播 TF 变换，这样 RViz 里的机器人模型才能跟着坐标系走
        tfBroadcaster_.sendTransform(baseToWorldTransform);
      }
    }

    /******************************************************************************************************/
    // 轨迹回放函数：将一段存储好的观察序列按指定速度在 RViz 里“放电影”
    void LeggedRobotVisualizer::publishTrajectory(
        const std::vector<SystemObservation> &system_observation_array,
        scalar_t speed)
    {
      for (size_t k = 0; k < system_observation_array.size() - 1; k++)
      {
        // 计算两帧之间理论上应该等待的时间（受 speed 缩放影响）
        scalar_t frameDuration = speed * (system_observation_array[k + 1].time -
                                          system_observation_array[k].time);

        // 执行当前帧的发布，并测量发布动作本身消耗的时间（timedExecutionInSeconds）
        scalar_t publishDuration = timedExecutionInSeconds([&]()
                                                           { publishObservation(node_->get_clock()->now(),
                                                                                system_observation_array[k]); });

        // 如果理论间隔大于实际消耗时间，则进入休眠，以维持回放速率
        if (frameDuration > publishDuration)
        {
          const rclcpp::Duration duration =
              rclcpp::Duration::from_seconds(frameDuration - publishDuration);
          rclcpp::sleep_for((std::chrono::nanoseconds(duration.nanoseconds())));
        }
      }
    }

    /******************************************************************************************************/
    // 发布笛卡尔空间标记：包括脚部球体、力矢量箭头、压力中心(CoP)和支撑多边形
    void LeggedRobotVisualizer::publishCartesianMarkers(
        rclcpp::Time timeStamp, const contact_flag_t &contactFlags,
        const std::vector<vector3_t> &feetPositions,
        const std::vector<vector3_t> &feetForces) const
    {
      // 预留空间：4只脚 + 4个力箭头 + 1个CoP + 1个支撑多边形 = 10
      const size_t numberOfCartesianMarkers = 10;
      visualization_msgs::msg::MarkerArray markerArray;
      markerArray.markers.reserve(numberOfCartesianMarkers);

      // 遍历每只脚，生成足端位置标记和地面反作用力箭头
      for (size_t i = 0; i < centroidalModelInfo_.numThreeDofContacts; ++i)
      {
        // 足端球体：触地时实心，摆动时半透明
        markerArray.markers.emplace_back(
            getFootMarker(feetPositions[i], contactFlags[i], feetColorMap_[i],
                          footMarkerDiameter_, footAlphaWhenLifted_));
        // 力矢量：绿色的箭头，长度与力的大小成正比
        markerArray.markers.emplace_back(
            getForceMarker(feetForces[i], feetPositions[i], contactFlags[i],
                           Color::green, forceScale_));
      }

      // 计算并生成压力中心 (CoP) 标记：显示所有触地脚受力的合力中心
      markerArray.markers.emplace_back(getCenterOfPressureMarker(
          feetForces.begin(), feetForces.end(), feetPositions.begin(),
          contactFlags.begin(), Color::green, copMarkerDiameter_));

      // 生成支撑多边形：将触地的脚连成线，显示当前的稳定支撑区域
      markerArray.markers.emplace_back(getSupportPolygonMarker(
          feetPositions.begin(), feetPositions.end(), contactFlags.begin(),
          Color::black, supportPolygonLineWidth_));

      // 统一分配 Header（坐标系和时间戳）以及递增的 ID
      assignHeader(markerArray.markers.begin(), markerArray.markers.end(),
                   getHeaderMsg(frameId_, timeStamp));
      assignIncreasingId(markerArray.markers.begin(), markerArray.markers.end());

      // 发布当前的 MarkerArray
      currentStatePublisher_->publish(markerArray);
    }

    /******************************************************************************************************/
    // 发布期望轨迹：在 RViz 中绘制参考命令给出的路径线（包括机身和四只脚）
    void LeggedRobotVisualizer::publishDesiredTrajectory(
        rclcpp::Time timeStamp, const TargetTrajectories &targetTrajectories)
    {
      const auto &stateTrajectory = targetTrajectories.stateTrajectory;
      const auto &inputTrajectory = targetTrajectories.inputTrajectory;

      // 预留机身轨迹消息空间
      std::vector<geometry_msgs::msg::Point> desiredBasePositionMsg;
      desiredBasePositionMsg.reserve(stateTrajectory.size());

      // 预留四只脚轨迹消息空间
      feet_array_t<std::vector<geometry_msgs::msg::Point>> desiredFeetPositionMsgs;
      for (size_t i = 0; i < centroidalModelInfo_.numThreeDofContacts; i++)
      {
        desiredFeetPositionMsgs[i].reserve(stateTrajectory.size());
      }

      // 遍历整个期望轨迹的时间序列
      for (size_t j = 0; j < stateTrajectory.size(); j++)
      {
        const auto state = stateTrajectory.at(j);
        vector_t input(centroidalModelInfo_.inputDim);
        if (j < inputTrajectory.size())
        {
          input = inputTrajectory.at(j);
        }
        else
        {
          input.setZero();
        }

        // 提取并填充参考的基座位置
        const auto basePose =
            centroidal_model::getBasePose(state, centroidalModelInfo_);
        geometry_msgs::msg::Pose pose;
        pose.position = getPointMsg(basePose.head<3>());
        desiredBasePositionMsg.push_back(pose.position);

        // 核心：为了画参考腿部轨迹，必须再次对参考状态执行正运动学计算
        const auto &model = pinocchioInterface_.getModel();
        auto &data = pinocchioInterface_.getData();
        pinocchio::forwardKinematics(model, data,
                                     centroidal_model::getGeneralizedCoordinates(
                                         state, centroidalModelInfo_));
        pinocchio::updateFramePlacements(model, data);

        // 获取参考状态下的足端位置并存入容器
        const auto feetPositions = endEffectorKinematicsPtr_->getPosition(state);
        for (size_t i = 0; i < centroidalModelInfo_.numThreeDofContacts; i++)
        {
          geometry_msgs::msg::Pose footPose;
          footPose.position = getPointMsg(feetPositions[i]);
          desiredFeetPositionMsgs[i].push_back(footPose.position);
        }
      }

      // 构造机身期望路径线（绿色）
      auto comLineMsg = getLineMsg(std::move(desiredBasePositionMsg), Color::green,
                                   trajectoryLineWidth_);
      comLineMsg.header = getHeaderMsg(frameId_, timeStamp);
      comLineMsg.id = 0;

      // 发布机身参考线
      costDesiredBasePositionPublisher_->publish(comLineMsg);

      // 分别发布四只脚的参考线（对应各自的颜色）
      for (size_t i = 0; i < centroidalModelInfo_.numThreeDofContacts; i++)
      {
        auto footLineMsg = getLineMsg(std::move(desiredFeetPositionMsgs[i]),
                                      feetColorMap_[i], trajectoryLineWidth_);
        footLineMsg.header = getHeaderMsg(frameId_, timeStamp);
        footLineMsg.id = 0;
        costDesiredFeetPositionPublishers_[i]->publish(footLineMsg);
      }
    }
    /******************************************************************************************************/
    // 发布优化后的状态轨迹：显示 MPC 在预测周期（Horizon）内算出的未来路径
    void LeggedRobotVisualizer::publishOptimizedStateTrajectory(
        rclcpp::Time timeStamp, const scalar_array_t &mpcTimeTrajectory,
        const vector_array_t &mpcStateTrajectory,
        const ModeSchedule &modeSchedule)
    {
      // 判空保护：如果时间序列或状态序列为空，直接返回
      if (mpcTimeTrajectory.empty() || mpcStateTrajectory.empty())
      {
        return; // 无内容可发布
      }

      // 1. 为四只脚的轨迹消息预留空间
      feet_array_t<std::vector<geometry_msgs::msg::Point>> feetMsgs;
      std::for_each(feetMsgs.begin(), feetMsgs.end(),
                    [&](std::vector<geometry_msgs::msg::Point> &v)
                    {
                      v.reserve(mpcStateTrajectory.size()); // 预留与轨迹长度一致的空间
                    });

      // 2. 为质心 (CoM) 轨迹消息预留空间
      std::vector<geometry_msgs::msg::Point> mpcComPositionMsgs;
      mpcComPositionMsgs.reserve(mpcStateTrajectory.size());

      // 3. 核心提取循环：遍历整个预测轨迹中的每一个状态点
      std::for_each(
          mpcStateTrajectory.begin(), mpcStateTrajectory.end(),
          [&](const vector_t &state)
          {
            // 提取该时刻的基座（机身）位姿
            const auto basePose =
                centroidal_model::getBasePose(state, centroidalModelInfo_);

            // 填充机身位置点
            geometry_msgs::msg::Pose pose;
            pose.position = getPointMsg(basePose.head<3>());
            mpcComPositionMsgs.push_back(pose.position);

            // 填充足端位置：这里必须为轨迹上的每一个点重新跑一遍 Pinocchio 正运动学
            const auto &model = pinocchioInterface_.getModel();
            auto &data = pinocchioInterface_.getData();
            pinocchio::forwardKinematics(
                model, data,
                centroidal_model::getGeneralizedCoordinates(state,
                                                            centroidalModelInfo_));
            pinocchio::updateFramePlacements(model, data);

            // 获取该状态下的足端坐标
            const auto feetPositions =
                endEffectorKinematicsPtr_->getPosition(state);
            for (size_t i = 0; i < centroidalModelInfo_.numThreeDofContacts; i++)
            {
              const auto position = getPointMsg(feetPositions[i]);
              feetMsgs[i].push_back(position);
            }
          });

      // 4. 将提取出的足端点序列转换为 MarkerArray（线带形式）
      visualization_msgs::msg::MarkerArray markerArray;
      // 预留空间：4条足端线 + 1条机身红线 + 1组落脚点球体
      markerArray.markers.reserve(centroidalModelInfo_.numThreeDofContacts + 2);

      for (size_t i = 0; i < centroidalModelInfo_.numThreeDofContacts; i++)
      {
        // 创建足端轨迹线，颜色与脚对应
        markerArray.markers.emplace_back(getLineMsg(
            std::move(feetMsgs[i]), feetColorMap_[i], trajectoryLineWidth_));
        markerArray.markers.back().ns = "EE Trajectories"; // 命名空间：末端轨迹
      }

      // 创建机身质心轨迹线（固定为红色）
      markerArray.markers.emplace_back(getLineMsg(
          std::move(mpcComPositionMsgs), Color::red, trajectoryLineWidth_));
      markerArray.markers.back().ns = "CoM Trajectory";

      // 5. 绘制未来的落脚点 (Future Footholds)：显示大脑预判的脚会踩在哪
      visualization_msgs::msg::Marker sphereList;
      sphereList.type = visualization_msgs::msg::Marker::SPHERE_LIST; // 使用球体列表模式
      sphereList.scale.x = footMarkerDiameter_;
      sphereList.scale.y = footMarkerDiameter_;
      sphereList.scale.z = footMarkerDiameter_;
      sphereList.ns = "Future footholds";
      sphereList.pose.orientation = getOrientationMsg({1., 0., 0., 0.});

      const auto &eventTimes = modeSchedule.eventTimes;          // 获取步态切换的时间点（如 0.2s 后抬脚）
      const auto &subsystemSequence = modeSchedule.modeSequence; // 获取步态序列（如 支撑->摆动->支撑）
      const auto tStart = mpcTimeTrajectory.front();             // 预测窗口开始时间
      const auto tEnd = mpcTimeTrajectory.back();                // 预测窗口结束时间

      // 遍历所有预测的步态切换事件
      for (size_t event = 0; event < eventTimes.size(); ++event)
      {
        // 只处理发生在当前预测周期（Horizon）内的事件
        if (tStart < eventTimes[event] &&
            eventTimes[event] < tEnd)
        {
          const auto preEventContactFlags =
              modeNumber2StanceLeg(subsystemSequence[event]); // 事件前的触地状态
          const auto postEventContactFlags =
              modeNumber2StanceLeg(subsystemSequence[event + 1]); // 事件后的触地状态

          // 在事件发生的精确时间点进行状态插值，计算那一瞬间的机器人姿态
          const auto postEventState = LinearInterpolation::interpolate(
              eventTimes[event], mpcTimeTrajectory, mpcStateTrajectory);

          // 计算插值状态下的正运动学
          const auto &model = pinocchioInterface_.getModel();
          auto &data = pinocchioInterface_.getData();
          pinocchio::forwardKinematics(model, data,
                                       centroidal_model::getGeneralizedCoordinates(
                                           postEventState, centroidalModelInfo_));
          pinocchio::updateFramePlacements(model, data);

          const auto feetPosition =
              endEffectorKinematicsPtr_->getPosition(postEventState);

          for (size_t i = 0; i < centroidalModelInfo_.numThreeDofContacts; i++)
          {
            // 如果某只脚从“不触地”变成“触地”（即着地瞬间 landing）
            if (!preEventContactFlags[i] &&
                postEventContactFlags[i])
            {
              // 在该坐标处添加一个球体标记，代表未来的落脚点
              sphereList.points.emplace_back(getPointMsg(feetPosition[i]));
              sphereList.colors.push_back(getColor(feetColorMap_[i]));
            }
          }
        }
      }
      markerArray.markers.push_back(std::move(sphereList));

      // 6. 为所有 Marker 添加 Header（坐标系和时间戳）并分配递增 ID
      assignHeader(markerArray.markers.begin(), markerArray.markers.end(),
                   getHeaderMsg(frameId_, timeStamp));
      assignIncreasingId(markerArray.markers.begin(), markerArray.markers.end());

      // 7. 发送到 RViz
      stateOptimizedPublisher_->publish(markerArray);
    }
  } // namespace legged_robot
} // namespace ocs2
