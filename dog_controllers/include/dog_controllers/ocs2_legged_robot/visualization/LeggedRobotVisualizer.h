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
#pragma once

// OCS2 与机器人动力学相关依赖
#include <ocs2_centroidal_model/CentroidalModelInfo.h>
#include <ocs2_core/Types.h>
#include <ocs2_legged_robot/common/Types.h>
#include <ocs2_pinocchio_interface/PinocchioEndEffectorKinematics.h>
#include <ocs2_ros_interfaces/mrt/DummyObserver.h>
#include <ocs2_ros_interfaces/visualization/VisualizationColors.h>
#include <tf2_ros/transform_broadcaster.h>

// ROS 2 标准接口
#include <robot_state_publisher/robot_state_publisher.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include "rclcpp/rclcpp.hpp"

namespace ocs2
{
    namespace legged_robot
    {
        class LeggedRobotVisualizer : public DummyObserver
        {
        public:
            /* --- 可视化配置参数 --- */
            std::string frameId_ = "odom";             // 所有消息发布的坐标系，通常是全球坐标系 odom 或 world
            scalar_t footMarkerDiameter_ = 0.03;       // RViz 中足端球体的大小
            scalar_t footAlphaWhenLifted_ = 0.3;       // 摆动腿（抬起时）的透明度
            scalar_t forceScale_ = 1000.0;             // 接触力矢量箭头的缩放比例 (N/m)
            scalar_t velScale_ = 5.0;                  // 速度矢量箭头的缩放比例
            scalar_t copMarkerDiameter_ = 0.03;        // 压力中心 (CoP) 球体的大小
            scalar_t supportPolygonLineWidth_ = 0.005; // 支撑多边形连线的粗细
            scalar_t trajectoryLineWidth_ = 0.01;      // 预测轨迹线的粗细

            // 四只脚对应的颜色：蓝、橙、黄、紫
            std::vector<Color> feetColorMap_ = {
                Color::blue, Color::orange, Color::yellow, Color::purple};

            LeggedRobotVisualizer(
                PinocchioInterface pinocchioInterface,
                CentroidalModelInfo centroidalModelInfo,
                const PinocchioEndEffectorKinematics &endEffectorKinematics,
                const rclcpp::Node::SharedPtr &node,
                scalar_t maxUpdateFrequency = 100.0);

            ~LeggedRobotVisualizer() override = default;

            void update(const SystemObservation &observation,
                        const PrimalSolution &primalSolution,
                        const CommandData &command) override;

            // 发布一组历史观察序列（用于轨迹回放）
            void publishTrajectory(
                const std::vector<SystemObservation> &system_observation_array,
                scalar_t speed = 1.0);

            // 发布当前观察值：驱动 RViz 里的机器人实体模型移动
            void publishObservation(rclcpp::Time timeStamp,
                                    const SystemObservation &observation);

            // 发布期望轨迹
            void publishDesiredTrajectory(rclcpp::Time timeStamp,
                                          const TargetTrajectories &targetTrajectories);

            // 发布最优状态轨迹
            void publishOptimizedStateTrajectory(rclcpp::Time timeStamp,
                                                 const scalar_array_t &mpcTimeTrajectory,
                                                 const vector_array_t &mpcStateTrajectory,
                                                 const ModeSchedule &modeSchedule);

        protected:
            rclcpp::Node::SharedPtr node_;

        private:
            LeggedRobotVisualizer(const LeggedRobotVisualizer &) = delete; // 禁止拷贝

            // 发布关节状态 (JointState)，用于 URDF 模型渲染
            void publishJointTransforms(rclcpp::Time timeStamp, const vector_t &jointAngles) const;

            // 发布基座 (Base) 的位姿 TF 变换
            void publishBaseTransform(rclcpp::Time timeStamp, const vector_t &basePose);

            // 发布笛卡尔空间的标记点（足端位置、受力箭头等）
            void publishCartesianMarkers(rclcpp::Time timeStamp,
                                         const contact_flag_t &contactFlags,
                                         const std::vector<vector3_t> &feetPositions,
                                         const std::vector<vector3_t> &feetForces) const;

            /* --- 内部核心组件 --- */
            PinocchioInterface pinocchioInterface_;
            const CentroidalModelInfo centroidalModelInfo_;
            std::unique_ptr<PinocchioEndEffectorKinematics> endEffectorKinematicsPtr_;

            tf2_ros::TransformBroadcaster tfBroadcaster_;
            rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr jointPublisher_;

            // 各种可视化 Marker 的发布者
            rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr costDesiredBasePositionPublisher_;
            std::vector<rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr> costDesiredFeetPositionPublishers_;
            rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr stateOptimizedPublisher_;
            rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr currentStatePublisher_;

            scalar_t lastTime_;
            scalar_t minPublishTimeDifference_;
        };

    } // namespace legged_robot
} // namespace ocs2