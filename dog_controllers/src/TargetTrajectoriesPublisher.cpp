#include "legged_controllers/TargetTrajectoriesPublisher.hpp"
#include <ocs2_core/misc/LoadData.h>
#include <ocs2_robotic_tools/common/RotationTransforms.h>

namespace dog_controllers
{

  TargetTrajectoriesPublisher::TargetTrajectoriesPublisher(const rclcpp::NodeOptions &options)
      : Node("target_trajectories_publisher", options), DEFAULT_JOINT_STATE_(12)
  {

    // 1. 获取参数文件路径
    std::string topicPrefix = "dog_robot";
    std::string pkg_share_path = ament_index_cpp::get_package_share_directory("dog_bringup");
    std::string taskFile = pkg_share_path + "/config/description/task.info";
    std::string referenceFile = pkg_share_path + "/config/description/reference.info";

    // 2. 加载 .info 数据
    loadData::loadCppDataType(referenceFile, "comHeight", COM_HEIGHT_);
    loadData::loadEigenMatrix(referenceFile, "defaultJointState", DEFAULT_JOINT_STATE_);
    loadData::loadCppDataType(referenceFile, "targetRotationVelocity", TARGET_ROTATION_VELOCITY_);
    loadData::loadCppDataType(referenceFile, "targetDisplacementVelocity", TARGET_DISPLACEMENT_VELOCITY_);
    loadData::loadCppDataType(taskFile, "mpc.timeHorizon", TIME_TO_TARGET_);

    // 3. 初始化 TF2
    tfBuffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tfListener_ = std::make_shared<tf2_ros::TransformListener>(*tfBuffer_);

    // 4. 初始化 OCS2 发布器
    targetTrajectoriesPublisher_ = std::make_unique<TargetTrajectoriesRosPublisher>(
        this->shared_from_this(), topicPrefix);

    // 5. 订阅 Observation
    observationSub_ = this->create_subscription<ocs2_msgs::msg::MpcObservation>(
        topicPrefix + "_mpc_observation", 1,
        [this](const ocs2_msgs::msg::MpcObservation::SharedPtr msg)
        {
          std::lock_guard<std::mutex> lock(latestObservationMutex_);
          latestObservation_ = ros_msg_conversions::readObservationMsg(*msg);
        });

    // 6. 订阅 Goal (Nav2 默认话题通常是 /goal_pose)
    goalSub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "/goal_pose", 1,
        [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg)
        {
          if (latestObservation_.time == 0.0)
            return;

          geometry_msgs::msg::PoseStamped poseInOdom;
          try
          {
            poseInOdom = tfBuffer_->transform(*msg, "odom", tf2::durationFromSec(0.2));
          }
          catch (const tf2::TransformException &ex)
          {
            RCLCPP_WARN(this->get_logger(), "TF Transform failed: %s", ex.what());
            return;
          }

          vector_t cmdGoal = vector_t::Zero(6);
          cmdGoal[0] = poseInOdom.pose.position.x;
          cmdGoal[1] = poseInOdom.pose.position.y;
          cmdGoal[2] = poseInOdom.pose.position.z;

          Eigen::Quaternion<scalar_t> q(
              poseInOdom.pose.orientation.w, poseInOdom.pose.orientation.x,
              poseInOdom.pose.orientation.y, poseInOdom.pose.orientation.z);
          // 提取 Yaw
          cmdGoal[3] = getRotationMatrixFromQuaternion(q).eulerAngles(0, 1, 2).z();

          const auto trajectories = goalToTargetTrajectories(cmdGoal, latestObservation_);
          targetTrajectoriesPublisher_->publishTargetTrajectories(trajectories);
        });

    // 7. 订阅 cmd_vel
    cmdVelSub_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "/cmd_vel", 1,
        [this](const geometry_msgs::msg::Twist::SharedPtr msg)
        {
          if (latestObservation_.time == 0.0)
            return;

          vector_t cmdVel = vector_t::Zero(4);
          cmdVel[0] = msg->linear.x;
          cmdVel[1] = msg->linear.y;
          cmdVel[2] = msg->linear.z;
          cmdVel[3] = msg->angular.z;

          const auto trajectories = cmdVelToTargetTrajectories(cmdVel, latestObservation_);
          targetTrajectoriesPublisher_->publishTargetTrajectories(trajectories);
        });
  }

  scalar_t TargetTrajectoriesPublisher::estimateTimeToTarget(const vector_t &desiredBaseDisplacement)
  {
    const scalar_t rotationTime = std::abs(desiredBaseDisplacement(3)) / TARGET_ROTATION_VELOCITY_;
    const scalar_t displacement = desiredBaseDisplacement.head<2>().norm();
    const scalar_t displacementTime = displacement / TARGET_DISPLACEMENT_VELOCITY_;
    return std::max(rotationTime, displacementTime);
  }

  TargetTrajectories TargetTrajectoriesPublisher::targetPoseToTargetTrajectories(
      const vector_t &targetPose, const SystemObservation &observation, const scalar_t &targetReachingTime)
  {

    const scalar_array_t timeTrajectory{observation.time, targetReachingTime};

    vector_t currentPose = observation.state.segment<6>(6);
    currentPose(2) = COM_HEIGHT_;
    currentPose(4) = 0;
    currentPose(5) = 0;

    vector_array_t stateTrajectory(2, vector_t::Zero(observation.state.size()));
    stateTrajectory[0] << vector_t::Zero(6), currentPose, DEFAULT_JOINT_STATE_;
    stateTrajectory[1] << vector_t::Zero(6), targetPose, DEFAULT_JOINT_STATE_;

    const vector_array_t inputTrajectory(2, vector_t::Zero(observation.input.size()));
    return {timeTrajectory, stateTrajectory, inputTrajectory};
  }

  TargetTrajectories TargetTrajectoriesPublisher::goalToTargetTrajectories(const vector_t &goal, const SystemObservation &observation)
  {
    const vector_t currentPose = observation.state.segment<6>(6);
    vector_t targetPose = vector_t::Zero(6);
    targetPose << goal(0), goal(1), COM_HEIGHT_, goal(3), 0, 0;

    const scalar_t targetReachingTime = observation.time + estimateTimeToTarget(targetPose - currentPose);
    return targetPoseToTargetTrajectories(targetPose, observation, targetReachingTime);
  }

  TargetTrajectories TargetTrajectoriesPublisher::cmdVelToTargetTrajectories(const vector_t &cmdVel, const SystemObservation &observation)
  {
    const vector_t currentPose = observation.state.segment<6>(6);
    const Eigen::Matrix<scalar_t, 3, 1> zyx = currentPose.tail(3);
    vector_t cmdVelRot = getRotationMatrixFromZyxEulerAngles(zyx) * cmdVel.head(3);

    const scalar_t timeToTarget = TIME_TO_TARGET_;
    vector_t targetPose = vector_t::Zero(6);
    targetPose << currentPose(0) + cmdVelRot(0) * timeToTarget,
        currentPose(1) + cmdVelRot(1) * timeToTarget,
        COM_HEIGHT_,
        currentPose(3) + cmdVel(3) * timeToTarget, 0, 0;

    const scalar_t targetReachingTime = observation.time + timeToTarget;
    auto trajectories = targetPoseToTargetTrajectories(targetPose, observation, targetReachingTime);

    trajectories.stateTrajectory[0].head(3) = cmdVelRot;
    trajectories.stateTrajectory[1].head(3) = cmdVelRot;
    return trajectories;
  }

}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(dog_controllers::TargetTrajectoriesPublisher)