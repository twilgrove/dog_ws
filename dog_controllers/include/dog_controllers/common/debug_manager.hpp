#pragma once

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <mutex>
#include <memory>

class DebugManager final
{
public:
    DebugManager(rclcpp_lifecycle::LifecycleNode::SharedPtr &node);

    void update();

private:
    void publish();

    rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
};
