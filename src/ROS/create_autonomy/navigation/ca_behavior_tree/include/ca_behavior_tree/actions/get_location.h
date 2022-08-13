/**
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2021, Nicolas Bortoni
 *
 */
#pragma once

#include <queue>
#include <stdexcept>
#include <string>
#include <vector>

#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <behaviortree_cpp_v3/action_node.h>

#include "ca_behavior_tree/common/bt_common.h"


class GetLocation : public BT::AsyncActionNode
{
  private:
    std::queue<uint32_t> list_;
    static const std::vector<BT::Pose2D> desk_locations_;
    ros::NodeHandle nh_;
    ros::Subscriber sub_;
    std::mutex mutex_;
  public:
    GetLocation(const std::string& name, const BT::NodeConfiguration& config);
    ~GetLocation() {};
    static BT::PortsList providedPorts()
    {
      return
      {
        BT::OutputPort<BT::Pose2D>("NextLocation"),
      };
    }

    BT::NodeStatus tick() override;
};
