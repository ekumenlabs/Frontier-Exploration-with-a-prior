/**
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2020, Emiliano Borghi
 *
 */
#pragma once

#include <string>

#include <ros/ros.h>
#include <tf/transform_datatypes.h>

#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>

#include <behaviortree_cpp_v3/action_node.h>

#include "ca_behavior_tree/common/bt_common.h"


class MoveBase : public BT::AsyncActionNode
{
public:
  MoveBase(const std::string& name, const BT::NodeConfiguration& config)
    : BT::AsyncActionNode(name, config)
  {
  }

  // It is mandatory to define this static method.
  static BT::PortsList providedPorts()
  {
    return
    {
      BT::InputPort<BT::Pose2D>("goal"),
      BT::InputPort<std::string>("robot"),
    };
  }

  BT::NodeStatus tick() override;

  void halt() override
  {
    _aborted.store(true);
  }

private:
  typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
  std::atomic_bool _aborted;
};
