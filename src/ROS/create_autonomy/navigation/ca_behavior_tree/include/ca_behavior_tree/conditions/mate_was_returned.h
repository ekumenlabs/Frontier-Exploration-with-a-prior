/**
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2021, Nicolas Bortoni
 *
 */
#pragma once

#include <string>

#include <behaviortree_cpp_v3/bt_factory.h>
#include <behaviortree_cpp_v3/condition_node.h>

#include <ros/ros.h>
#include <std_msgs/Float32.h>


/**
 * This condition is mocking a check to detect when a mate was detected.
 */
class MateWasReturned : public BT::ConditionNode
{
private:
  static int i;
public:
  MateWasReturned(const std::string& name, const BT::NodeConfiguration& config)
    : BT::ConditionNode::ConditionNode(name, config) {}
  ~MateWasReturned(){}
  static BT::PortsList providedPorts();
  BT::NodeStatus tick() override;
};
