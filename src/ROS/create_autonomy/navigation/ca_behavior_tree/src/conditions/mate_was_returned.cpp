/**
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2021, Nicolas Bortoni
 *
 */
#include <ca_behavior_tree/conditions/mate_was_returned.h>

int MateWasReturned::i{ 0 };

BT::NodeStatus MateWasReturned::tick()
{
  if (i <= 3)
  {
    i++;
    return BT::NodeStatus::FAILURE;
  }
  i = 0;
  return BT::NodeStatus::SUCCESS;
}

BT::PortsList MateWasReturned::providedPorts()
{
  return
  {
  };
}
