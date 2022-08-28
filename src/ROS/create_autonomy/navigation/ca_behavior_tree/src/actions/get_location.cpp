/**
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2021, Nicolas Bortoni
 *
 */

#include "ca_behavior_tree/actions/get_location.h"


const std::vector<BT::Pose2D> GetLocation::desk_locations_ {
  {  -7.0,  5.9, 3.14 },
  {  -8.8,  5.9, 3.14 },
  { -10.5,  5.8, 3.14 },
  {  -8.2,  9.1, 3.14 },
  { -10.0,  9.1, 3.14 },
  { -11.7,  9.1, 3.14 },
  {  -7.0, 10.4, 3.14 },
  {  -8.8, 10.4, 3.14 },
  { -10.5, 10.4, 3.14 },
};

GetLocation::GetLocation(const std::string& name, const BT::NodeConfiguration& config)
  : BT::AsyncActionNode(name, config)
{
  // This subscriber receives a message from the RoboMate UI
  sub_ = nh_.subscribe<std_msgs::Float32>("/robomate/ui", 1,
    [this](const std_msgs::Float32::ConstPtr& msg)
    {
      std::lock_guard<std::mutex> lock{ mutex_ };
      const uint32_t request_id = static_cast<uint32_t>(msg->data);
      if (msg->data > 0) list_.push(request_id);
    });
}

BT::NodeStatus GetLocation::tick()
{
  if( list_.empty() ) return BT::NodeStatus::FAILURE;

  BT::Pose2D next_location;
  {
    std::lock_guard<std::mutex> lock{ mutex_ };
    try {
      next_location = desk_locations_.at(list_.front()-1);
      list_.pop();
    } catch(const std::out_of_range& e) {
      ROS_ERROR_STREAM("Desk index out of range: " << e.what());
      return BT::NodeStatus::FAILURE;
    }

  }

  ROS_INFO_STREAM("Sending robot to desk at pose: [" <<
    next_location.x << ", " <<
    next_location.y << ", " <<
    next_location.theta << "]"
  );
  setOutput("NextLocation", next_location);

  return BT::NodeStatus::SUCCESS;
}
