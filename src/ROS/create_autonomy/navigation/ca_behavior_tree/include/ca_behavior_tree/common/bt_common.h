/**
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2020, Emiliano Borghi
 *
 */
#pragma once

namespace BT
{

// Custom type
struct Pose2D
{
  double x;
  double y;
  double theta;
};

template <> inline
BT::Pose2D convertFromString(StringView key)
{
  // three real numbers separated by semicolons
  auto parts = BT::splitString(key, ';');
  if (parts.size() != 3)
  {
    throw BT::RuntimeError("invalid input)");
  }
  else
  {
    BT::Pose2D output;
    output.x     = convertFromString<double>(parts[0]);
    output.y     = convertFromString<double>(parts[1]);
    output.theta = convertFromString<double>(parts[2]);
    return output;
  }
}

}  // namespace BT
