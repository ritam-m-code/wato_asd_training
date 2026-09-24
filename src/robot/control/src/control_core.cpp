#include <algorithm>
#include <cmath>
#include <limits>

#include "control_core.hpp"

namespace robot
{

ControlCore::ControlCore(const rclcpp::Logger& logger)
: logger_(logger),
  lookahead_distance_(1.5),
  linear_speed_(0.5),
  max_angular_(1.0),
  goal_tolerance_(0.5) {}

void ControlCore::configure(double lookahead_distance, double linear_speed,
                            double max_angular, double goal_tolerance)
{
  lookahead_distance_ = lookahead_distance;
  linear_speed_ = linear_speed;
  max_angular_ = max_angular;
  goal_tolerance_ = goal_tolerance;
  RCLCPP_INFO(logger_, "Control configured: lookahead %.2fm, speed %.2fm/s, max yaw %.2frad/s",
              lookahead_distance_, linear_speed_, max_angular_);
}

size_t ControlCore::closestPointIndex(const nav_msgs::msg::Path & path,
                                      double robot_x, double robot_y) const
{
  size_t best = 0;
  double best_distance = std::numeric_limits<double>::max();

  for (size_t i = 0; i < path.poses.size(); ++i) {
    const double dx = path.poses[i].pose.position.x - robot_x;
    const double dy = path.poses[i].pose.position.y - robot_y;
    const double distance = dx * dx + dy * dy;  // squared is enough for a comparison
    if (distance < best_distance) {
      best_distance = distance;
      best = i;
    }
  }

  return best;
}

geometry_msgs::msg::Twist ControlCore::computeCommand(const nav_msgs::msg::Path & path,
                                                     double robot_x, double robot_y,
                                                     double robot_yaw) const
{
  geometry_msgs::msg::Twist command;

  // An empty path means the planner has nothing for us, so stop. Leaving the
  // last command in place would let the robot coast on after arriving.
  if (path.poses.empty()) {
    return command;
  }

  const auto & final_pose = path.poses.back().pose.position;
  if (std::hypot(final_pose.x - robot_x, final_pose.y - robot_y) < goal_tolerance_) {
    return command;
  }

  // Walk forward from the nearest point to the first one at least a lookahead
  // away; if the path ends before that, chase its final point.
  const size_t start = closestPointIndex(path, robot_x, robot_y);
  size_t target = path.poses.size() - 1;
  for (size_t i = start; i < path.poses.size(); ++i) {
    const double dx = path.poses[i].pose.position.x - robot_x;
    const double dy = path.poses[i].pose.position.y - robot_y;
    if (std::hypot(dx, dy) >= lookahead_distance_) {
      target = i;
      break;
    }
  }

  const double dx = path.poses[target].pose.position.x - robot_x;
  const double dy = path.poses[target].pose.position.y - robot_y;

  // Rotate the target into the robot frame: +x is straight ahead, +y is left.
  const double cos_yaw = std::cos(robot_yaw);
  const double sin_yaw = std::sin(robot_yaw);
  const double local_y = -dx * sin_yaw + dy * cos_yaw;

  const double distance = std::hypot(dx, dy);
  if (distance < 1e-6) {
    return command;
  }

  // Curvature of the arc from the robot through the target point.
  const double curvature = 2.0 * local_y / (distance * distance);

  double angular = linear_speed_ * curvature;
  angular = std::max(-max_angular_, std::min(max_angular_, angular));

  // Ease off the throttle in hard turns so the robot does not swing wide.
  const double turn_fraction = std::abs(angular) / max_angular_;
  command.linear.x = linear_speed_ * (1.0 - 0.5 * turn_fraction);
  command.angular.z = angular;

  return command;
}

}
