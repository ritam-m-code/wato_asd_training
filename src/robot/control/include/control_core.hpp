#ifndef CONTROL_CORE_HPP_
#define CONTROL_CORE_HPP_

#include <cstddef>

#include "rclcpp/rclcpp.hpp"

#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/path.hpp"

namespace robot
{

// Pure pursuit path follower.
//
// The controller picks a point a fixed distance ahead on the path and steers
// along the circular arc that connects the robot to it. The lookahead distance
// is the only real tuning knob: short tracks the path tightly but oscillates,
// long is smooth but cuts corners.
class ControlCore {
  public:
    // Constructor, we pass in the node's RCLCPP logger to enable logging to terminal
    ControlCore(const rclcpp::Logger& logger);

    void configure(double lookahead_distance, double linear_speed,
                   double max_angular, double goal_tolerance);

    // Produces the velocity command for the robot's current pose. Returns a zero
    // twist when the path is empty or the end of the path has been reached.
    geometry_msgs::msg::Twist computeCommand(const nav_msgs::msg::Path & path,
                                             double robot_x, double robot_y,
                                             double robot_yaw) const;

  private:
    // Index of the path point nearest the robot. Pursuit starts from here so the
    // controller cannot latch onto an earlier leg of a path that doubles back.
    size_t closestPointIndex(const nav_msgs::msg::Path & path,
                             double robot_x, double robot_y) const;

    rclcpp::Logger logger_;

    double lookahead_distance_;
    double linear_speed_;
    double max_angular_;
    double goal_tolerance_;
};

}

#endif
