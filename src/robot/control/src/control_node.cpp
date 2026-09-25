#include <chrono>
#include <cmath>
#include <memory>

#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include "control_node.hpp"

ControlNode::ControlNode()
: Node("control"),
  control_(robot::ControlCore(this->get_logger())),
  have_odom_(false),
  robot_x_(0.0),
  robot_y_(0.0),
  robot_yaw_(0.0)
{
  const double lookahead_distance = this->declare_parameter<double>("lookahead_distance", 1.5);
  const double linear_speed = this->declare_parameter<double>("linear_speed", 0.5);
  const double max_angular = this->declare_parameter<double>("max_angular", 1.0);
  const double goal_tolerance = this->declare_parameter<double>("goal_tolerance", 0.5);
  const double control_rate = this->declare_parameter<double>("control_rate", 10.0);
  base_offset_ = this->declare_parameter<double>("base_offset", 0.8);

  control_.configure(lookahead_distance, linear_speed, max_angular, goal_tolerance);

  cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
  path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
    "/path", 10, std::bind(&ControlNode::pathCallback, this, std::placeholders::_1));
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/odom/filtered", 10, std::bind(&ControlNode::odomCallback, this, std::placeholders::_1));

  // Steering runs on its own clock rather than inside the path callback. The
  // planner only replans about once a second, and the robot has to keep tracking
  // the path in between.
  timer_ = this->create_wall_timer(
    std::chrono::duration<double>(1.0 / control_rate),
    std::bind(&ControlNode::timerCallback, this));
}

void ControlNode::pathCallback(const nav_msgs::msg::Path::SharedPtr path)
{
  path_ = *path;
}

void ControlNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr odom)
{
  robot_yaw_ = tf2::getYaw(odom->pose.pose.orientation);
  // Steer the chassis centre rather than the lidar, which sits base_offset_
  // ahead of it. The centre is still forward of the wheel axle, which keeps
  // pure pursuit well damped.
  robot_x_ = odom->pose.pose.position.x - base_offset_ * std::cos(robot_yaw_);
  robot_y_ = odom->pose.pose.position.y - base_offset_ * std::sin(robot_yaw_);
  have_odom_ = true;
}

void ControlNode::timerCallback()
{
  if (!have_odom_) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                         "Waiting for odometry before issuing velocity commands");
    return;
  }

  cmd_vel_pub_->publish(control_.computeCommand(path_, robot_x_, robot_y_, robot_yaw_));
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ControlNode>());
  rclcpp::shutdown();
  return 0;
}
