#include <chrono>
#include <cmath>
#include <memory>

#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include "planner_node.hpp"

PlannerNode::PlannerNode()
: Node("planner"),
  planner_(robot::PlannerCore(this->get_logger())),
  state_(State::WAITING_FOR_GOAL),
  have_map_(false),
  have_odom_(false),
  robot_x_(0.0),
  robot_y_(0.0),
  goal_x_(0.0),
  goal_y_(0.0)
{
  const int occupancy_threshold = this->declare_parameter<int>("occupancy_threshold", 80);
  const double cost_weight = this->declare_parameter<double>("cost_weight", 2.0);
  goal_tolerance_ = this->declare_parameter<double>("goal_tolerance", 0.5);
  base_offset_ = this->declare_parameter<double>("base_offset", 0.8);
  const double replan_period = this->declare_parameter<double>("replan_period", 1.0);

  planner_.configure(occupancy_threshold, cost_weight);

  path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/path", 10);
  map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
    "/map", 10, std::bind(&PlannerNode::mapCallback, this, std::placeholders::_1));
  goal_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
    "/goal_point", 10, std::bind(&PlannerNode::goalCallback, this, std::placeholders::_1));
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/odom/filtered", 10, std::bind(&PlannerNode::odomCallback, this, std::placeholders::_1));

  timer_ = this->create_wall_timer(
    std::chrono::duration<double>(replan_period),
    std::bind(&PlannerNode::timerCallback, this));
}

void PlannerNode::mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr map)
{
  map_ = *map;
  have_map_ = true;

  // A fresh map may have revealed an obstacle across the current path, so replan
  // straight away rather than waiting for the timer.
  if (state_ == State::WAITING_FOR_ROBOT_TO_REACH_GOAL) {
    replan();
  }
}

void PlannerNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr odom)
{
  // odom/filtered reports the lidar frame, which sits base_offset_ ahead of the
  // chassis centre, with most of the body trailing behind it. Planning for that
  // point lets the rear of the robot clip obstacles the path appeared to clear,
  // so shift back to the centre of the chassis before planning.
  const double yaw = tf2::getYaw(odom->pose.pose.orientation);
  robot_x_ = odom->pose.pose.position.x - base_offset_ * std::cos(yaw);
  robot_y_ = odom->pose.pose.position.y - base_offset_ * std::sin(yaw);
  have_odom_ = true;
}

void PlannerNode::goalCallback(const geometry_msgs::msg::PointStamped::SharedPtr goal)
{
  goal_x_ = goal->point.x;
  goal_y_ = goal->point.y;
  state_ = State::WAITING_FOR_ROBOT_TO_REACH_GOAL;

  RCLCPP_INFO(this->get_logger(), "New goal: (%.2f, %.2f)", goal_x_, goal_y_);
  replan();
}

void PlannerNode::timerCallback()
{
  if (state_ != State::WAITING_FOR_ROBOT_TO_REACH_GOAL) {
    return;
  }

  if (goalReached()) {
    RCLCPP_INFO(this->get_logger(), "Goal reached, waiting for the next one");
    state_ = State::WAITING_FOR_GOAL;
    // An empty path tells the controller to stop; without it the robot would
    // keep driving on the last path it was given.
    publishEmptyPath();
    return;
  }

  replan();
}

bool PlannerNode::goalReached() const
{
  if (!have_odom_) {
    return false;
  }
  return std::hypot(goal_x_ - robot_x_, goal_y_ - robot_y_) < goal_tolerance_;
}

void PlannerNode::replan()
{
  if (!have_map_ || !have_odom_) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                         "Waiting for map and odometry before planning");
    return;
  }

  nav_msgs::msg::Path path;
  path.header.stamp = this->now();
  path.header.frame_id = map_.header.frame_id;

  if (!planner_.planPath(map_, robot_x_, robot_y_, goal_x_, goal_y_, path)) {
    // Hold the previous path rather than publishing a broken one. If the goal is
    // genuinely unreachable the controller will finish the old path and stop.
    return;
  }

  path_pub_->publish(path);
}

void PlannerNode::publishEmptyPath()
{
  nav_msgs::msg::Path path;
  path.header.stamp = this->now();
  path.header.frame_id = map_.header.frame_id;
  path_pub_->publish(path);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PlannerNode>());
  rclcpp::shutdown();
  return 0;
}
