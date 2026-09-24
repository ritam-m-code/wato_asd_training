#include <chrono>
#include <cmath>
#include <memory>

#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include "map_memory_node.hpp"

MapMemoryNode::MapMemoryNode()
: Node("map_memory"),
  map_memory_(robot::MapMemoryCore(this->get_logger())),
  have_odom_(false),
  have_fused_(false),
  robot_x_(0.0),
  robot_y_(0.0),
  robot_yaw_(0.0),
  last_fused_x_(0.0),
  last_fused_y_(0.0)
{
  const double resolution = this->declare_parameter<double>("resolution", 0.1);
  const int width = this->declare_parameter<int>("width", 300);
  const int height = this->declare_parameter<int>("height", 300);
  const double origin_x = this->declare_parameter<double>("origin_x", -15.0);
  const double origin_y = this->declare_parameter<double>("origin_y", -15.0);
  const std::string frame_id = this->declare_parameter<std::string>("frame_id", "sim_world");
  update_distance_ = this->declare_parameter<double>("update_distance", 1.5);
  const double publish_rate = this->declare_parameter<double>("publish_rate", 1.0);

  map_memory_.configure(resolution, width, height, origin_x, origin_y, frame_id);

  map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/map", 10);
  costmap_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
    "/costmap", 10, std::bind(&MapMemoryNode::costmapCallback, this, std::placeholders::_1));
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/odom/filtered", 10, std::bind(&MapMemoryNode::odomCallback, this, std::placeholders::_1));

  // Publishing on a timer rather than on fusion means the map is available from
  // startup. If it only appeared after the first fusion, the planner would have
  // nothing to plan against until the robot was driven manually, and the whole
  // stack would look dead.
  timer_ = this->create_wall_timer(
    std::chrono::duration<double>(1.0 / publish_rate),
    std::bind(&MapMemoryNode::publishMap, this));
}

void MapMemoryNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr odom)
{
  robot_x_ = odom->pose.pose.position.x;
  robot_y_ = odom->pose.pose.position.y;
  robot_yaw_ = tf2::getYaw(odom->pose.pose.orientation);
  have_odom_ = true;
}

void MapMemoryNode::costmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr costmap)
{
  if (!have_odom_) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                         "Costmap received before any odometry; skipping fusion");
    return;
  }

  // Fusing every frame would burn CPU redrawing the same cells. Gate on travel
  // instead, but always take the first scan so the map is populated on startup.
  if (have_fused_) {
    const double moved = std::hypot(robot_x_ - last_fused_x_, robot_y_ - last_fused_y_);
    if (moved < update_distance_) {
      return;
    }
  }

  map_memory_.fuse(*costmap, robot_x_, robot_y_, robot_yaw_);

  last_fused_x_ = robot_x_;
  last_fused_y_ = robot_y_;
  have_fused_ = true;
}

void MapMemoryNode::publishMap()
{
  nav_msgs::msg::OccupancyGrid & map = map_memory_.mutableMap();
  map.header.stamp = this->now();
  map.info.map_load_time = map.header.stamp;
  map_pub_->publish(map);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MapMemoryNode>());
  rclcpp::shutdown();
  return 0;
}
