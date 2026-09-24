#include <chrono>
#include <memory>

#include "costmap_node.hpp"

CostmapNode::CostmapNode() : Node("costmap"), costmap_(robot::CostmapCore(this->get_logger())) {
  const double resolution = this->declare_parameter<double>("resolution", 0.1);
  const int width = this->declare_parameter<int>("width", 200);
  const int height = this->declare_parameter<int>("height", 200);
  const double inflation_radius = this->declare_parameter<double>("inflation_radius", 1.2);
  const int max_cost = this->declare_parameter<int>("max_cost", 100);
  const double max_range = this->declare_parameter<double>("max_range", 10.0);

  costmap_.configure(resolution, width, height, inflation_radius, max_cost, max_range);

  costmap_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/costmap", 10);
  scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
    "/lidar", 10, std::bind(&CostmapNode::scanCallback, this, std::placeholders::_1));
}

void CostmapNode::scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan) {
  nav_msgs::msg::OccupancyGrid grid = costmap_.buildFromScan(*scan);

  // The costmap is expressed in the sensor's own frame, so inherit it from the
  // scan rather than hardcoding a frame name the bridge might rename.
  grid.header.stamp = scan->header.stamp;
  grid.header.frame_id = scan->header.frame_id;

  costmap_pub_->publish(grid);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CostmapNode>());
  rclcpp::shutdown();
  return 0;
}
