#include <algorithm>
#include <cmath>

#include "map_memory_core.hpp"

namespace robot
{

MapMemoryCore::MapMemoryCore(const rclcpp::Logger& logger)
: logger_(logger), resolution_(0.1), width_(300), height_(300) {}

void MapMemoryCore::configure(double resolution, int width, int height,
                              double origin_x, double origin_y,
                              const std::string & frame_id)
{
  resolution_ = resolution;
  width_ = width;
  height_ = height;

  map_.header.frame_id = frame_id;
  map_.info.resolution = resolution_;
  map_.info.width = static_cast<uint32_t>(width_);
  map_.info.height = static_cast<uint32_t>(height_);
  map_.info.origin.position.x = origin_x;
  map_.info.origin.position.y = origin_y;
  map_.info.origin.position.z = 0.0;
  map_.info.origin.orientation.w = 1.0;

  // -1 is "unknown" in the OccupancyGrid convention. The planner treats it as
  // traversable, so the robot can head into territory it has not seen yet.
  map_.data.assign(static_cast<size_t>(width_) * static_cast<size_t>(height_), -1);

  RCLCPP_INFO(logger_, "Global map configured: %dx%d cells at %.2fm, origin (%.1f, %.1f)",
              width_, height_, resolution_, origin_x, origin_y);
}

bool MapMemoryCore::worldToGrid(double x, double y, int & grid_x, int & grid_y) const
{
  grid_x = static_cast<int>(std::floor((x - map_.info.origin.position.x) / resolution_));
  grid_y = static_cast<int>(std::floor((y - map_.info.origin.position.y) / resolution_));
  return grid_x >= 0 && grid_x < width_ && grid_y >= 0 && grid_y < height_;
}

void MapMemoryCore::fuse(const nav_msgs::msg::OccupancyGrid & local,
                         double robot_x, double robot_y, double yaw)
{
  const int local_width = static_cast<int>(local.info.width);
  const int local_height = static_cast<int>(local.info.height);
  const double local_resolution = local.info.resolution;
  const double local_origin_x = local.info.origin.position.x;
  const double local_origin_y = local.info.origin.position.y;

  const double cos_yaw = std::cos(yaw);
  const double sin_yaw = std::sin(yaw);

  for (int j = 0; j < local_height; ++j) {
    for (int i = 0; i < local_width; ++i) {
      const int8_t value =
        local.data[static_cast<size_t>(j) * static_cast<size_t>(local_width) +
                   static_cast<size_t>(i)];

      // Unknown local cells carry no evidence; writing them would erase what we
      // already know about that patch of the world.
      if (value < 0) {
        continue;
      }

      // Cell centre in the robot's frame.
      const double lx = local_origin_x + (static_cast<double>(i) + 0.5) * local_resolution;
      const double ly = local_origin_y + (static_cast<double>(j) + 0.5) * local_resolution;

      // Rotate by the robot's yaw, then translate by its position.
      const double gx = robot_x + lx * cos_yaw - ly * sin_yaw;
      const double gy = robot_y + lx * sin_yaw + ly * cos_yaw;

      int grid_x = 0;
      int grid_y = 0;
      if (!worldToGrid(gx, gy, grid_x, grid_y)) {
        continue;
      }

      int8_t & cell = map_.data[index(grid_x, grid_y)];
      // Taking the max makes the map monotonic: a free reading never clears an
      // obstacle. The world here is static, and without raytracing we cannot
      // distinguish "seen empty" from "behind an obstacle, never observed".
      if (value > cell) {
        cell = value;
      }
    }
  }
}

}
