#ifndef MAP_MEMORY_CORE_HPP_
#define MAP_MEMORY_CORE_HPP_

#include <cstdint>
#include <string>

#include "rclcpp/rclcpp.hpp"

#include "nav_msgs/msg/occupancy_grid.hpp"

namespace robot
{

// Accumulates robot-centric costmaps into a single world-fixed occupancy grid.
//
// The local costmap arrives in the sensor frame, which rotates with the robot,
// so every cell has to be rotated by the robot's yaw before it lands in the
// global grid. Translating without rotating is the classic failure here: the
// map still fills in, but obstacles smear around the true geometry.
class MapMemoryCore {
  public:
    explicit MapMemoryCore(const rclcpp::Logger& logger);

    // Sizes the global grid and fills it with "unknown". Call once at startup.
    void configure(double resolution, int width, int height,
                   double origin_x, double origin_y, const std::string & frame_id);

    // Transforms every known cell of the local costmap into the global grid.
    void fuse(const nav_msgs::msg::OccupancyGrid & local,
              double robot_x, double robot_y, double yaw);

    const nav_msgs::msg::OccupancyGrid & map() const { return map_; }
    nav_msgs::msg::OccupancyGrid & mutableMap() { return map_; }

  private:
    bool worldToGrid(double x, double y, int & grid_x, int & grid_y) const;

    size_t index(int grid_x, int grid_y) const {
      return static_cast<size_t>(grid_y) * static_cast<size_t>(width_) +
             static_cast<size_t>(grid_x);
    }

    rclcpp::Logger logger_;

    double resolution_;
    int width_;
    int height_;

    nav_msgs::msg::OccupancyGrid map_;
};

}  

#endif  
