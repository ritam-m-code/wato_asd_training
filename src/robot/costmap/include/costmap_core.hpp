#ifndef COSTMAP_CORE_HPP_
#define COSTMAP_CORE_HPP_

#include <cstdint>
#include <utility>
#include <vector>

#include "rclcpp/rclcpp.hpp"

#include "nav_msgs/msg/occupancy_grid.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

namespace robot
{

// Builds a robot-centric occupancy grid from a single laser scan.
//
// The grid is rebuilt from scratch on every scan, so there is no state to decay
// or time out. Each beam is raytraced: the cells it sweeps through become free,
// its endpoint becomes an obstacle, and everything it never reached stays
// unknown. Marking unswept cells free instead would publish the inside of every
// obstacle as confirmed-clear space.
//
// Obstacles are then inflated radially, which also closes the gaps between
// beams: at 256 samples over 2*pi neighbouring rays are ~0.25m apart at 10m,
// far wider than one 0.1m cell.
class CostmapCore {
  public:
    // Constructor, we pass in the node's RCLCPP logger to enable logging to terminal
    explicit CostmapCore(const rclcpp::Logger& logger);

    // Sizes the grid and precomputes the inflation window. Call once at startup.
    void configure(double resolution, int width, int height,
                   double robot_radius, double inflation_radius,
                   int max_cost, double max_range);

    // Rebuilds the grid from the scan. The returned grid has its geometry filled
    // in but an empty header; the caller stamps it before publishing.
    const nav_msgs::msg::OccupancyGrid & buildFromScan(
      const sensor_msgs::msg::LaserScan & scan);

  private:
    void reset();
    // Marks the cells between two grid points as free, endpoint excluded.
    void traceFree(int x0, int y0, int x1, int y1);
    void inflate();
    bool worldToGrid(double x, double y, int & grid_x, int & grid_y) const;

    size_t index(int grid_x, int grid_y) const {
      return static_cast<size_t>(grid_y) * static_cast<size_t>(width_) +
             static_cast<size_t>(grid_x);
    }

    rclcpp::Logger logger_;

    double resolution_;
    int width_;
    int height_;
    // Cells within this distance of an obstacle are lethal: the robot body
    // would occupy them. Beyond it, up to inflation_radius_, cost decays.
    double robot_radius_;
    double inflation_radius_;
    int max_cost_;
    double max_range_;
    int inflation_cells_;

    nav_msgs::msg::OccupancyGrid grid_;
    // Flat indices of cells hit by a beam this cycle, collected during marking so
    // inflation only visits real obstacles instead of rescanning the whole grid.
    std::vector<size_t> obstacles_;
};

}  

#endif  
