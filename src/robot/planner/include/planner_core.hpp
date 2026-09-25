#ifndef PLANNER_CORE_HPP_
#define PLANNER_CORE_HPP_

#include <cstddef>
#include <functional>

#include "rclcpp/rclcpp.hpp"

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/path.hpp"

namespace robot
{

// A position in the occupancy grid.
struct CellIndex
{
  int x;
  int y;

  CellIndex() : x(0), y(0) {}
  CellIndex(int xx, int yy) : x(xx), y(yy) {}

  bool operator==(const CellIndex & other) const { return x == other.x && y == other.y; }
  bool operator!=(const CellIndex & other) const { return !(*this == other); }
};

// Lets CellIndex be used as a key in the unordered containers A* needs.
struct CellIndexHash
{
  std::size_t operator()(const CellIndex & index) const
  {
    return std::hash<int>()(index.x) ^ (std::hash<int>()(index.y) << 1);
  }
};

// An entry in the open set, ordered by f = g + h.
struct AStarNode
{
  CellIndex index;
  double f_score;

  AStarNode(CellIndex idx, double f) : index(idx), f_score(f) {}
};

// std::priority_queue is a max-heap by default, so invert the comparison to pop
// the lowest f-score first.
struct CompareF
{
  bool operator()(const AStarNode & a, const AStarNode & b) const
  {
    return a.f_score > b.f_score;
  }
};

class PlannerCore {
  public:
    explicit PlannerCore(const rclcpp::Logger& logger);

    void configure(int occupancy_threshold, double cost_weight);

    // Runs A* from start to goal over the map. Returns false and leaves the path
    // untouched if no route exists; the caller decides what to do about it.
    bool planPath(const nav_msgs::msg::OccupancyGrid & map,
                  double start_x, double start_y,
                  double goal_x, double goal_y,
                  nav_msgs::msg::Path & path) const;

    bool worldToGrid(const nav_msgs::msg::OccupancyGrid & map,
                     double x, double y, CellIndex & cell) const;
    void gridToWorld(const nav_msgs::msg::OccupancyGrid & map,
                     const CellIndex & cell, double & x, double & y) const;
    bool isTraversable(const nav_msgs::msg::OccupancyGrid & map,
                       const CellIndex & cell) const;

  private:
    // Multiplier applied to a step entering this cell, so the path is pushed away
    // from inflated regions rather than merely avoiding lethal ones.
    double cellPenalty(const nav_msgs::msg::OccupancyGrid & map,
                       const CellIndex & cell) const;

    rclcpp::Logger logger_;

    int occupancy_threshold_;
    double cost_weight_;
    // Multiplier applied to lethal cells when the robot starts inside one. It
    // has to be large enough that escaping is always preferred to loitering.
    double escape_penalty_;
};

}

#endif
