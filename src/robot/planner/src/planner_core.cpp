#include <algorithm>
#include <cmath>
#include <queue>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "planner_core.hpp"

namespace robot
{

namespace
{
constexpr double kDiagonalStep = 1.41421356237;
}

PlannerCore::PlannerCore(const rclcpp::Logger& logger)
: logger_(logger), occupancy_threshold_(80), cost_weight_(2.0), escape_penalty_(25.0) {}

void PlannerCore::configure(int occupancy_threshold, double cost_weight)
{
  occupancy_threshold_ = occupancy_threshold;
  cost_weight_ = cost_weight;
  RCLCPP_INFO(logger_, "Planner configured: threshold %d, cost weight %.2f",
              occupancy_threshold_, cost_weight_);
}

bool PlannerCore::worldToGrid(const nav_msgs::msg::OccupancyGrid & map,
                              double x, double y, CellIndex & cell) const
{
  const double resolution = map.info.resolution;
  cell.x = static_cast<int>(std::floor((x - map.info.origin.position.x) / resolution));
  cell.y = static_cast<int>(std::floor((y - map.info.origin.position.y) / resolution));
  return cell.x >= 0 && cell.x < static_cast<int>(map.info.width) &&
         cell.y >= 0 && cell.y < static_cast<int>(map.info.height);
}

void PlannerCore::gridToWorld(const nav_msgs::msg::OccupancyGrid & map,
                              const CellIndex & cell, double & x, double & y) const
{
  // Aim at cell centres so the path does not hug cell corners.
  x = map.info.origin.position.x + (static_cast<double>(cell.x) + 0.5) * map.info.resolution;
  y = map.info.origin.position.y + (static_cast<double>(cell.y) + 0.5) * map.info.resolution;
}

bool PlannerCore::isTraversable(const nav_msgs::msg::OccupancyGrid & map,
                                const CellIndex & cell) const
{
  if (cell.x < 0 || cell.x >= static_cast<int>(map.info.width) ||
      cell.y < 0 || cell.y >= static_cast<int>(map.info.height)) {
    return false;
  }
  const int8_t value = map.data[static_cast<size_t>(cell.y) *
                                static_cast<size_t>(map.info.width) +
                                static_cast<size_t>(cell.x)];
  // Unknown (-1) counts as free: the map starts entirely unknown, and refusing
  // to enter unseen territory would leave the robot unable to plan anywhere.
  return value < occupancy_threshold_;
}

double PlannerCore::cellPenalty(const nav_msgs::msg::OccupancyGrid & map,
                                const CellIndex & cell) const
{
  const int8_t value = map.data[static_cast<size_t>(cell.y) *
                                static_cast<size_t>(map.info.width) +
                                static_cast<size_t>(cell.x)];
  const double cost = value < 0 ? 0.0 : static_cast<double>(value);
  return 1.0 + cost_weight_ * cost / 100.0;
}

bool PlannerCore::planPath(const nav_msgs::msg::OccupancyGrid & map,
                           double start_x, double start_y,
                           double goal_x, double goal_y,
                           nav_msgs::msg::Path & path) const
{
  if (map.data.empty()) {
    RCLCPP_WARN(logger_, "Cannot plan: map is empty");
    return false;
  }

  CellIndex start;
  CellIndex goal;
  if (!worldToGrid(map, start_x, start_y, start)) {
    RCLCPP_WARN(logger_, "Cannot plan: start (%.2f, %.2f) is outside the map", start_x, start_y);
    return false;
  }
  if (!worldToGrid(map, goal_x, goal_y, goal)) {
    RCLCPP_WARN(logger_, "Cannot plan: goal (%.2f, %.2f) is outside the map", goal_x, goal_y);
    return false;
  }
  if (!isTraversable(map, goal)) {
    RCLCPP_WARN(logger_, "Cannot plan: goal (%.2f, %.2f) lies inside an obstacle",
                goal_x, goal_y);
    return false;
  }

  // If the robot has ended up inside lethal space -- nudged by a controller
  // overshoot, or an obstacle mapped after it drove past -- every neighbour is
  // lethal too and a strict search expands nothing, reports failure, and leaves
  // the robot grinding against whatever it hit. When that happens, allow the
  // search to cross lethal cells at a punitive cost so it can find its way out.
  const bool escaping = !isTraversable(map, start);
  if (escaping) {
    RCLCPP_WARN(logger_, "Start (%.2f, %.2f) is inside lethal space; planning an escape",
                start_x, start_y);
  }

  std::priority_queue<AStarNode, std::vector<AStarNode>, CompareF> open;
  std::unordered_map<CellIndex, double, CellIndexHash> g_score;
  std::unordered_map<CellIndex, CellIndex, CellIndexHash> came_from;
  std::unordered_set<CellIndex, CellIndexHash> closed;

  const auto heuristic = [&goal](const CellIndex & cell) {
    return std::hypot(static_cast<double>(cell.x - goal.x),
                      static_cast<double>(cell.y - goal.y));
  };

  g_score[start] = 0.0;
  open.emplace(start, heuristic(start));

  bool found = false;
  while (!open.empty()) {
    const AStarNode current = open.top();
    open.pop();

    // Lazy deletion: a cell can be queued more than once, so skip any copy that
    // has already been expanded.
    if (closed.count(current.index)) {
      continue;
    }
    closed.insert(current.index);

    if (current.index == goal) {
      found = true;
      break;
    }

    const double current_g = g_score[current.index];

    for (int dy = -1; dy <= 1; ++dy) {
      for (int dx = -1; dx <= 1; ++dx) {
        if (dx == 0 && dy == 0) {
          continue;
        }

        const CellIndex neighbour(current.index.x + dx, current.index.y + dy);
        if (closed.count(neighbour)) {
          continue;
        }
        if (neighbour.x < 0 || neighbour.x >= static_cast<int>(map.info.width) ||
            neighbour.y < 0 || neighbour.y >= static_cast<int>(map.info.height)) {
          continue;
        }

        const bool passable = isTraversable(map, neighbour);
        if (!passable && !escaping) {
          continue;
        }

        const double step = (dx != 0 && dy != 0) ? kDiagonalStep : 1.0;
        const double penalty =
          cellPenalty(map, neighbour) * (passable ? 1.0 : escape_penalty_);
        const double tentative_g = current_g + step * penalty;

        const auto existing = g_score.find(neighbour);
        if (existing != g_score.end() && tentative_g >= existing->second) {
          continue;
        }

        g_score[neighbour] = tentative_g;
        came_from[neighbour] = current.index;
        open.emplace(neighbour, tentative_g + heuristic(neighbour));
      }
    }
  }

  if (!found) {
    RCLCPP_WARN(logger_, "A* exhausted the open set without reaching the goal");
    return false;
  }

  // Walk the parent chain back from the goal, then reverse it.
  std::vector<CellIndex> cells;
  CellIndex cursor = goal;
  cells.push_back(cursor);
  while (cursor != start) {
    cursor = came_from[cursor];
    cells.push_back(cursor);
  }
  std::reverse(cells.begin(), cells.end());

  path.poses.clear();
  path.poses.reserve(cells.size());
  for (const CellIndex & cell : cells) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header = path.header;
    gridToWorld(map, cell, pose.pose.position.x, pose.pose.position.y);
    pose.pose.position.z = 0.0;
    pose.pose.orientation.w = 1.0;
    path.poses.push_back(pose);
  }

  RCLCPP_INFO(logger_, "Planned a path of %zu cells", cells.size());
  return true;
}

}
