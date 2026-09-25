#include <algorithm>
#include <cmath>

#include "costmap_core.hpp"

namespace robot
{

CostmapCore::CostmapCore(const rclcpp::Logger& logger)
: logger_(logger),
  resolution_(0.1),
  width_(200),
  height_(200),
  robot_radius_(1.15),
  inflation_radius_(2.0),
  max_cost_(100),
  max_range_(10.0),
  inflation_cells_(20) {}

void CostmapCore::configure(double resolution, int width, int height,
                            double robot_radius, double inflation_radius,
                            int max_cost, double max_range)
{
  resolution_ = resolution;
  width_ = width;
  height_ = height;
  robot_radius_ = robot_radius;
  inflation_radius_ = std::max(inflation_radius, robot_radius + resolution);
  max_cost_ = max_cost;
  max_range_ = max_range;
  inflation_cells_ = static_cast<int>(std::ceil(inflation_radius_ / resolution_));

  // The robot sits at the centre of its own costmap, so the grid origin is half
  // the extent back along each axis.
  grid_.info.resolution = resolution_;
  grid_.info.width = static_cast<uint32_t>(width_);
  grid_.info.height = static_cast<uint32_t>(height_);
  grid_.info.origin.position.x = -0.5 * width_ * resolution_;
  grid_.info.origin.position.y = -0.5 * height_ * resolution_;
  grid_.info.origin.position.z = 0.0;
  grid_.info.origin.orientation.w = 1.0;
  grid_.data.assign(static_cast<size_t>(width_) * static_cast<size_t>(height_), -1);

  RCLCPP_INFO(logger_,
              "Costmap configured: %dx%d cells at %.2fm, lethal core %.2fm, inflation %.2fm (%d cells)",
              width_, height_, resolution_, robot_radius_, inflation_radius_, inflation_cells_);
}

void CostmapCore::reset()
{
  // Everything starts unknown. Only cells a beam actually swept through are
  // downgraded to free, so the grid never claims to have seen behind an obstacle.
  std::fill(grid_.data.begin(), grid_.data.end(), -1);
  obstacles_.clear();
}

bool CostmapCore::worldToGrid(double x, double y, int & grid_x, int & grid_y) const
{
  grid_x = static_cast<int>(std::floor((x - grid_.info.origin.position.x) / resolution_));
  grid_y = static_cast<int>(std::floor((y - grid_.info.origin.position.y) / resolution_));
  return grid_x >= 0 && grid_x < width_ && grid_y >= 0 && grid_y < height_;
}

void CostmapCore::traceFree(int x0, int y0, int x1, int y1)
{
  // Bresenham from the sensor to the return, marking everything it crosses as
  // free. The endpoint itself is left alone; pass two paints the obstacle there.
  int dx = std::abs(x1 - x0);
  int dy = -std::abs(y1 - y0);
  const int sx = x0 < x1 ? 1 : -1;
  const int sy = y0 < y1 ? 1 : -1;
  int error = dx + dy;

  int x = x0;
  int y = y0;
  while (true) {
    if (x == x1 && y == y1) {
      return;
    }
    if (x < 0 || x >= width_ || y < 0 || y >= height_) {
      return;  // the beam has left the grid; nothing further is observable
    }

    int8_t & cell = grid_.data[index(x, y)];
    if (cell < 0) {
      cell = 0;
    }

    const int doubled = 2 * error;
    if (doubled >= dy) {
      error += dy;
      x += sx;
    }
    if (doubled <= dx) {
      error += dx;
      y += sy;
    }
  }
}

const nav_msgs::msg::OccupancyGrid & CostmapCore::buildFromScan(
  const sensor_msgs::msg::LaserScan & scan)
{
  reset();

  const double range_ceiling = std::min(static_cast<double>(scan.range_max), max_range_);

  int centre_x = 0;
  int centre_y = 0;
  worldToGrid(0.0, 0.0, centre_x, centre_y);

  std::vector<std::pair<int, int>> hits;
  hits.reserve(scan.ranges.size());

  // Pass one: carve out the free space every beam swept through.
  for (size_t i = 0; i < scan.ranges.size(); ++i) {
    const double range = static_cast<double>(scan.ranges[i]);
    const double angle = scan.angle_min + static_cast<double>(i) * scan.angle_increment;

    double beam_length = 0.0;
    bool struck = false;

    if (std::isfinite(range)) {
      if (range < scan.range_min) {
        continue;  // a spurious short return tells us nothing
      }
      if (range <= range_ceiling) {
        beam_length = range;
        struck = true;
      } else {
        // A return beyond our window still proves the space in between is clear.
        beam_length = range_ceiling;
      }
    } else {
      // No return at all: clear all the way out to the edge of the window.
      beam_length = range_ceiling;
    }

    int end_x = 0;
    int end_y = 0;
    const bool inside = worldToGrid(beam_length * std::cos(angle),
                                    beam_length * std::sin(angle), end_x, end_y);

    traceFree(centre_x, centre_y, end_x, end_y);

    if (struck && inside) {
      hits.emplace_back(end_x, end_y);
    }
  }

  // Pass two: paint the returns. Doing this after all the raytracing stops one
  // beam's free space from erasing another beam's obstacle.
  for (const auto & hit : hits) {
    int8_t & cell = grid_.data[index(hit.first, hit.second)];
    if (cell != static_cast<int8_t>(max_cost_)) {
      cell = static_cast<int8_t>(max_cost_);
      obstacles_.push_back(index(hit.first, hit.second));
    }
  }

  inflate();
  return grid_;
}

void CostmapCore::inflate()
{
  for (const size_t obstacle : obstacles_) {
    const int ox = static_cast<int>(obstacle % static_cast<size_t>(width_));
    const int oy = static_cast<int>(obstacle / static_cast<size_t>(width_));

    for (int dy = -inflation_cells_; dy <= inflation_cells_; ++dy) {
      const int ny = oy + dy;
      if (ny < 0 || ny >= height_) {
        continue;
      }
      for (int dx = -inflation_cells_; dx <= inflation_cells_; ++dx) {
        const int nx = ox + dx;
        if (nx < 0 || nx >= width_) {
          continue;
        }
        if (dx == 0 && dy == 0) {
          continue;  // the obstacle itself stays at max cost
        }

        const double distance = std::hypot(dx, dy) * resolution_;
        if (distance > inflation_radius_) {
          continue;  // the window is square, the falloff is circular
        }

        // Anything the robot body would occupy is lethal, not merely expensive.
        // The chassis circumscribed radius measured from the control point is
        // ~1.12m, so a path grazing an obstacle at 0.3m would drag the body
        // through it. Beyond that core the cost decays, which steers the planner
        // towards the middle of a gap without forbidding a tight but legal one.
        int8_t cost;
        if (distance <= robot_radius_) {
          cost = static_cast<int8_t>(max_cost_);
        } else {
          const double decay =
            1.0 - (distance - robot_radius_) / (inflation_radius_ - robot_radius_);
          cost = static_cast<int8_t>(max_cost_ * decay);
        }

        int8_t & cell = grid_.data[index(nx, ny)];
        if (cost > cell) {
          cell = cost;  // overlapping inflations keep the highest cost
        }
      }
    }
  }
}

}
