#ifndef PLANNER_NODE_HPP_
#define PLANNER_NODE_HPP_

#include "rclcpp/rclcpp.hpp"

#include "geometry_msgs/msg/point_stamped.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"

#include "planner_core.hpp"

class PlannerNode : public rclcpp::Node {
  public:
    PlannerNode();

  private:
    // Either we are idle, or we are driving to a goal and replanning as the map
    // fills in. Everything the node does hangs off which of these is current.
    enum class State
    {
      WAITING_FOR_GOAL,
      WAITING_FOR_ROBOT_TO_REACH_GOAL
    };

    void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr map);
    void goalCallback(const geometry_msgs::msg::PointStamped::SharedPtr goal);
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr odom);
    void timerCallback();

    void replan();
    bool goalReached() const;
    void publishEmptyPath();

    robot::PlannerCore planner_;

    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr goal_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    double goal_tolerance_;
    // Distance from the reported odom frame back to the chassis centre.
    double base_offset_;

    State state_;
    bool have_map_;
    bool have_odom_;

    nav_msgs::msg::OccupancyGrid map_;
    double robot_x_;
    double robot_y_;
    double goal_x_;
    double goal_y_;
};

#endif
