#pragma once

#include <rclcpp/rclcpp.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <sqlite3.h>
#include <string>

class WormholeNavigator : public rclcpp::Node
{
public:
  WormholeNavigator();
  void navigate_to_goal(const std::string& target_map, float x, float y, float yaw);

private:
  std::string current_map_;
  bool is_same_map(const std::string& target_map);
  void load_map(const std::string& map_name);
  void move_to_pose(float x, float y, float yaw);
  bool get_wormhole_pose(const std::string& from, const std::string& to, float &wx, float &wy, float &wyaw);
};

