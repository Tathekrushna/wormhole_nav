#include "rclcpp/rclcpp.hpp"
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <string>
#include <sqlite3.h>



class WormholeNavigator : public rclcpp::Node
{
public:
    WormholeNavigator()
    : Node("wormhole_navigator"), current_map_("r1")  // Start in map 'r1'
    {
        RCLCPP_INFO(this->get_logger(), "Wormhole Navigator Node Started in map: %s", current_map_.c_str());

        // Set example target
        std::string target_map = "r2";
        float goal_x = -3.336;  
        float goal_y = 9.321;
        float goal_yaw = 1.534;

        navigate_to_goal(target_map, goal_x, goal_y, goal_yaw);
    }

private:
    std::string current_map_;

    void navigate_to_goal(const std::string& target_map, float x, float y, float yaw)
    {
        if (target_map == current_map_)
        {
            RCLCPP_INFO(this->get_logger(), "Same map, going directly to goal");
            move_to_pose(x, y, yaw);
        }
        else
        {
            float wx, wy, wyaw;
            if (get_wormhole_pose(current_map_, target_map, wx, wy, wyaw))
            {
                RCLCPP_INFO(this->get_logger(), "Moving to wormhole at (%.2f, %.2f)", wx, wy);
                move_to_pose(wx, wy, wyaw);

                // Delay to allow movement before switching maps
                rclcpp::sleep_for(std::chrono::seconds(5));

                load_map(target_map);

                RCLCPP_INFO(this->get_logger(), "Now navigating in %s to (%.2f, %.2f)", target_map.c_str(), x, y);
                move_to_pose(x, y, yaw);
            }
            else
            {
                RCLCPP_ERROR(this->get_logger(), "No wormhole found from %s to %s", current_map_.c_str(), target_map.c_str());
            }
        }
    }

    void move_to_pose(float x, float y, float yaw)
    {
        tf2::Quaternion q;
        q.setRPY(0, 0, yaw);
        q.normalize();

        std::string cmd = "ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \"{pose: {header: {frame_id: 'map'}, pose: {position: {x: " +
                          std::to_string(x) + ", y: " + std::to_string(y) + "}, orientation: {z: " + std::to_string(q.getZ()) +
                          ", w: " + std::to_string(q.getW()) + "}}}}\"";

        RCLCPP_INFO(this->get_logger(), "Executing navigation command...");
        system(cmd.c_str());
    }

    void load_map(const std::string& map_name)
    {
        std::string full_path = "/home/krushna/dev_ws/src/basic_mobile_robot/maps/" + map_name + ".yaml";
        std::string command = "ros2 service call /map_server/load_map nav2_msgs/srv/LoadMap \"{map_url: '" + full_path + "'}\"";
        RCLCPP_INFO(this->get_logger(), "Loading map: %s", full_path.c_str());

        system(command.c_str());

        current_map_ = map_name;
        RCLCPP_INFO(this->get_logger(), "Map switched to: %s", current_map_.c_str());
    }

    bool get_wormhole_pose(const std::string& from, const std::string& to, float &wx, float &wy, float &wyaw)
    {
        sqlite3* db;
        int rc = sqlite3_open("/home/krushna/dev_ws/src/wormhole_nav/wormholes.db", &db);
        if (rc)
        {
            RCLCPP_ERROR(this->get_logger(), "Can't open wormhole database: %s", sqlite3_errmsg(db));
            return false;
        }

        std::string query = "SELECT source_x, source_y, source_yaw FROM wormholes WHERE source_map='" + from + "' AND target_map='" + to + "';";
        sqlite3_stmt* stmt;

        rc = sqlite3_prepare_v2(db, query.c_str(), -1, &stmt, nullptr);
        if (rc != SQLITE_OK)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to execute query: %s", sqlite3_errmsg(db));
            sqlite3_close(db);
            return false;
        }

        if (sqlite3_step(stmt) == SQLITE_ROW)
        {
            wx = sqlite3_column_double(stmt, 0);
            wy = sqlite3_column_double(stmt, 1);
            wyaw = sqlite3_column_double(stmt, 2);

            sqlite3_finalize(stmt);
            sqlite3_close(db);
            return true;
        }
        else
        {
            RCLCPP_WARN(this->get_logger(), "No wormhole entry found in DB.");
            sqlite3_finalize(stmt);
            sqlite3_close(db);
            return false;
        }
    }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<WormholeNavigator>());
    rclcpp::shutdown();
    return 0;
}

