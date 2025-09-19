#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include <string>
#include <cmath>
#include <vector>

class WaypointPublisher : public rclcpp::Node {
public:
    WaypointPublisher() : Node("waypoint_publisher_cpp") {
        pub_ = this->create_publisher<geometry_msgs::msg::PoseArray>("/waypoints", 10);
        
        path_type_ = this->declare_parameter<std::string>("path_type", "turns");
        
        // Publish once, then every 5 seconds
        timer_ = this->create_wall_timer(std::chrono::seconds(5),
            std::bind(&WaypointPublisher::publish_waypoints, this));
        
        // Publish immediately on startup
        publish_waypoints();
        
        RCLCPP_INFO(this->get_logger(), "WaypointPublisher started with path_type='%s'", path_type_.c_str());
    }

private:
    void publish_waypoints() {
        geometry_msgs::msg::PoseArray msg;
        msg.header.stamp = this->now();
        msg.header.frame_id = "odom";

        if (path_type_ == "line") {
            // Simple straight line
            for (int i = 0; i <= 20; i++) {
                geometry_msgs::msg::Pose p;
                p.position.x = 0.25 * i;
                p.position.y = 0.0;
                p.position.z = 0.0;
                p.orientation.w = 1.0;
                msg.poses.push_back(p);
            }
        }
        else if (path_type_ == "circle") {
            // Perfect circle
            int N = 60;  // More points for smoother circle
            double R = 1.5;
            double cx = 2.0, cy = 0.0;
            
            for (int i = 0; i <= N; i++) {
                double theta = 2.0 * M_PI * i / N;
                geometry_msgs::msg::Pose p;
                p.position.x = cx + R * cos(theta);
                p.position.y = cy + R * sin(theta);
                p.position.z = 0.0;
                p.orientation.w = 1.0;
                msg.poses.push_back(p);
            }
        }
        else if (path_type_ == "scurve") {
            // Smooth S-curve
            for (int i = 0; i <= 60; i++) {
                double x = i * 0.1;
                double y = 1.5 * sin(x * 0.8);
                geometry_msgs::msg::Pose p;
                p.position.x = x;
                p.position.y = y;
                p.position.z = 0.0;
                p.orientation.w = 1.0;
                msg.poses.push_back(p);
            }
        }
        else if (path_type_ == "turns") {
            std::vector<std::pair<double, double>> waypoints;
            double side_length = 2.5;  // Reduced from 3.0 for tighter turns
            double step = 0.1;         // Much smaller steps for dense waypoints
            
            // Start at origin
            waypoints.push_back({0.0, 0.0});
            
            // Side 1: Move right (0,0) to (2.5,0)
            for (double x = step; x <= side_length; x += step) {
                waypoints.push_back({x, 0.0});
            }
            
            // CORNER 1: Dense waypoints for 90° turn at (2.5, 0)
            // Add extra waypoints very close to corner
            waypoints.push_back({side_length, 0.05});
            waypoints.push_back({side_length, 0.1});
            waypoints.push_back({side_length, 0.15});
            waypoints.push_back({side_length, 0.2});
            
            // Side 2: Move up (2.5,0.2) to (2.5,2.5)
            for (double y = 0.3; y <= side_length; y += step) {
                waypoints.push_back({side_length, y});
            }
            
            // CORNER 2: Dense waypoints for 90° turn at (2.5, 2.5)  
            waypoints.push_back({side_length - 0.05, side_length});
            waypoints.push_back({side_length - 0.1, side_length});
            waypoints.push_back({side_length - 0.15, side_length});
            waypoints.push_back({side_length - 0.2, side_length});
            
            // Side 3: Move left (2.3,2.5) to (0,2.5)
            for (double x = side_length - 0.3; x >= 0; x -= step) {
                waypoints.push_back({x, side_length});
            }
            
            // CORNER 3: Dense waypoints for 90° turn at (0, 2.5)
            waypoints.push_back({0.0, side_length - 0.05});
            waypoints.push_back({0.0, side_length - 0.1});
            waypoints.push_back({0.0, side_length - 0.15});
            waypoints.push_back({0.0, side_length - 0.2});
            
            // Side 4: Move down (0,2.3) to (0,0.1)
            for (double y = side_length - 0.3; y >= 0.1; y -= step) {
                waypoints.push_back({0.0, y});
            }
            
            // Final approach to start
            waypoints.push_back({0.0, 0.05});
            waypoints.push_back({0.0, 0.0});
            
            // Convert to poses
            for (const auto& wp : waypoints) {
                geometry_msgs::msg::Pose p;
                p.position.x = wp.first;
                p.position.y = wp.second;
                p.position.z = 0.0;
                p.orientation.w = 1.0;
                msg.poses.push_back(p);
            }
            
            RCLCPP_INFO(this->get_logger(), "Generated square path with %zu waypoints for sharp turns", waypoints.size());
        }

        pub_->publish(msg);
        RCLCPP_INFO(this->get_logger(), "Published %zu waypoints (%s)", msg.poses.size(), path_type_.c_str());
    }

    std::string path_type_;
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<WaypointPublisher>());
    rclcpp::shutdown();
    return 0;
}
