#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <visualization_msgs/msg/marker.hpp>
#include <vector>
#include <cmath>
#include <algorithm>

double quat_to_yaw(const geometry_msgs::msg::Quaternion &q) {
    tf2::Quaternion quat(q.x, q.y, q.z, q.w);
    tf2::Matrix3x3 m(quat);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    return yaw;
}

class PurePursuitController : public rclcpp::Node {
public:
    PurePursuitController() : Node("pure_pursuit_controller_cpp") {
        base_speed_            = this->declare_parameter<double>("base_speed", 0.18);
        lookahead_distance_    = this->declare_parameter<double>("lookahead_distance", 0.35);
        min_lookahead_         = this->declare_parameter<double>("min_lookahead", 0.25);
        max_lookahead_         = this->declare_parameter<double>("max_lookahead", 0.6);
        goal_tolerance_        = this->declare_parameter<double>("goal_tolerance", 0.12);
        control_frequency_     = this->declare_parameter<double>("control_frequency", 25.0);
        max_angular_vel_       = this->declare_parameter<double>("max_angular_vel", 1.5);
        sharp_turn_threshold_  = this->declare_parameter<double>("sharp_turn_threshold", 1.2);

        path_sub_ = create_subscription<nav_msgs::msg::Path>(
            "/smoothed_path", 10,
            std::bind(&PurePursuitController::path_callback, this, std::placeholders::_1));

        odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 50,
            std::bind(&PurePursuitController::odom_callback, this, std::placeholders::_1));

        cmd_pub_       = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        lookahead_pub_ = create_publisher<visualization_msgs::msg::Marker>("/lookahead_point", 10);

        control_timer_ = create_wall_timer(
            std::chrono::milliseconds(static_cast<int>(1000.0 / control_frequency_)),
            std::bind(&PurePursuitController::control_loop, this));

        RCLCPP_INFO(get_logger(), "Sharp-Turn Pure Pursuit Controller initialized");
    }

private:
    void path_callback(const nav_msgs::msg::Path::SharedPtr msg) {
        path_points_.clear();
        for (const auto &pose : msg->poses) {
            path_points_.push_back({pose.pose.position.x, pose.pose.position.y});
        }
        closest_path_index_ = 0;
        goal_reached_ = false;
        RCLCPP_INFO(get_logger(), "Received path with %zu points", path_points_.size());
    }

    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        current_x_   = msg->pose.pose.position.x;
        current_y_   = msg->pose.pose.position.y;
        current_yaw_ = quat_to_yaw(msg->pose.pose.orientation);
        odom_received_ = true;
    }

    int find_closest_point() {
        if (path_points_.empty()) return 0;
        double min_dist = 1e9;
        int idx = closest_path_index_;
        int start = std::max(0, idx - 3);
        int end   = std::min((int)path_points_.size(), idx + 15);
        for (int i = start; i < end; i++) {
            double dx = path_points_[i][0] - current_x_;
            double dy = path_points_[i][1] - current_y_;
            double d  = std::hypot(dx, dy);
            if (d < min_dist) {
                min_dist = d;
                idx = i;
            }
        }
        return idx;
    }

    double calculate_path_curvature(int i) {
        if (i < 1 || i >= (int)path_points_.size()-1) return 0.0;
        auto &p1 = path_points_[i-1];
        auto &p2 = path_points_[i];
        auto &p3 = path_points_[i+1];
        double area = std::abs(
            (p1[0]*(p2[1]-p3[1]) +
             p2[0]*(p3[1]-p1[1]) +
             p3[0]*(p1[1]-p2[1])) * 0.5);
        double a = std::hypot(p2[0]-p3[0], p2[1]-p3[1]);
        double b = std::hypot(p1[0]-p3[0], p1[1]-p3[1]);
        double c = std::hypot(p1[0]-p2[0], p1[1]-p2[1]);
        if (a<1e-6||b<1e-6||c<1e-6) return 0.0;
        return 4.0 * area / (a*b*c);
    }

    std::pair<double,double> find_lookahead_point() {
        if (path_points_.empty()) return {current_x_, current_y_};
        closest_path_index_ = find_closest_point();
        double kappa = calculate_path_curvature(closest_path_index_);
        double Ld = lookahead_distance_;
        if (std::abs(kappa) > sharp_turn_threshold_) {
            Ld = min_lookahead_*0.7;
        } else {
            Ld = std::clamp(lookahead_distance_-kappa*0.3, min_lookahead_, max_lookahead_);
        }
        for (size_t i = closest_path_index_; i < path_points_.size(); i++) {
            double dx = path_points_[i][0]-current_x_;
            double dy = path_points_[i][1]-current_y_;
            if (std::hypot(dx,dy) >= Ld) {
                return {path_points_[i][0], path_points_[i][1]};
            }
        }
        return {path_points_.back()[0], path_points_.back()[1]};
    }

    void publish_lookahead_marker(double x, double y) {
        visualization_msgs::msg::Marker m;
        m.header.frame_id = "odom";
        m.header.stamp    = now();
        m.ns   = "lookahead";
        m.id   = 0;
        m.type = visualization_msgs::msg::Marker::SPHERE;
        m.action = visualization_msgs::msg::Marker::ADD;
        m.pose.position.x = x;
        m.pose.position.y = y;
        m.pose.orientation.w = 1.0;
        m.scale.x = m.scale.y = m.scale.z = 0.12;
        m.color.r = 1.0; m.color.a = 1.0;
        lookahead_pub_->publish(m);
    }

    void control_loop() {
        if (!odom_received_ || path_points_.empty() || goal_reached_) return;

        // Goal check
        double dxg = path_points_.back()[0] - current_x_;
        double dyg = path_points_.back()[1] - current_y_;
        if (std::hypot(dxg, dyg) < goal_tolerance_) {
            cmd_pub_->publish(geometry_msgs::msg::Twist{});
            if (!goal_reached_) {
                goal_reached_ = true;
                RCLCPP_INFO(get_logger(), "Goal reached");
            }
            return;
        }

        // Compute lookahead and curvature
        closest_path_index_ = find_closest_point();
        auto [lx, ly] = find_lookahead_point();
        double kappa = calculate_path_curvature(closest_path_index_);

        // Sharp-turn pivot
        if (std::abs(kappa) > sharp_turn_threshold_*1.5) {
            size_t next_i = std::min(closest_path_index_+1, path_points_.size()-1);
            double desired_yaw = std::atan2(
                path_points_[next_i][1]-current_y_,
                path_points_[next_i][0]-current_x_);
            double yaw_err = std::remainder(desired_yaw - current_yaw_, 2*M_PI);
            geometry_msgs::msg::Twist cmd;
            cmd.angular.z = std::clamp(yaw_err*1.5, -max_angular_vel_, max_angular_vel_);
            cmd_pub_->publish(cmd);
            return;
        }

        // Normal pursuit
        double dx = lx-current_x_, dy = ly-current_y_;
        double lx_r = std::cos(-current_yaw_)*dx - std::sin(-current_yaw_)*dy;
        double ly_r = std::sin(-current_yaw_)*dx + std::cos(-current_yaw_)*dy;
        double Ld = std::hypot(lx_r, ly_r);
        if (Ld < 1e-4) return;
        double curvature = 2.0*ly_r/(Ld*Ld);
        double omega = std::clamp(curvature*base_speed_, -max_angular_vel_, max_angular_vel_);
        double sf = 1.0/(1.0+std::abs(curvature)*2.0);
        sf = std::clamp(sf, 0.4, 1.0);
        geometry_msgs::msg::Twist cmd;
        cmd.linear.x  = base_speed_*sf;
        cmd.angular.z = omega;
        cmd_pub_->publish(cmd);
    }

    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr lookahead_pub_;
    rclcpp::TimerBase::SharedPtr control_timer_;

    std::vector<std::array<double,2>> path_points_;
    size_t closest_path_index_{0};
    double current_x_{0}, current_y_{0}, current_yaw_{0};
    bool odom_received_{false}, goal_reached_{false};
    double base_speed_, lookahead_distance_, min_lookahead_, max_lookahead_;
    double goal_tolerance_, control_frequency_, max_angular_vel_, sharp_turn_threshold_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PurePursuitController>());
    rclcpp::shutdown();
    return 0;
}
