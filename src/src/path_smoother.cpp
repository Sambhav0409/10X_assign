#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include <vector>
#include <cmath>
#include <algorithm>

using std::placeholders::_1;

// Natural Cubic Spline Implementation
class CubicSpline {
public:
    std::vector<double> x, y, a, b, c, d;
    
    void fit(const std::vector<double>& x_points, const std::vector<double>& y_points) {
        int n = x_points.size();
        if (n < 2) return;
        
        x = x_points;
        y = y_points;
        a = y;
        
        std::vector<double> h(n-1), alpha(n);
        for (int i = 0; i < n-1; i++) {
            h[i] = x[i+1] - x[i];
        }
        
        for (int i = 1; i < n-1; i++) {
            alpha[i] = (3.0/h[i])*(a[i+1] - a[i]) - (3.0/h[i-1])*(a[i] - a[i-1]);
        }
        
        std::vector<double> l(n), mu(n), z(n);
        l[0] = 1.0; mu[0] = 0.0; z[0] = 0.0;
        
        for (int i = 1; i < n-1; i++) {
            l[i] = 2.0*(x[i+1] - x[i-1]) - h[i-1]*mu[i-1];
            mu[i] = h[i]/l[i];
            z[i] = (alpha[i] - h[i-1]*z[i-1])/l[i];
        }
        
        l[n-1] = 1.0; z[n-1] = 0.0;
        c.resize(n); d.resize(n); b.resize(n);
        c[n-1] = 0.0;
        
        for (int j = n-2; j >= 0; j--) {
            c[j] = z[j] - mu[j]*c[j+1];
            b[j] = (a[j+1] - a[j])/h[j] - h[j]*(c[j+1] + 2.0*c[j])/3.0;
            d[j] = (c[j+1] - c[j])/(3.0*h[j]);
        }
    }
    
    double interpolate(double xi) {
        if (x.empty()) return 0.0;
        
        int j = 0;
        for (int i = 0; i < (int)x.size()-1; i++) {
            if (xi >= x[i] && xi <= x[i+1]) {
                j = i;
                break;
            }
        }
        
        if (xi < x[0]) j = 0;
        if (xi > x.back()) j = x.size()-2;
        
        double dx = xi - x[j];
        return a[j] + b[j]*dx + c[j]*dx*dx + d[j]*dx*dx*dx;
    }
};

class PathSmoother : public rclcpp::Node {
public:
    PathSmoother() : Node("path_smoother_cpp") {
        sub_waypoints_ = this->create_subscription<geometry_msgs::msg::PoseArray>(
            "/waypoints", 10, std::bind(&PathSmoother::waypoint_callback, this, _1));
        
        pub_path_ = this->create_publisher<nav_msgs::msg::Path>("/smoothed_path", 10);
        pub_marker_ = this->create_publisher<visualization_msgs::msg::Marker>("/path_marker", 10);
        
        // Parameters
        num_samples_ = this->declare_parameter<int>("num_samples", 200);
        
        RCLCPP_INFO(this->get_logger(), "PathSmoother initialized with %d samples", num_samples_);
    }

private:
    void waypoint_callback(const geometry_msgs::msg::PoseArray::SharedPtr msg) {
        if (msg->poses.size() < 2) {
            RCLCPP_WARN(this->get_logger(), "Need at least 2 waypoints for smoothing");
            return;
        }
        
        // Extract waypoints
        std::vector<double> x_waypoints, y_waypoints, t_params;
        for (size_t i = 0; i < msg->poses.size(); i++) {
            x_waypoints.push_back(msg->poses[i].position.x);
            y_waypoints.push_back(msg->poses[i].position.y);
            t_params.push_back(static_cast<double>(i));  // Parameter t
        }
        
        // Create cubic splines for x(t) and y(t)
        CubicSpline spline_x, spline_y;
        spline_x.fit(t_params, x_waypoints);
        spline_y.fit(t_params, y_waypoints);
        
        // Generate smooth path
        nav_msgs::msg::Path path;
        path.header.frame_id = "odom";
        path.header.stamp = this->now();
        
        visualization_msgs::msg::Marker line;
        line.header = path.header;
        line.ns = "smooth_path";
        line.id = 0;
        line.type = visualization_msgs::msg::Marker::LINE_STRIP;
        line.action = visualization_msgs::msg::Marker::ADD;
        line.scale.x = 0.05;
        line.color.a = 1.0;
        line.color.g = 1.0; // Green
        
        // Sample the spline
        double t_max = static_cast<double>(t_params.size() - 1);
        for (int i = 0; i < num_samples_; i++) {
            double t = (t_max * i) / (num_samples_ - 1);
            
            geometry_msgs::msg::PoseStamped pose;
            pose.header = path.header;
            pose.pose.position.x = spline_x.interpolate(t);
            pose.pose.position.y = spline_y.interpolate(t);
            pose.pose.position.z = 0.0;
            pose.pose.orientation.w = 1.0;
            
            path.poses.push_back(pose);
            
            // Add to marker
            geometry_msgs::msg::Point p;
            p.x = pose.pose.position.x;
            p.y = pose.pose.position.y;
            p.z = 0.0;
            line.points.push_back(p);
        }
        
        pub_path_->publish(path);
        pub_marker_->publish(line);
        
        RCLCPP_INFO(this->get_logger(), "Published smoothed path with %zu points", path.poses.size());
    }
    
    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr sub_waypoints_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_path_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_marker_;
    int num_samples_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PathSmoother>());
    rclcpp::shutdown();
    return 0;
}
