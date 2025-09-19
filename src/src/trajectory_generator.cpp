// src/trajectory_generator.cpp
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <vector>
#include <cmath>

using std::vector;

class TrajectoryGeneratorNode : public rclcpp::Node {
public:
  TrajectoryGeneratorNode() : Node("trajectory_generator_cpp") {
    v_max_ = this->declare_parameter<double>("v_max", 0.18);
    a_max_ = this->declare_parameter<double>("a_max", 0.6);
    dt_ = this->declare_parameter<double>("dt", 0.1);
    path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
      "smoothed_path", 10, std::bind(&TrajectoryGeneratorNode::path_cb, this, std::placeholders::_1));
    traj_pub_ = this->create_publisher<nav_msgs::msg::Path>("timed_trajectory", 10);
    RCLCPP_INFO(this->get_logger(), "TrajectoryGenerator ready");
  }

private:
  void path_cb(const nav_msgs::msg::Path::SharedPtr msg) {
    size_t n = msg->poses.size();
    if (n < 2) {
      RCLCPP_WARN(this->get_logger(), "Path too short");
      return;
    }
    vector<double> xs(n), ys(n), s(n);
    for (size_t i=0;i<n;i++){ xs[i]=msg->poses[i].pose.position.x; ys[i]=msg->poses[i].pose.position.y; }
    s[0]=0.0;
    for (size_t i=1;i<n;i++) s[i] = s[i-1] + hypot(xs[i]-xs[i-1], ys[i]-ys[i-1]);
    double total_len = s.back();
    if (total_len <= 0.0) {
      RCLCPP_WARN(this->get_logger(), "Zero-length path");
      return;
    }

    double v_max = v_max_;
    double a = a_max_;
    double t_acc = v_max / a;
    double s_acc = 0.5*a*t_acc*t_acc;
    double t_cruise = 0.0;
    double v_peak = v_max;
    if (2.0*s_acc > total_len) {
      t_acc = sqrt(total_len / a);
      t_cruise = 0.0;
      v_peak = a * t_acc;
    } else {
      double s_cruise = total_len - 2.0*s_acc;
      t_cruise = s_cruise / v_max;
    }
    double t_total = 2.0*t_acc + t_cruise;

    vector<double> times;
    for (double tt=0.0; tt <= t_total + 1e-8; tt += dt_) times.push_back(tt);

    vector<double> svals;
    svals.reserve(times.size());
    for (double tt : times) {
      double sval;
      if (tt < t_acc) sval = 0.5*a*tt*tt;
      else if (tt < t_acc + t_cruise) sval = s_acc + v_peak*(tt - t_acc);
      else {
        double dt2 = tt - (t_acc + t_cruise);
        sval = s_acc + v_peak*t_cruise + v_peak*dt2 - 0.5*a*dt2*dt2;
      }
      svals.push_back(sval);
    }

    // interpolate along s to (x,y)
    vector<double> xs_out, ys_out;
    xs_out.reserve(svals.size()); ys_out.reserve(svals.size());
    for (double sv : svals) {
      size_t idx = 0;
      while (idx + 1 < s.size() && s[idx+1] < sv) ++idx;
      if (idx + 1 >= s.size()) {
        xs_out.push_back(xs.back()); ys_out.push_back(ys.back());
        continue;
      }
      double seg = s[idx+1]-s[idx];
      double frac = (seg > 1e-9) ? ((sv - s[idx]) / seg) : 0.0;
      double x = xs[idx] + frac*(xs[idx+1]-xs[idx]);
      double y = ys[idx] + frac*(ys[idx+1]-ys[idx]);
      xs_out.push_back(x); ys_out.push_back(y);
    }

    nav_msgs::msg::Path out;
    out.header = msg->header;
    rclcpp::Time t0 = this->now();
    for (size_t i=0;i<xs_out.size(); ++i) {
      geometry_msgs::msg::PoseStamped ps;
      ps.header = out.header;
      ps.header.stamp = t0 + rclcpp::Duration::from_seconds(dt_ * (double)i);
      ps.pose.position.x = xs_out[i];
      ps.pose.position.y = ys_out[i];
      ps.pose.position.z = 0.0;
      ps.pose.orientation.w = 1.0;
      out.poses.push_back(ps);
    }
    traj_pub_->publish(out);
    RCLCPP_INFO(this->get_logger(), "Published timed_trajectory with %zu points (duration %.2f s)", out.poses.size(), t_total);
  }

  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr traj_pub_;
  double v_max_, a_max_, dt_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TrajectoryGeneratorNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
