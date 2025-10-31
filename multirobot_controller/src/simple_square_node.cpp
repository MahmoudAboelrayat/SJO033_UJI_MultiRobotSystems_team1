#include <memory>
#include <cmath>
#include <vector>
#include <string>
#include <sstream>
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/string.hpp"

using std::placeholders::_1;

class SquareFormationController : public rclcpp::Node
{
public:
  SquareFormationController()
  : Node("square_formation_controller")
  {
    this->declare_parameter<std::string>("robot_namespaces", "robot1,robot2,robot3,robot4");
    this->declare_parameter<double>("kp", 0.5);
    this->declare_parameter<double>("angle_kp", 0.8);
    this->declare_parameter<double>("distance_threshold", 0.05);
    this->declare_parameter<double>("angle_threshold", 0.05);
    this->declare_parameter<double>("max_linear_speed", 0.2);
    this->declare_parameter<double>("max_angular_speed", 0.5);

    std::string ns_list = this->get_parameter("robot_namespaces").as_string();
    std::stringstream ss(ns_list);
    std::string ns;
    while (std::getline(ss, ns, ',')) robot_namespaces_.push_back(ns);

    size_t n = robot_namespaces_.size();
    latest_odoms_.resize(n);
    vel_pubs_.resize(n);

    this->declare_parameter<bool>("simulation", true);

    bool simulation = this->get_parameter("simulation").as_bool();
    std::string odom_frame = simulation ? "/odom" : "/odom_map_frame";

    if (simulation) {
      RCLCPP_INFO(this->get_logger(), "Operating in simulation mode.");
    } else {
      RCLCPP_INFO(this->get_logger(), "Operating in real robot mode.");
    }

    for (size_t i = 0; i < n; ++i) {
      odom_subs_.push_back(
        this->create_subscription<nav_msgs::msg::Odometry>(
          "/" + robot_namespaces_[i] + odom_frame, 10,
          [this, i](nav_msgs::msg::Odometry::SharedPtr msg) {
            latest_odoms_[i] = msg;
          })
      );
      vel_pubs_[i] = this->create_publisher<geometry_msgs::msg::Twist>(
        "/" + robot_namespaces_[i] + "/cmd_vel", 10);
    }

    signal_sub_ = this->create_subscription<std_msgs::msg::String>(
      "/move_signal", 10, std::bind(&SquareFormationController::move_robots, this, _1));

    RCLCPP_INFO(this->get_logger(), "Square formation controller ready.");
  }

private:
  std::vector<std::string> robot_namespaces_;
  std::vector<nav_msgs::msg::Odometry::SharedPtr> latest_odoms_;
  std::vector<rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr> odom_subs_;
  std::vector<rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr> vel_pubs_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr signal_sub_;
  std::vector<std::string> states_ = {"init", "init", "init", "init"};

  void move_robots(const std_msgs::msg::String::SharedPtr msg)
  {
    if (msg->data != "CW" && msg->data != "CCW") {
      RCLCPP_WARN(this->get_logger(), "Invalid move signal: %s", msg->data.c_str());
      return;
    }

    size_t n = robot_namespaces_.size();
    if (n < 2) return;

    struct TargetPose {
      double x;
      double y;
      double yaw;
    };

    std::vector<TargetPose> targets(n);

    for (size_t i = 0; i < n; i++) {
      int next = (msg->data == "CW") ? (i + n - 1) % n : (i + 1) % n;
      if (!latest_odoms_[next]) {
        RCLCPP_WARN(this->get_logger(), "Missing odometry for target %s.", robot_namespaces_[next].c_str());
        continue;
      }
      targets[i].x = latest_odoms_[next]->pose.pose.position.x;
      targets[i].y = latest_odoms_[next]->pose.pose.position.y;
      targets[i].yaw = getYaw(latest_odoms_[next]);
    }

    for (size_t i = 0; i < n; i++) {
      if (!latest_odoms_[i]) {
        RCLCPP_WARN(this->get_logger(), "Missing odometry for robot %s.", robot_namespaces_[i].c_str());
        continue;
      }
      std::thread(
        &SquareFormationController::move_to_target, this,
        i, targets[i].x, targets[i].y, targets[i].yaw
      ).detach();
    }
  }

  void move_to_target(size_t i, double target_x, double target_y, double target_yaw)
{
  auto kp = this->get_parameter("kp").as_double();
  auto th = this->get_parameter("distance_threshold").as_double();
  auto yaw_th = this->get_parameter("angle_threshold").as_double();
  auto max_linear_speed = this->get_parameter("max_linear_speed").as_double();
  auto max_angular_speed = this->get_parameter("max_angular_speed").as_double();


  double ki = 0.0;  // You can tune this for yaw integral term
  double theta_ei = 0.0;
  double distance_ei = 0.0;

  rclcpp::Rate loop_rate(10);
  geometry_msgs::msg::Twist velocity_msg;

  RCLCPP_INFO(this->get_logger(),
              "%s moving toward fixed goal (x=%.2f, y=%.2f, yaw=%.2f)",
              robot_namespaces_[i].c_str(), target_x, target_y, target_yaw);

  while (rclcpp::ok() && latest_odoms_[i]) {
    double current_x = latest_odoms_[i]->pose.pose.position.x;
    double current_y = latest_odoms_[i]->pose.pose.position.y;
    double current_yaw = getYaw(latest_odoms_[i]);

    double dx = target_x - current_x;
    double dy = target_y - current_y;
    double distance = std::sqrt(dx * dx + dy * dy);

    // if (distance < th) break;

    double angle_to_target = std::atan2(dy, dx);
    double angle_diff = angle_to_target - current_yaw;
    while (angle_diff > M_PI) angle_diff -= 2 * M_PI;
    while (angle_diff < -M_PI) angle_diff += 2 * M_PI;

    double u = 0.0;
    double yaw_rate = 0.0;

    if (std::fabs(angle_diff) > yaw_th) {
      u = 0.0;
      theta_ei += angle_diff * 0.1;  // Simple integral
      yaw_rate = kp * angle_diff + ki * theta_ei;

      yaw_rate = std::clamp(yaw_rate, -max_angular_speed, max_angular_speed);

      RCLCPP_WARN(this->get_logger(),
                  "%s rotating to face target: angle_diff=%.2f, yaw_rate=%.2f",
                  robot_namespaces_[i].c_str(), angle_diff, yaw_rate);

      this->states_[i] = "Rotating";
    }else if(this->states_[i] == "Rotating") {
      this->states_[i] = "Waiting";
      RCLCPP_INFO(this->get_logger(),
                  "%s finished rotating, now waiting for others.",
                  robot_namespaces_[i].c_str());
      RCLCPP_INFO(this->get_logger(),
                  "states: [%s, %s, %s, %s]",
                  this->states_[0].c_str(), this->states_[1].c_str(), this->states_[2].c_str(), this->states_[3].c_str());
    }else if(checkValue(this->states_,"Rotating") && distance >= th) {
      // Reset integral when switching from rotating to moving
      theta_ei = 0.0;
      this->states_[i] = "MovingForward";
      u = kp * distance;
      if (u > max_linear_speed) u = max_linear_speed;

      yaw_rate = 0.5 * kp * angle_diff;
      yaw_rate = std::clamp(yaw_rate, -max_angular_speed, max_angular_speed);}
    else if (this->states_[i] == "MovingForward"){
      this->states_[i] = "Waiting";
    }else if (checkValue(this->states_,"MovingForward") && distance < th) {
      this->states_[i] = "Reached goal";
      u = 0.0;
      yaw_rate = 0.0;
      break;
    }

    velocity_msg.linear.x = u;
    velocity_msg.angular.z = yaw_rate;
    vel_pubs_[i]->publish(velocity_msg);

    loop_rate.sleep();
  }

  // Stop movement
  theta_ei = 0.0;
  distance_ei = 0.0;
  velocity_msg.linear.x = 0.0;
  velocity_msg.angular.z = 0.0;
  vel_pubs_[i]->publish(velocity_msg);

  RCLCPP_INFO(this->get_logger(),
              "%s reached target position, aligning orientation...",
              robot_namespaces_[i].c_str());

  // Align final yaw
  while (rclcpp::ok() && latest_odoms_[i]) {
    double current_yaw = getYaw(latest_odoms_[i]);
    double yaw_diff = target_yaw - current_yaw;

    while (yaw_diff > M_PI) yaw_diff -= 2.0 * M_PI;
    while (yaw_diff < -M_PI) yaw_diff += 2.0 * M_PI;

    if (std::fabs(yaw_diff) < yaw_th) break;

    velocity_msg.linear.x = 0.0;
    velocity_msg.angular.z = kp * yaw_diff;
    velocity_msg.angular.z = std::clamp(velocity_msg.angular.z,
                                        -max_angular_speed, max_angular_speed);

    vel_pubs_[i]->publish(velocity_msg);
    loop_rate.sleep();
  }

  // Final stop
  velocity_msg.linear.x = 0.0;
  velocity_msg.angular.z = 0.0;
  vel_pubs_[i]->publish(velocity_msg);

  RCLCPP_INFO(this->get_logger(),
              "%s orientation aligned! Target reached (%.2f, %.2f, %.2f)",
              robot_namespaces_[i].c_str(), target_x, target_y, target_yaw);
}


  double getYaw(const nav_msgs::msg::Odometry::SharedPtr &odom)
  {
    double qx = odom->pose.pose.orientation.x;
    double qy = odom->pose.pose.orientation.y;
    double qz = odom->pose.pose.orientation.z;
    double qw = odom->pose.pose.orientation.w;
    return std::atan2(2.0 * (qw * qz + qx * qy),
                      1.0 - 2.0 * (qy * qy + qz * qz));
  }

  double normalizeAngle(double a)
  {
    while (a > M_PI) a -= 2.0 * M_PI;
    while (a < -M_PI) a += 2.0 * M_PI;
    return a;
  }

  double checkValue(std::vector<std::string> states, std::string value)
  {
    for (const auto &state : states) {
      if (state == value) return false;
    }
    return true;
  }
};


int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SquareFormationController>());
  rclcpp::shutdown();
  return 0;
}
