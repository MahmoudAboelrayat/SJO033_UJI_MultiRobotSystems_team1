#include <memory>
#include <thread>
#include <cmath>
#include <vector>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "multi_robot_interface/action/move_to_corner.hpp"

namespace move_to_corner
{
class MoveServer : public rclcpp::Node
{
public:

  using Move = multi_robot_interface::action::MoveToCorner;
  using GoalHandleMove = rclcpp_action::ServerGoalHandle<Move>;

  explicit MoveServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("move_to_corner_server", options)
  {
    this->declare_parameter<double>("kp", 0.5);
    this->declare_parameter<double>("distance_threshold", 0.05);
    this->declare_parameter<double>("angle_th", 0.01);
    this->declare_parameter<double>("max_linear_speed", 0.2);
    this->declare_parameter<double>("max_angular_speed", 0.5);

    action_server_ = rclcpp_action::create_server<Move>(
      this,
      "move_to_corner",
      std::bind(&MoveServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&MoveServer::handle_cancel, this, std::placeholders::_1),
      std::bind(&MoveServer::handle_accepted, this, std::placeholders::_1));

    vel_pub = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "odom", 10,
      std::bind(&MoveServer::current_odom_callback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "MoveToCorner action server started with kp= %f and waiting for goals...",
      this->get_parameter("kp").as_double());
  }

private:
  rclcpp_action::Server<Move>::SharedPtr action_server_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub;

  double current_x_{0.0}, current_y_{0.0}, current_yaw_{0.0};

  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const Move::Goal> goal)
  {
    RCLCPP_INFO(this->get_logger(), "Received goal");
    (void)uuid;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandleMove> goal_handle)
  {
    RCLCPP_WARN(this->get_logger(), "Received cancel request");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandleMove> goal_handle)
  {
    std::thread{std::bind(&MoveServer::execute, this, goal_handle)}.detach();
  }

  void current_odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    current_x_ = msg->pose.pose.position.x;
    current_y_ = msg->pose.pose.position.y;

    double qx = msg->pose.pose.orientation.x;
    double qy = msg->pose.pose.orientation.y;
    double qz = msg->pose.pose.orientation.z;
    double qw = msg->pose.pose.orientation.w;

    double siny_cosp = 2.0 * (qw * qz + qx * qy);
    double cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz);
    current_yaw_ = std::atan2(siny_cosp, cosy_cosp);
  }

  void execute(const std::shared_ptr<GoalHandleMove> goal_handle)
  {
    geometry_msgs::msg::Twist velocity_msg;
    RCLCPP_INFO(this->get_logger(), "Starting move to corner...");
    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<Move::Feedback>();
    auto result = std::make_shared<Move::Result>();

    float target_x = goal->target_pose.pose.pose.position.x;
    float target_y = goal->target_pose.pose.pose.position.y;
    
    double kp = this->get_parameter("kp").as_double();
    double th = this->get_parameter("distance_threshold").as_double();
    double yaw_th = this->get_parameter("angle_th").as_double();
    double max_linear_speed = this->get_parameter("max_linear_speed").as_double();
    double max_angular_speed = this->get_parameter("max_angular_speed").as_double();

    rclcpp::Rate loop_rate(10.0);

    while (rclcpp::ok()) {
      double dx = target_x - current_x_;
      double dy = target_y - current_y_;
      double distance = std::sqrt(dx*dx + dy*dy);
      if (distance < th) break;

      double angle_to_target = std::atan2(dy, dx);
      double angle_diff = angle_to_target - current_yaw_;
      while (angle_diff > M_PI) angle_diff -= 2 * M_PI;
      while (angle_diff < -M_PI) angle_diff += 2 * M_PI;

      float u = 0.0;
      float yaw_rate = 0.0;

      if (std::fabs(angle_diff) > yaw_th) {
        u = 0.0;
        yaw_rate = kp * angle_diff;
        if (yaw_rate > max_angular_speed) yaw_rate = max_angular_speed;
        if (yaw_rate < -max_angular_speed) yaw_rate = -max_angular_speed;
      } else {
        u = kp * distance;
        if (u > max_linear_speed) u = max_linear_speed;
        yaw_rate = 0.0;
      }

      velocity_msg.linear.x = u;
      velocity_msg.angular.z = yaw_rate;
      vel_pub->publish(velocity_msg);

      feedback->distance_remaining = distance;
      goal_handle->publish_feedback(feedback);

      loop_rate.sleep();
    }

    velocity_msg.linear.x = 0.0;
    velocity_msg.angular.z = 0.0;
    vel_pub->publish(velocity_msg);

    result->success = true;
    goal_handle->succeed(result);
    RCLCPP_INFO(this->get_logger(), "Reached target corner!");
  }
};
}  

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<move_to_corner::MoveServer>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
