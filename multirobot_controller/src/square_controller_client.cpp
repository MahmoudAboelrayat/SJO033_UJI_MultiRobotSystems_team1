#include <functional>
#include <future>
#include <memory>
#include <string>
#include <sstream>
#include <chrono>

#include "multi_robot_interface/action/move_to_corner.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/string.hpp>

using namespace std::chrono_literals;

namespace move_to_corner
{

class MoveClient : public rclcpp::Node
{
public:
  using Move = multi_robot_interface::action::MoveToCorner;
  using GoalHandleMove = rclcpp_action::ClientGoalHandle<Move>;

  explicit MoveClient(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("move_to_corner_client", options)
  {
    latest_odoms_.resize(4);
    for (int i = 0; i < 4; ++i) {
      odom_subs_.push_back(this->create_subscription<nav_msgs::msg::Odometry>(
        "/robot" + std::to_string(i + 1) + "/odom", 10,
        [this, i](nav_msgs::msg::Odometry::SharedPtr msg) {
          latest_odoms_[i] = msg;
        }));
    }

    for (int i = 1; i <= 4; ++i) {
      auto client = rclcpp_action::create_client<Move>(
        this, "/robot" + std::to_string(i) + "/move_to_corner");
      clients_.push_back(client);
    }

    signal_sub_ = this->create_subscription<std_msgs::msg::String>(
      "/move_signal", 10,
      std::bind(&MoveClient::send_goal, this, std::placeholders::_1));
  }

  void send_goal(const std_msgs::msg::String::SharedPtr msg)
  {
    RCLCPP_INFO(this->get_logger(), "Received move signal");
    for (int i = 0; i < 4; ++i) {
      auto goal_msg = Move::Goal();
      int next_corner_cw = (i + 1) % 4;
      int next_corner_ccw = (i + 3) % 4;
      if (!latest_odoms_[next_corner_cw]) {
          RCLCPP_WARN(this->get_logger(), "No odometry yet for robot %d", next_corner_cw + 1);
          continue;
      }
      if (msg->data == "CW") {
        RCLCPP_INFO(this->get_logger(), "Robot %d moving to corner %d (CW)", i + 1, next_corner_cw + 1);
        goal_msg.target_pose = *latest_odoms_[next_corner_cw];
      } else if (msg->data == "CCW") {
        RCLCPP_INFO(this->get_logger(), "Robot %d moving to corner %d (CCW)", i + 1, next_corner_ccw + 1);
        goal_msg.target_pose = *latest_odoms_[next_corner_ccw];
      } else {
        RCLCPP_WARN(this->get_logger(), "Unknown move signal: %s", msg->data.c_str());
        return;
      }

      auto send_goal_options = rclcpp_action::Client<Move>::SendGoalOptions();
      send_goal_options.goal_response_callback =
        [](std::shared_ptr<GoalHandleMove> goal_handle) {
          if (!goal_handle) {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Goal rejected");
          } else {
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Goal accepted");
          }
        };

      send_goal_options.result_callback =
        [i](const GoalHandleMove::WrappedResult & result) {
          RCLCPP_INFO(rclcpp::get_logger("rclcpp"),
                      "Robot %d finished with status %d", i + 1, static_cast<int>(result.code));
        };

      clients_[i]->async_send_goal(goal_msg, send_goal_options);
    }
  }


private:
  std::vector<rclcpp_action::Client<Move>::SharedPtr> clients_;
  std::vector<rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr> odom_subs_;
  std::vector<nav_msgs::msg::Odometry::SharedPtr> latest_odoms_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr signal_sub_;
};

}  // namespace move_to_corner

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<move_to_corner::MoveClient>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
