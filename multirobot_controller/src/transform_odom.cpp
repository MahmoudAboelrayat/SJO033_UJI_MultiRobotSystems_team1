#include <memory>
#include <vector>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

class OdomTransformer : public rclcpp::Node
{
public:
    OdomTransformer()
    : Node("odom_transformer")
    {
        this->declare_parameter<std::vector<std::string>>("robot_namespaces", {"robot1", "robot2", "robot3", "robot4"});
        robot_namespaces_ = this->get_parameter("robot_namespaces").as_string_array();

        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        for (size_t i = 0; i < robot_namespaces_.size(); ++i)
        {
            std::string ns = robot_namespaces_[i];

            odom_subs_.push_back(
                this->create_subscription<nav_msgs::msg::Odometry>(
                    ns + "/odom", 10,
                    [this, i](nav_msgs::msg::Odometry::SharedPtr msg){
                        this->odom_callback(msg, i);
                    }
                )
            );

            transformed_pub_.push_back(
                this->create_publisher<geometry_msgs::msg::PoseStamped>(
                    "/robot" + std::to_string(i+1) + "/odom", 10
                )
            );
        }
    }

private:
    std::vector<std::string> robot_namespaces_;
    std::vector<rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr> odom_subs_;
    std::vector<rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr> transformed_pub_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    void odom_callback(nav_msgs::msg::Odometry::SharedPtr msg, size_t index)
    {
        geometry_msgs::msg::PoseStamped pose_in, pose_out;
        pose_in.header = msg->header;
        pose_in.pose = msg->pose.pose;

        try
        {
            pose_out = tf_buffer_->transform(pose_in, "map", tf2::durationFromSec(0.1));
            transformed_pub_[index]->publish(pose_out);
        }
        catch (tf2::TransformException & ex)
        {
            RCLCPP_WARN(this->get_logger(), "Could not transform %s: %s",
                        robot_namespaces_[index].c_str(), ex.what());
        }
    }
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<OdomTransformer>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
