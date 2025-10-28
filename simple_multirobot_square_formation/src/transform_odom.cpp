#include <memory>
#include <vector>
#include <string>
#include <set>
#include <regex>
#include <algorithm>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
// #include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

class OdomTransformer : public rclcpp::Node
{
public:
    OdomTransformer()
    : Node("odom_transformer")
    {
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        this->declare_parameter<double>("side_length", 1.0);
        this->declare_parameter<bool>("lagging", false);
        this->side_length = this->get_parameter("side_length").as_double();

        this->declare_parameter<std::string>("robot_namespaces", "robot1,robot2,robot3,robot4");
        std::string ns_string = this->get_parameter("robot_namespaces").as_string();
        std::stringstream ss(ns_string);
        std::string ns;
        while (std::getline(ss, ns, ',')) {
            robot_namespaces_.push_back(ns);
        }
        
        // Check we found all 4 robots
        if (robot_namespaces_.size() < 4) // Change back to 4 for actual use
        {
            RCLCPP_ERROR(this->get_logger(),
                         "Could not find all four robot namespaces. Found %zu. Make sure all robots are running.",
                         robot_namespaces_.size());
            throw std::runtime_error("Missing robot namespaces");
        }
        else
        {
            RCLCPP_INFO(this->get_logger(),
                        "Found %zu robot namespaces.", robot_namespaces_.size());
        }

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

            // Publish transformed PoseStamped to /robotX/odom
            transformed_pub_.push_back(
                this->create_publisher<nav_msgs::msg::Odometry>(
                    ns + "/odom_map_frame", 10
                )
            );
        }
    }

private:
    std::vector<std::string> robot_namespaces_;
    std::vector<rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr> odom_subs_;
    std::vector<rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr> transformed_pub_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    double side_length;

    void odom_callback(nav_msgs::msg::Odometry::SharedPtr msg, size_t index)
{
    nav_msgs::msg::Odometry odom_out;
    odom_out.header = msg->header;
    odom_out.header.frame_id = "map";  // transformed frame
    odom_out.child_frame_id = msg->child_frame_id;

    try
    {
        // Transform the pose part to the "map" frame
        geometry_msgs::msg::PoseStamped pose_in, pose_out;
        pose_in.header = msg->header;
        pose_in.pose = msg->pose.pose;

        pose_out = tf_buffer_->transform(pose_in, "map", tf2::durationFromSec(1.0));

        // Copy transformed pose to odom_out
        odom_out.pose.pose = pose_out.pose;

        // Keep original twist (optional: you could transform it too if needed)
        odom_out.twist = msg->twist;

        transformed_pub_[index]->publish(odom_out);
    }
    catch (tf2::TransformException &ex)
    {
        RCLCPP_WARN(this->get_logger(),
                    "Could not transform %s: %s. Using static fallback.",
                    robot_namespaces_[index].c_str(), ex.what());

        float x = this ->side_length * ((index+1 < 4 && index+1 > 1) ? 0.5 : -0.5);
        float y = this ->side_length * ((index+1 > 2) ? 0.5 : -0.5);
        float yaw = (index+1 -1) * (M_PI / 2.0) - (M_PI / 2.0); // -90, 0, 90, 180 degrees in radians 
        tf2::Transform static_tf;
        tf2::Vector3 translation(x, y, 0.0);      // x,y,z
        tf2::Quaternion rotation;
        rotation.setRPY(0.0, 0.0, yaw);              // roll, pitch, yaw
        static_tf.setOrigin(translation);
        static_tf.setRotation(rotation);

        // Convert odom pose to tf2
        tf2::Transform odom_pose_tf;
        tf2::fromMsg(msg->pose.pose, odom_pose_tf);

        // Apply static transform
        tf2::Transform transformed_pose = static_tf * odom_pose_tf;

        // Convert back to ROS message
        odom_out.pose.pose.position.x = transformed_pose.getOrigin().x();
        odom_out.pose.pose.position.y = transformed_pose.getOrigin().y();
        odom_out.pose.pose.position.z = transformed_pose.getOrigin().z();

        tf2::Quaternion q = transformed_pose.getRotation();
        odom_out.pose.pose.orientation.x = q.x();
        odom_out.pose.pose.orientation.y = q.y();
        odom_out.pose.pose.orientation.z = q.z();
        odom_out.pose.pose.orientation.w = q.w();


        // Keep original twist
        odom_out.twist = msg->twist;

        transformed_pub_[index]->publish(odom_out);
    }
}

};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<OdomTransformer>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
