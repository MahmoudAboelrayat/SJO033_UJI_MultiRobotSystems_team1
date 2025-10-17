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
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

class OdomTransformer : public rclcpp::Node
{
public:
    OdomTransformer()
    : Node("odom_transformer")
    {
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        // Get all topics
        auto topics_and_types = this->get_topic_names_and_types();

        // Extract unique namespaces
        std::set<std::string> robot_namespaces_set;
        std::regex tb_pattern("^/tb_\\d+/");  // Matches "/tb_###/"

        for (const auto &topic : topics_and_types)
        {
            const std::string &topic_name = topic.first;
            std::smatch match;

            if (std::regex_search(topic_name, match, tb_pattern))
            {
                std::string ns = match.str();
                ns.pop_back(); 
                robot_namespaces_set.insert(ns);
            }
        }

        robot_namespaces_ = std::vector<std::string>(
            robot_namespaces_set.begin(), robot_namespaces_set.end()
        );

        // Sort by robot id number so that the smallest number comes first
        std::sort(robot_namespaces_.begin(), robot_namespaces_.end(),
            [](const std::string &a, const std::string &b){
                std::regex tb_pattern("^/tb_(\\d+)");
                std::smatch match_a, match_b;
                int num_a = 0, num_b = 0;

                if (std::regex_search(a, match_a, tb_pattern))
                    num_a = std::stoi(match_a[1].str());
                if (std::regex_search(b, match_b, tb_pattern))
                    num_b = std::stoi(match_b[1].str());

                return num_a < num_b;
            }
        );

        // Check we found all 4 robots
        if (robot_namespaces_.size() < 1) // Change back to 4 for actual use
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

        pose_out = tf_buffer_->transform(pose_in, "map", tf2::durationFromSec(0.1));

        // Copy transformed pose to odom_out
        odom_out.pose.pose = pose_out.pose;

        // Keep original twist (optional: you could transform it too if needed)
        odom_out.twist = msg->twist;

        transformed_pub_[index]->publish(odom_out);
    }
    catch (tf2::TransformException &ex)
    {
        RCLCPP_WARN(this->get_logger(),
                    "Could not transform %s: %s",
                    robot_namespaces_[index].c_str(), ex.what());
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
