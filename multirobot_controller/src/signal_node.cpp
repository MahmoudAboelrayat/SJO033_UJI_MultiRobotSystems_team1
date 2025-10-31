#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

class MoveSignalPublisher : public rclcpp::Node
{
public:
    MoveSignalPublisher(const std::string & direction)
    : Node("move_signal_publisher")
    {
        // Validate input
        std::string dir = direction;
        if (dir != "CW" && dir != "CCW") {
            RCLCPP_WARN(this->get_logger(), "Invalid direction '%s', defaulting to 'CCW'.", dir.c_str());
            dir = "CCW";
        }

        // Publisher
        auto publisher = this->create_publisher<std_msgs::msg::String>("/move_signal", 10);

        // Publish once
        std_msgs::msg::String msg;
        msg.data = dir;
        publisher->publish(msg);
        RCLCPP_INFO(this->get_logger(), "Published move signal: '%s'", msg.data.c_str());

        // Give a short delay so subscriber can receive
        rclcpp::sleep_for(std::chrono::milliseconds(500));
        rclcpp::shutdown();
    }
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);

    // Check if user provided argument, otherwise default to CCW
    std::string direction = "CCW";
    if (argc > 1) {
        direction = argv[1];
    }

    auto node = std::make_shared<MoveSignalPublisher>(direction);
    rclcpp::spin(node);

    return 0;
}
