// ...existing code...
#include <chrono>
#include <memory>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

class Talker : public rclcpp::Node
{
public:
    Talker()
    : Node("talker_node"), count_(0)
    {
        // 改為明確使用 rclcpp::QoS(10)
        pub_ = this->create_publisher<std_msgs::msg::String>("chatter", rclcpp::QoS(10));
        timer_ = this->create_wall_timer(500ms, std::bind(&Talker::timer_callback, this));
        RCLCPP_INFO(this->get_logger(), "talker_node started");
    }

private:
    void timer_callback()
    {
        auto msg = std_msgs::msg::String();
        msg.data = "Hello " + std::to_string(count_++);
        RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", msg.data.c_str());
        pub_->publish(msg);
    }

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    size_t count_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Talker>());
    rclcpp::shutdown();
    return 0;
}
// ...existing code...