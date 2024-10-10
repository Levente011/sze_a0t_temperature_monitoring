#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>

class TemperaturePublisher : public rclcpp::Node
{
public:
    TemperaturePublisher() : Node("temperature_publisher")
    {
        publisher_ = this->create_publisher<std_msgs::msg::Float32>("temperature", 10);
        timer_ = this->create_wall_timer(
            std::chrono::seconds(1), std::bind(&TemperaturePublisher::publish_temperature, this));
    }

private:
    void publish_temperature()
    {
        auto msg = std_msgs::msg::Float32();
        msg.data = 23.5;
        publisher_->publish(msg);
        RCLCPP_INFO(this->get_logger(), "Temperature: '%f'", msg.data);
    }

    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TemperaturePublisher>());
    rclcpp::shutdown();
    return 0;
}
