#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include <random>  // For random number generation

class TemperaturePublisher : public rclcpp::Node
{
public:
    TemperaturePublisher() : Node("temperature_publisher"), gen(rd()), dist(-20.0, 50.0)
    {
        publisher_ = this->create_publisher<std_msgs::msg::Float32>("temperature", 10);
        timer_ = this->create_wall_timer(
            std::chrono::seconds(1), std::bind(&TemperaturePublisher::publish_temperature, this));
    }

private:
    void publish_temperature()
    {
        auto msg = std_msgs::msg::Float32();
        msg.data = dist(gen);
        publisher_->publish(msg);
    }

    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;

    std::random_device rd;
    std::mt19937 gen;
    std::uniform_real_distribution<float> dist;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TemperaturePublisher>());
    rclcpp::shutdown();
    return 0;
}
