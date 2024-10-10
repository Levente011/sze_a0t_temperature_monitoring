#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include <vector>
#include <iostream>

class TemperatureSubscriber : public rclcpp::Node
{
public:
    TemperatureSubscriber(int days) : Node("temperature_subscriber_node"), days_to_simulate(days), current_day(0)
    {
        subscription_ = this->create_subscription<std_msgs::msg::Float32>(
            "temperature", 10, std::bind(&TemperatureSubscriber::temperature_callback, this, std::placeholders::_1));

        // Simulate one temperature reading per day (set to 1 second for testing purposes)
        timer_ = this->create_wall_timer(std::chrono::seconds(1), std::bind(&TemperatureSubscriber::simulate_day, this));
    }

private:
    void temperature_callback(const std_msgs::msg::Float32::SharedPtr msg)
    {
        recorded_temperature = msg->data;
    }

    void simulate_day()
    {
        if (current_day < days_to_simulate)
        {
            current_day++;
            // Print only the necessary information (temperature and day status)
            if (recorded_temperature < 18.0)
            {
                RCLCPP_INFO(this->get_logger(), "Day %d: Temperature %.2f°C - LOW", current_day, recorded_temperature);
            }
            else if (recorded_temperature >= 18.0 && recorded_temperature <= 25.0)
            {
                RCLCPP_INFO(this->get_logger(), "Day %d: Temperature %.2f°C - NORMAL", current_day, recorded_temperature);
            }
            else
            {
                RCLCPP_INFO(this->get_logger(), "Day %d: Temperature %.2f°C - HIGH", current_day, recorded_temperature);
            }
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "Simulation finished. Generating graph...");
            generate_graph();
            rclcpp::shutdown();
        }
    }

    void generate_graph()
    {
        // TODO: Implement code to generate a graph
    }

    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr subscription_;
    rclcpp::TimerBase::SharedPtr timer_;
    float recorded_temperature = 0.0;

    int days_to_simulate;
    int current_day;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    // Set the number of days to simulate (you can change this value)
    int days_to_simulate = 10;

    rclcpp::spin(std::make_shared<TemperatureSubscriber>(days_to_simulate));
    rclcpp::shutdown();
    return 0;
}
