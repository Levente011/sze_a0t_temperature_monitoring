#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include <vector>
#include <iostream>

class TemperatureSubscriber : public rclcpp::Node
{
public:
    TemperatureSubscriber(int days) : Node("temperature_subscriber"), days_to_simulate(days), current_day(0)
    {
        subscription_ = this->create_subscription<std_msgs::msg::Float32>(
            "temperature", 10, std::bind(&TemperatureSubscriber::temperature_callback, this, std::placeholders::_1));
        
        // Simulate one temperature reading per hour (24 readings per day)
        timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&TemperatureSubscriber::simulate_day, this));
    }

private:
    void temperature_callback(const std_msgs::msg::Float32::SharedPtr msg)
    {
        float temp = msg->data;
        
        if (temp < 18.0)
        {
            RCLCPP_INFO(this->get_logger(), "Temperature: %.2f°C - LOW", temp);
        }
        else if (temp >= 18.0 && temp <= 25.0)
        {
            RCLCPP_INFO(this->get_logger(), "Temperature: %.2f°C - NORMAL", temp);
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "Temperature: %.2f°C - HIGH", temp);
        }

        recorded_temperatures.push_back(temp);
    }

    void simulate_day()
    {
        if (current_day < days_to_simulate)
        {
            current_day++;
            RCLCPP_INFO(this->get_logger(), "Simulating day %d out of %d...", current_day, days_to_simulate);
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
        // TODO: make a graph
    }

    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr subscription_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::vector<float> recorded_temperatures;

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
