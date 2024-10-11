#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include <vector>
#include <iostream>
#include <fstream>

class TemperatureSubscriber : public rclcpp::Node
{
public:
    TemperatureSubscriber(int days) : Node("temperature_subscriber_node"), days_to_simulate(days), current_day(0)
    {
        subscription_ = this->create_subscription<std_msgs::msg::Float32>(
            "temperature", 10, std::bind(&TemperatureSubscriber::temperature_callback, this, std::placeholders::_1));

        timer_ = this->create_wall_timer(std::chrono::seconds(1), std::bind(&TemperatureSubscriber::simulate_day, this));
    }

private:
    std::vector<float> recorded_temperatures;

    void temperature_callback(const std_msgs::msg::Float32::SharedPtr msg)
    {
        recorded_temperature = msg->data;
        recorded_temperatures.push_back(recorded_temperature);
    }

    void simulate_day()
    {
        if (current_day < days_to_simulate)
        {
            current_day++;
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
        std::ofstream data_file("temperature_data.txt");
        for (size_t i = 0; i < recorded_temperatures.size(); ++i)
        {
            data_file << (i + 1) << " " << recorded_temperatures[i] << "\n";
        }
        data_file.close();

        FILE* gnuplot = popen("gnuplot -persistent", "w");
        fprintf(gnuplot, "set title 'Daily Temperatures'\n");
        fprintf(gnuplot, "set xlabel 'Day'\n");
        fprintf(gnuplot, "set ylabel 'Temperature (°C)'\n");
        fprintf(gnuplot, "set grid\n");
        fprintf(gnuplot, "set xtics 1\n");
        fprintf(gnuplot, "set ytics nomirror\n");
        fprintf(gnuplot, "plot 'temperature_data.txt' using 1:2 with linespoints title 'Temperature'\n");
        fprintf(gnuplot, "pause -1\n");
        pclose(gnuplot);

        std::remove("temperature_data.txt");
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

    int days_to_simulate = 10;

    rclcpp::spin(std::make_shared<TemperatureSubscriber>(days_to_simulate));
    rclcpp::shutdown();
    return 0;
}
