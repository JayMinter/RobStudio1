#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

class LaserScanNth : public rclcpp::Node
{
public:
    LaserScanNth()
        : Node("laser_scan_nth")
    {

        
        
        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10, std::bind(&LaserScanNth::scanCallback, this, std::placeholders::_1));

        scan_pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>("/nth_scan", 10);

        this->declare_parameter<int>("nth_value", 5);

        this->get_parameter("nth_value", n);

        RCLCPP_INFO(this->get_logger(), "The turtlebot will republish every  %d scans", n);

    }

private:
    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan)

    {
        auto scan_msg = std::make_shared<sensor_msgs::msg::LaserScan>(*scan);
        std::vector<float> nthScan;

        scan_msg->header = scan->header; 
        scan_msg->angle_min = scan->angle_min;
        scan_msg->angle_max = scan->angle_max;
        scan_msg->angle_increment = scan->angle_increment;
        scan_msg->time_increment = scan->time_increment;
        scan_msg->scan_time = scan->scan_time;
        scan_msg->range_min = scan->range_min;
        scan_msg->range_max = scan->range_max;

        for(int i = 0; i < scan->ranges.size(); i += n){
            nthScan.push_back(scan->ranges.at(i));
            RCLCPP_INFO(this->get_logger(), "republished scan index: %d", i);
        }
        scan_msg->angle_increment = scan->angle_increment * n;
        scan_msg->ranges = nthScan;
        scan_pub_->publish(*scan_msg);

        
    }

    int n;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LaserScanNth>());
    rclcpp::shutdown();
    return 0;
}