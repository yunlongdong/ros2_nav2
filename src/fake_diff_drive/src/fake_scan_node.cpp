#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <cmath>

class FakeScan : public rclcpp::Node {
public:
  FakeScan() : Node("fake_scan") {
    pub_ = create_publisher<sensor_msgs::msg::LaserScan>("/scan", 10);
    timer_ = create_wall_timer(std::chrono::milliseconds(100),
                               std::bind(&FakeScan::publish, this));
    RCLCPP_INFO(get_logger(), "Fake scan node started");
  }

private:
  void publish() {
    sensor_msgs::msg::LaserScan scan;
    scan.header.stamp = this->get_clock()->now();
    scan.header.frame_id = "base_scan";
    scan.angle_min = -M_PI;
    scan.angle_max = M_PI;
    scan.angle_increment = M_PI / 180.0;  // 1 degree
    scan.time_increment = 0.0;
    scan.range_min = 0.1;
    scan.range_max = 30.0;

    int num_readings = static_cast<int>(
        (scan.angle_max - scan.angle_min) / scan.angle_increment);
    scan.ranges.resize(num_readings, scan.range_max);  // all clear, no obstacles
    scan.intensities.resize(num_readings, 0.0);

    pub_->publish(scan);
  }

  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FakeScan>());
  rclcpp::shutdown();
  return 0;
}
