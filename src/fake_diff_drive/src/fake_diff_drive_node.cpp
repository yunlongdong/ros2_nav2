#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <cmath>

class FakeDiffDrive : public rclcpp::Node {
public:
  FakeDiffDrive() : Node("fake_diff_drive"), x_(0.0), y_(0.0), theta_(0.0),
                    vx_(0.0), vtheta_(0.0) {
    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("/odom", 50);
    cmd_sub_ = create_subscription<geometry_msgs::msg::Twist>(
        "/cmd_vel", 10,
        std::bind(&FakeDiffDrive::cmdCallback, this, std::placeholders::_1));

    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    static_tf_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(*this);

    // Publish static map -> odom transform (identity, no localization drift)
    publishStaticMapToOdom();

    // Publish base_link -> base_scan static TF
    publishStaticBaseToScan();

    double dt = 0.02;  // 50 Hz
    timer_ = create_wall_timer(
        std::chrono::milliseconds(static_cast<int>(dt * 1000)),
        [this, dt]() { this->update(dt); });

    RCLCPP_INFO(get_logger(), "Fake diff drive node started");
  }

private:
  void cmdCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
    vx_ = msg->linear.x;
    vtheta_ = msg->angular.z;
  }

  void update(double dt) {
    // Integrate kinematics
    x_ += vx_ * cos(theta_) * dt;
    y_ += vx_ * sin(theta_) * dt;
    theta_ += vtheta_ * dt;

    auto now = this->get_clock()->now();

    // Publish odom -> base_link TF
    geometry_msgs::msg::TransformStamped tf;
    tf.header.stamp = now;
    tf.header.frame_id = "odom";
    tf.child_frame_id = "base_link";
    tf.transform.translation.x = x_;
    tf.transform.translation.y = y_;
    tf.transform.translation.z = 0.0;
    tf2::Quaternion q;
    q.setRPY(0, 0, theta_);
    tf.transform.rotation.x = q.x();
    tf.transform.rotation.y = q.y();
    tf.transform.rotation.z = q.z();
    tf.transform.rotation.w = q.w();
    tf_broadcaster_->sendTransform(tf);

    // Publish odometry message
    nav_msgs::msg::Odometry odom;
    odom.header.stamp = now;
    odom.header.frame_id = "odom";
    odom.child_frame_id = "base_link";
    odom.pose.pose.position.x = x_;
    odom.pose.pose.position.y = y_;
    odom.pose.pose.orientation.x = q.x();
    odom.pose.pose.orientation.y = q.y();
    odom.pose.pose.orientation.z = q.z();
    odom.pose.pose.orientation.w = q.w();
    odom.twist.twist.linear.x = vx_;
    odom.twist.twist.angular.z = vtheta_;
    odom_pub_->publish(odom);
  }

  void publishStaticMapToOdom() {
    geometry_msgs::msg::TransformStamped tf;
    tf.header.stamp = this->get_clock()->now();
    tf.header.frame_id = "map";
    tf.child_frame_id = "odom";
    tf.transform.rotation.w = 1.0;
    static_tf_broadcaster_->sendTransform(tf);
  }

  void publishStaticBaseToScan() {
    geometry_msgs::msg::TransformStamped tf;
    tf.header.stamp = this->get_clock()->now();
    tf.header.frame_id = "base_link";
    tf.child_frame_id = "base_scan";
    tf.transform.translation.z = 0.1;
    tf.transform.rotation.w = 1.0;
    static_tf_broadcaster_->sendTransform(tf);
  }

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster_;
  rclcpp::TimerBase::SharedPtr timer_;
  double x_, y_, theta_, vx_, vtheta_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FakeDiffDrive>());
  rclcpp::shutdown();
  return 0;
}
