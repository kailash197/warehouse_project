#include "geometry_msgs/msg/pose2_d.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "helper/srv/rotation.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/string.hpp"
#include "tf2/utils.h"
#include <cmath>
#include <string>

using namespace rclcpp;
using namespace std;
using namespace std::chrono_literals;
using namespace std::placeholders;
using namespace helper;

using Twist = geometry_msgs::msg::Twist;
using Odometry = nav_msgs::msg::Odometry;
using Rotation = helper::srv::Rotation;
using Pose2D = geometry_msgs::msg::Pose2D;
using LaserScan = sensor_msgs::msg::LaserScan;

namespace helper {

std::string format_message(const std::string &message) {
  return "[Formatted]: " + message;
}

// Function to find index for a given angle and specific points
int find_index_scan_msg(const LaserScan::SharedPtr scan_msg, double angle) {
  if (!scan_msg) {
    RCLCPP_ERROR(rclcpp::get_logger("find_index_scan_msg"),
                 "Invalid LaserScan message.");
    return -1;
  }

  if (angle < scan_msg->angle_min || angle > scan_msg->angle_max) {
    std::ostringstream error_msg;
    error_msg << "Angle out of bounds, [" << scan_msg->angle_min << " ,"
              << scan_msg->angle_max << "]";
    throw std::invalid_argument(error_msg.str());
  }

  auto angle_to_index = [&](double ang) -> int {
    return static_cast<int>((ang - scan_msg->angle_min) /
                            scan_msg->angle_increment);
  };

  int temp = angle_to_index(angle);
  RCLCPP_DEBUG(rclcpp::get_logger("find_index_scan_msg"),
               "Angle: %f, Index: %d", angle, temp);
  return temp;
}

// Function to normalize angle to [-PI, PI]
double normalize_angle(double angle) {
  while (angle > M_PI)
    angle -= 2.0 * M_PI;
  while (angle < -M_PI)
    angle += 2.0 * M_PI;
  return angle;
}

// Functions to convert degrees to radians and vice versa
double rad2deg(double rad) { return rad * 180.0 / M_PI; }
double deg2rad(double deg) { return deg * M_PI / 180.0; }

class HelperNode : public Node {
private:
  Service<Rotation>::SharedPtr rotation_server_;
  void
  rotation_server_callback(const std::shared_ptr<Rotation::Request> request,
                           const std::shared_ptr<Rotation::Response> response);
  Publisher<Twist>::SharedPtr cmd_vel_pub_;
  Subscription<Odometry>::SharedPtr odom_sub_;
  void odom_callback(const Odometry::SharedPtr odom_msg);
  CallbackGroup::SharedPtr odom_cb_group_;
  Pose2D current_pos_;

public:
  void publish_velocity(double linear, double angular);
  HelperNode();
};

HelperNode::HelperNode() : Node("helper_node") {
  // parameters
  this->declare_parameter<std::string>("scan_topic", "/scan");
  this->declare_parameter<std::string>(
      "cmd_topic", "/diffbot_base_controller/cmd_vel_unstamped");
  this->declare_parameter<std::string>("rotation_service_name",
                                       "/rotation_degrees");
  this->declare_parameter<double>("linear_vel", 0.50);
  this->declare_parameter<std::string>("odom_topic",
                                       "/diffbot_base_controller/odom");
  this->declare_parameter<double>("rotation_offset", 0.10);

  // Fetch parameters
  auto scan_topic = this->get_parameter("scan_topic").as_string();
  auto cmd_topic = this->get_parameter("cmd_topic").as_string();
  auto odom_topic = this->get_parameter("odom_topic").as_string();
  auto rotation_service_name =
      this->get_parameter("rotation_service_name").as_string();

  rotation_server_ = this->create_service<Rotation>(
      rotation_service_name,
      std::bind(&HelperNode::rotation_server_callback, this, _1, _2));

  odom_cb_group_ =
      this->create_callback_group(CallbackGroupType::MutuallyExclusive);
  auto odom_sub_options = SubscriptionOptions();
  odom_sub_options.callback_group = odom_cb_group_;
  odom_sub_ = this->create_subscription<Odometry>(
      odom_topic, QoS(10).reliable(),
      std::bind(&HelperNode::odom_callback, this, _1), odom_sub_options);

  cmd_vel_pub_ =
      this->create_publisher<Twist>(cmd_topic, QoS(10).best_effort());

  RCLCPP_INFO(this->get_logger(), "Helper Node Ready.");
}

void HelperNode::odom_callback(const Odometry::SharedPtr odom_msg) {
  RCLCPP_DEBUG(get_logger(), "Inside ODOM Callback");
  tf2::Quaternion q(
      odom_msg->pose.pose.orientation.x, odom_msg->pose.pose.orientation.y,
      odom_msg->pose.pose.orientation.z, odom_msg->pose.pose.orientation.w);

  current_pos_.x = odom_msg->pose.pose.position.x;
  current_pos_.y = odom_msg->pose.pose.position.y;
  current_pos_.theta = normalize_angle(tf2::getYaw(q));
}

void HelperNode::rotation_server_callback(
    const std::shared_ptr<Rotation::Request> request,
    const std::shared_ptr<Rotation::Response> response) {
  auto rotation_offset = this->get_parameter("rotation_offset").as_double();

  RCLCPP_INFO(this->get_logger(), "Rotation Service Request Received.");
  double rads = helper::deg2rad(request->angle);
  double angle_diff = M_PI;

  do {
    angle_diff = normalize_angle(rads - current_pos_.theta);
    publish_velocity(0, 2 * angle_diff);
    rclcpp::sleep_for(100ms);
  } while (rclcpp::ok() && fabs(angle_diff) > rotation_offset);
  publish_velocity(0.0, 0.0);

  response->complete = true;
  RCLCPP_INFO(this->get_logger(), "Rotation Service Completed.");
}

void HelperNode::publish_velocity(double linear, double angular) {
  auto cmd_msg = Twist();
  cmd_msg.linear.x = linear;
  cmd_msg.angular.z = angular;
  cmd_vel_pub_->publish(cmd_msg);
  rclcpp::sleep_for(std::chrono::milliseconds(20));
}
} // namespace helper
int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<HelperNode>();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}