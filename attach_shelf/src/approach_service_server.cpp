#include "attach_shelf/srv/go_to_loading.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/string.hpp"
#include "tf2/utils.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"
#include <algorithm> //std::find_if()

using namespace rclcpp;
using namespace std;
using namespace std::chrono_literals;
using namespace std::placeholders;
using namespace tf2_ros;

using LaserScan = sensor_msgs::msg::LaserScan;
using Twist = geometry_msgs::msg::Twist;
using Odometry = nav_msgs::msg::Odometry;
using GoToLoading = attach_shelf::srv::GoToLoading;
using Groups =
    std::vector<std::tuple<size_t, size_t, std::pair<double, double>>>;
using TransformStamped = geometry_msgs::msg::TransformStamped;
using Pose2D = geometry_msgs::msg::Pose2D;

// const string scan_topic = "/scan";
// const string cmd_topic = "/diffbot_base_controller/cmd_vel_unstamped";
// const string service_name = "/approach_shelf";
// constexpr double linear_vel = 0.50;
// constexpr float intensity_threshold = 7000;
// const string target_frame = "robot_front_laser_base_link";
// const string source_frame = "cart_frame";
// const string reference_frame = "odom";
// const string odom_topic = "/diffbot_base_controller/odom";
// constexpr double distance_forward = 0.65; // meters
// const string elevator_up_topic = "/elevator_up";

// Function to normalize angle to [-PI, PI]
double normalize_angle(double angle) {
  while (angle > M_PI)
    angle -= 2.0 * M_PI;
  while (angle < -M_PI)
    angle += 2.0 * M_PI;
  return angle;
}

class AppoarchServiceServerNode : public Node {
private:
  Service<GoToLoading>::SharedPtr server_;
  void service_callback(const std::shared_ptr<GoToLoading::Request> request,
                        const std::shared_ptr<GoToLoading::Response> response);
  Subscription<LaserScan>::SharedPtr laser_sub_;
  CallbackGroup::SharedPtr laser_cb_group_;
  void laser_scan_callback(const LaserScan::SharedPtr scan_msg);
  Groups find_midpoint_intensity_groups(const LaserScan::SharedPtr scan_msg,
                                        float threshold);
  std::unique_ptr<TransformBroadcaster> cart_tf_broadcaster_;
  TransformStamped cart_transform_;
  Buffer tf_buffer_;
  TransformListener tf_listener_;
  bool found_both_legs_ = false;
  Publisher<Twist>::SharedPtr cmd_vel_pub_;
  Subscription<Odometry>::SharedPtr odom_sub_;
  void odom_callback(const Odometry::SharedPtr odom_msg);
  CallbackGroup::SharedPtr odom_cb_group_;
  Pose2D current_pos_;
  Publisher<std_msgs::msg::String>::SharedPtr elevator_up_pub_;

public:
  void publish_velocity(double linear, double angular);
  AppoarchServiceServerNode()
      : Node("approach_shelf_server_node"), tf_buffer_(this->get_clock()),
        tf_listener_(tf_buffer_) {

    // parameters
    this->declare_parameter<std::string>("scan_topic", "/scan");
    this->declare_parameter<std::string>(
        "cmd_topic", "/diffbot_base_controller/cmd_vel_unstamped");
    this->declare_parameter<std::string>("service_name", "/approach_shelf");
    this->declare_parameter<double>("linear_vel", 0.50);
    this->declare_parameter<float>("intensity_threshold", 7000.0);
    this->declare_parameter<std::string>("target_frame",
                                         "robot_front_laser_base_link");
    this->declare_parameter<std::string>("source_frame", "cart_frame");
    this->declare_parameter<std::string>("reference_frame", "odom");
    this->declare_parameter<std::string>("odom_topic",
                                         "/diffbot_base_controller/odom");
    this->declare_parameter<double>("distance_forward", 0.65);
    this->declare_parameter<std::string>("elevator_up_topic", "/elevator_up");

    // Fetch parameters
    auto scan_topic = this->get_parameter("scan_topic").as_string();
    auto cmd_topic = this->get_parameter("cmd_topic").as_string();
    auto service_name = this->get_parameter("service_name").as_string();

    auto target_frame = this->get_parameter("target_frame").as_string();
    auto source_frame = this->get_parameter("source_frame").as_string();
    auto odom_topic = this->get_parameter("odom_topic").as_string();
    auto elevator_up_topic =
        this->get_parameter("elevator_up_topic").as_string();

    server_ = this->create_service<GoToLoading>(
        service_name,
        std::bind(&AppoarchServiceServerNode::service_callback, this, _1, _2));
    laser_cb_group_ =
        this->create_callback_group(CallbackGroupType::MutuallyExclusive);
    auto laser_sub_options = SubscriptionOptions();
    laser_sub_options.callback_group = laser_cb_group_;
    laser_sub_ = this->create_subscription<LaserScan>(
        scan_topic, QoS(10).best_effort(),
        std::bind(&AppoarchServiceServerNode::laser_scan_callback, this, _1),
        laser_sub_options);
    cart_tf_broadcaster_ = std::make_unique<TransformBroadcaster>(*this);
    cart_transform_.header.frame_id = target_frame;
    cart_transform_.child_frame_id = source_frame;
    cmd_vel_pub_ =
        this->create_publisher<Twist>(cmd_topic, QoS(10).best_effort());
    odom_cb_group_ =
        this->create_callback_group(CallbackGroupType::MutuallyExclusive);
    auto odom_sub_options = SubscriptionOptions();
    odom_sub_options.callback_group = odom_cb_group_;
    odom_sub_ = this->create_subscription<Odometry>(
        odom_topic, QoS(10).reliable(),
        std::bind(&AppoarchServiceServerNode::odom_callback, this, _1),
        odom_sub_options);
    elevator_up_pub_ = create_publisher<std_msgs::msg::String>(
        elevator_up_topic, QoS(10).reliable());
    RCLCPP_INFO(this->get_logger(), "Service Server Ready.");
  }
};

void AppoarchServiceServerNode::odom_callback(
    const Odometry::SharedPtr odom_msg) {
  RCLCPP_DEBUG(get_logger(), "Inside ODOM Callback");
  tf2::Quaternion q(
      odom_msg->pose.pose.orientation.x, odom_msg->pose.pose.orientation.y,
      odom_msg->pose.pose.orientation.z, odom_msg->pose.pose.orientation.w);

  current_pos_.x = odom_msg->pose.pose.position.x;
  current_pos_.y = odom_msg->pose.pose.position.y;
  current_pos_.theta = normalize_angle(tf2::getYaw(q));
}

void AppoarchServiceServerNode::service_callback(
    const std::shared_ptr<GoToLoading::Request> request,
    const std::shared_ptr<GoToLoading::Response> response) {
  /*
  detect the legs of the shelf using the laser intensity values
      - if the laser only detects 1 shelf leg or none: Return False
      - If it detects both legs, the service will publish a transform named
  cart_frame to the center point between both legs.
  */
  auto distance_forward = this->get_parameter("distance_forward").as_double();
  auto linear_vel = this->get_parameter("linear_vel").as_double();
  RCLCPP_INFO(this->get_logger(), "Service Request Received.");
  bool attach_to_shelf = request->attach_to_shelf;
  // i. publish cart_frame transform, in both cases
  if (!found_both_legs_) {
    // False: if the laser only detects 1 shelf leg or none
    response->complete = false;
    RCLCPP_INFO(this->get_logger(), "Service Completed.");
    return;
  }
  if (attach_to_shelf) {
    RCLCPP_INFO(this->get_logger(), "State: Get under the shelf.");
    // perform final approach
    // i. move robot underneathe the shelf
    //  Calculate distance

    /*

    // assign velocities
    double kp_distance = 1.0;
    double kp_yaw = 1.0;
    double linear_vel = 0.0;
    double angular_vel = 0.0;

    if (distance > PROXIMITY_LIMIT) {
      if (distance > 1.0) {
        kp_distance = 1.0;
        kp_yaw = 1.0;
      } else {
        kp_distance = 0.5;
        kp_yaw = 0.5;
      }
      if (fabs(yaw) > M_PI / 6.0) {
        kp_distance *= 0.5;
        kp_yaw *= 1.2;
      }

      linear_vel = kp_distance * distance;
      angular_vel = kp_yaw * yaw;
    }
    */
    TransformStamped transform;
    Twist cmd_vel_;
    double distance, dx, dy, yaw, angular_vel = 0.0;
    while (rclcpp::ok()) {
      try {
        transform = tf_buffer_.lookupTransform("robot_base_link", "cart_frame",
                                               tf2::TimePointZero);
      } catch (tf2::TransformException &ex) {
        RCLCPP_WARN(this->get_logger(), "Could not get transform: %s",
                    ex.what());
        return;
      }
      // Calculate distance
      dx = transform.transform.translation.x;
      dy = transform.transform.translation.y;
      distance = std::sqrt(dx * dx + dy * dy);

      //   // Calculate angle (rotation about Z-axis)
      yaw = atan2(dy, dx);

      if (distance > 0.15) {
        while (rclcpp::ok() && fabs(yaw) > 0.5) {
          try {
            transform = tf_buffer_.lookupTransform(
                "robot_base_link", "cart_frame", tf2::TimePointZero);
          } catch (tf2::TransformException &ex) {
            RCLCPP_WARN(this->get_logger(), "Could not get transform: %s",
                        ex.what());
            return;
          }
          dx = transform.transform.translation.x;
          dy = transform.transform.translation.y;
          yaw = atan2(dy, dx);
          angular_vel = (yaw >= 0.0) ? 0.3 : -0.3;
          publish_velocity(0.0, angular_vel);
          //   RCLCPP_INFO(this->get_logger(), "dx: %.2f, dy: %.2f, thetha:
          //   %.2f",
          //               dx, dy, current_pos_.theta);
        }
        publish_velocity(linear_vel, 0.0);
        RCLCPP_DEBUG(get_logger(), "Distance to cart TF: %.4fm", distance);

      } else {
        break;
      }
    }
    publish_velocity(0.0, 0.0);

    // move 30cm further
    RCLCPP_INFO(this->get_logger(), "State: Move forward.");
    rclcpp::sleep_for(100ms);
    yaw = current_pos_.theta;
    double x_target = current_pos_.x + distance_forward * cos(yaw);
    double y_target = current_pos_.y + distance_forward * sin(yaw);
    dx = x_target;
    dy = y_target;
    distance = std::sqrt(dx * dx + dy * dy);

    while (rclcpp::ok() && distance > 0.01) {
      publish_velocity(0.10, 0.0);
      // update
      yaw = current_pos_.theta;
      dx = x_target - current_pos_.x;
      dy = y_target - current_pos_.y;
      distance = std::sqrt(dx * dx + dy * dy);
      RCLCPP_DEBUG(this->get_logger(), "Distance: %.2f.", distance);
    }
    publish_velocity(0.0, 0.0);

    // ii. lift the shelf
    RCLCPP_INFO(this->get_logger(), "State: Lift the shelf.");
    elevator_up_pub_->publish(std_msgs::msg::String());
  }

  // True: only if the final approach is successful
  response->complete = true;
  RCLCPP_INFO(this->get_logger(), "Service Completed.");
}

void AppoarchServiceServerNode::laser_scan_callback(
    const LaserScan::SharedPtr scan_msg) {
  auto intensity_threshold =
      this->get_parameter("intensity_threshold").as_double();
  auto target_frame = this->get_parameter("target_frame").as_string();
  auto reference_frame = this->get_parameter("reference_frame").as_string();
  RCLCPP_DEBUG(get_logger(), "Inside LASER Callback");
  auto groups = find_midpoint_intensity_groups(scan_msg, intensity_threshold);
  found_both_legs_ = (groups.size() == 2);
  if (found_both_legs_) {
    double cart_x =
        (std::get<2>(groups[0]).first + std::get<2>(groups[1]).first) / 2.0;
    double cart_y =
        (std::get<2>(groups[0]).second + std::get<2>(groups[1]).second) / 2.0;
    RCLCPP_DEBUG(this->get_logger(), "Cart position: (%.2f, %.2f)", cart_x,
                 cart_y);
    /*
    fix Waiting for transform odom ->  cart_frame
    Lookup would require extrapolation into the past
    */
    cart_transform_.header.stamp = this->get_clock()->now();
    TransformStamped transform;
    try {
      transform = tf_buffer_.lookupTransform(reference_frame, target_frame,
                                             tf2::TimePointZero);
      cart_transform_.header.stamp = transform.header.stamp;
    } catch (const tf2::TransformException &ex) {
      RCLCPP_WARN(this->get_logger(), "Could not get transform: %s", ex.what());
    }
    cart_transform_.transform.translation.x = cart_x;
    cart_transform_.transform.translation.y = cart_y;
    cart_transform_.transform.translation.z = 0.0;

    tf2::Quaternion q;
    q.setRPY(0, 0, 0);
    transform.transform.rotation.x = q.x();
    transform.transform.rotation.y = q.y();
    transform.transform.rotation.z = q.z();
    transform.transform.rotation.w = q.w();
    cart_tf_broadcaster_->sendTransform(cart_transform_);
  }
}

Groups AppoarchServiceServerNode::find_midpoint_intensity_groups(
    const LaserScan::SharedPtr scan_msg, float threshold) {
  Groups groups;
  auto it = scan_msg->intensities.begin();
  while (it != scan_msg->intensities.end()) {
    it = std::find_if(
        it, scan_msg->intensities.end(),
        [threshold](float intensity) { return intensity > threshold; });
    if (it != scan_msg->intensities.end()) {
      size_t start_ind = std::distance(scan_msg->intensities.begin(), it);
      it = std::find_if(
          it, scan_msg->intensities.end(),
          [threshold](float intensity) { return intensity <= threshold; });
      size_t end_ind = std::distance(scan_msg->intensities.begin(), it) - 1;

      // compute midpoint
      double sum_x = 0.0, sum_y = 0.0;
      size_t count = 0;
      double angle;
      for (size_t i = start_ind; i <= end_ind; i++) {
        if (scan_msg->ranges[i] > 0.0) {
          angle = scan_msg->angle_min + i * scan_msg->angle_increment;
          sum_x += scan_msg->ranges[i] * std::cos(angle);
          sum_y += scan_msg->ranges[i] * std::sin(angle);
          count++;
        }
      }
      std::pair<double, double> midpoint = {sum_x / count, sum_y / count};
      groups.emplace_back(start_ind, end_ind, midpoint);
    }
  }
  return groups;
}

void AppoarchServiceServerNode::publish_velocity(double linear,
                                                 double angular) {
  auto cmd_msg = Twist();
  cmd_msg.linear.x = linear;
  cmd_msg.angular.z = angular;
  cmd_vel_pub_->publish(cmd_msg);
  rclcpp::sleep_for(std::chrono::milliseconds(20));
}

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<AppoarchServiceServerNode>();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}

/*
Service: /approach_shelf
The service will do the following:
It will detect the legs of the shelf using the laser intensity values (check the
Laser Intensities section below) If the laser only detects 1 shelf leg or none,
it will return a False message. If it detects both legs, the service will
publish a transform named cart_frame to the center point between both legs.
Then, the robot will use this TF to move towards the shelf (using the transform
coordinates). Once the robot has reached the TF coordinates, it will move
forward 30 cm more (to end up right underneath the shelf).
*/
