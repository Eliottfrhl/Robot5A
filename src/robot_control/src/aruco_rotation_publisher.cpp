/// @file aruco_rotation_publisher.cpp
/// @brief Node that publishes Euler angles (roll, pitch, yaw) for ArUco markers
/// based on TF transformations.

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/vector3_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include <string>
#include <vector>

/// @class ArucoRotationPublisher
/// @brief Publishes Euler angles of ArUco markers relative to a target frame.
class ArucoRotationPublisher : public rclcpp::Node {
public:
  /// @brief Constructor for ArucoRotationPublisher.
  ArucoRotationPublisher() : Node("aruco_rotation_publisher") {
    // Initialize TF buffer and listener
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // Create publishers for each marker
    for (int i = 0; i <= 15; ++i) {
      std::string marker_frame = "aruco_" + std::to_string(i);
      std::string topic_name = "/aruco_euler_angles/" + marker_frame;
      auto publisher =
          this->create_publisher<geometry_msgs::msg::Vector3Stamped>(topic_name,
                                                                     10);
      publishers_.emplace(i, publisher);
    }

    // Timer to update at regular intervals (e.g., 10 Hz)
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100),
        std::bind(&ArucoRotationPublisher::timer_callback, this));
  }

private:
  std::shared_ptr<tf2_ros::Buffer>
      tf_buffer_; ///< TF2 Buffer for managing transformations.
  std::shared_ptr<tf2_ros::TransformListener>
      tf_listener_; ///< TF2 Listener to listen to TF messages.
  rclcpp::TimerBase::SharedPtr timer_; ///< Timer for periodic updates.

  /// @brief Map of marker ID to their respective publishers.
  std::map<int,
           rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr>
      publishers_;

  /// @brief Callback function called at each timer interval to publish Euler
  /// angles.
  void timer_callback() {
    rclcpp::Time now = this->get_clock()->now();
    std::string target_frame =
        "world"; ///< Adjust if your fixed frame has a different name.

    for (int marker_id = 0; marker_id <= 15; ++marker_id) {
      std::string marker_frame = "aruco_" + std::to_string(marker_id);
      geometry_msgs::msg::TransformStamped transformStamped;

      try {
        // Try to lookup the transform from the marker to the target frame
        transformStamped = tf_buffer_->lookupTransform(
            target_frame, marker_frame, tf2::TimePointZero);

        // Extract the rotation quaternion
        tf2::Quaternion quat;
        tf2::fromMsg(transformStamped.transform.rotation, quat);

        // Convert quaternion to Euler angles (roll, pitch, yaw)
        double roll, pitch, yaw;
        tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw);

        // Create and publish the message
        geometry_msgs::msg::Vector3Stamped euler_msg;
        euler_msg.header.stamp = now;
        euler_msg.header.frame_id = target_frame;
        euler_msg.vector.x = roll;
        euler_msg.vector.y = pitch;
        euler_msg.vector.z = yaw;

        publishers_[marker_id]->publish(euler_msg);

        // Optional: Log the values
        RCLCPP_DEBUG(this->get_logger(), "Marker %d: Roll=%f, Pitch=%f, Yaw=%f",
                     marker_id, roll, pitch, yaw);
      } catch (tf2::TransformException &ex) {
        // Transform not available, skip this marker
        RCLCPP_DEBUG(this->get_logger(),
                     "Could not get transform for marker %d: %s", marker_id,
                     ex.what());
        continue;
      }
    }
  }
};

/// @brief Main function that initializes the node and spins.
/// @param argc Argument count.
/// @param argv Argument vector.
/// @return Exit status code.
int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ArucoRotationPublisher>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
