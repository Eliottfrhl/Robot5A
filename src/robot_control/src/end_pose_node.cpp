/// @file end_pose_node.cpp
/// @brief Node that publishes the end-effector pose of a robot arm.

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

/// @class EndPoseNode
/// @brief Publishes the end-effector pose by transforming between frames.
class EndPoseNode : public rclcpp::Node {
public:
  /// @brief Constructor for EndPoseNode.
  EndPoseNode()
      : Node("end_pose_node"), tf_buffer_(this->get_clock()),
        tf_listener_(tf_buffer_) {
    // Create a publisher for the end-effector pose
    publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
        "end_effector_pose", 10);

    // Create a timer to periodically publish the pose
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100),
        std::bind(&EndPoseNode::publishEndEffectorPose, this));
  }

private:
  /// @brief Publishes the end-effector pose at each timer callback.
  void publishEndEffectorPose() {
    geometry_msgs::msg::TransformStamped transformStamped;
    try {
      // Lookup the transform from base_link to R5A_link5
      transformStamped = tf_buffer_.lookupTransform("base_link", "R5A_link5",
                                                    tf2::TimePointZero);
    } catch (tf2::TransformException &ex) {
      RCLCPP_WARN(this->get_logger(),
                  "Could not transform base_link to R5A_link5: %s", ex.what());
      return;
    }

    // Create a PoseStamped message from the transform
    geometry_msgs::msg::PoseStamped pose;
    pose.header.stamp = transformStamped.header.stamp;
    pose.header.frame_id = transformStamped.header.frame_id;
    pose.pose.position.x = transformStamped.transform.translation.x;
    pose.pose.position.y = transformStamped.transform.translation.y;
    pose.pose.position.z = transformStamped.transform.translation.z;
    pose.pose.orientation = transformStamped.transform.rotation;

    // Publish the end-effector pose
    publisher_->publish(pose);
  }

  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr
      publisher_;             ///< Publisher for the end-effector pose.
  tf2_ros::Buffer tf_buffer_; ///< TF2 buffer for storing transforms.
  tf2_ros::TransformListener
      tf_listener_; ///< TF2 listener to receive transforms.
  rclcpp::TimerBase::SharedPtr
      timer_; ///< Timer to periodically publish the pose.
};

/// @brief Main function that initializes and spins the EndPoseNode.
/// @param argc Argument count.
/// @param argv Argument vector.
/// @return Exit status code.
int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<EndPoseNode>());
  rclcpp::shutdown();
  return 0;
}
