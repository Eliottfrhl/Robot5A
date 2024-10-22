/// @file visual_joint_state_publisher.cpp
/// @brief ROS2 node for estimating joint states using ArUco marker detections.

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2/LinearMath/Transform.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "yaml-cpp/yaml.h"

#include <string>
#include <vector>
#include <map>
#include <cmath>
#include <stdexcept>
#include <Eigen/Dense>

/// @class VisualJointStatePublisher
/// @brief Estimates and publishes joint states based on ArUco marker detections.
///
/// This class uses ArUco marker detections to estimate the joint states of a robot.
/// It reads marker configurations from a YAML file and uses TF2 to get marker poses.
class VisualJointStatePublisher : public rclcpp::Node
{
public:
    /// @brief Constructor for VisualJointStatePublisher.
    ///
    /// Initializes the node, loads configuration, sets up TF listener and publisher.
    VisualJointStatePublisher();

private:
    /// @struct MarkerInfo
    /// @brief Stores information about an ArUco marker.
    struct MarkerInfo
    {
        int id;                               ///< Marker ID.
        std::string parent_link;              ///< Name of the parent link.
        tf2::Transform marker_to_link_tf;     ///< Transform from marker to link.
    };

    std::map<int, MarkerInfo> marker_info_map_;                 ///< Map of marker ID to MarkerInfo.
    std::map<std::string, std::vector<int>> link_to_markers_;   ///< Map of link name to marker IDs.
    std::vector<std::string> joint_names_;                      ///< Names of the joints.
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;                ///< TF2 Buffer.
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;   ///< TF2 Listener.
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;  ///< Joint state publisher.
    rclcpp::TimerBase::SharedPtr timer_;                        ///< Timer for periodic updates.
    tf2::Transform camera_transform_;                           ///< Transform for the camera in the base frame.

    /// @brief Loads marker configuration from a YAML file.
    /// @param config_file Path to the YAML configuration file.
    /// @throw std::runtime_error if there's an error in loading or parsing the file.
    void load_config(const std::string& config_file);

    /// @brief Loads camera transform from a YAML file.
    /// @param config_file Path to the YAML configuration file.
    /// @throw std::runtime_error if there's an error in loading or parsing the file.
    void load_camera_transform(const std::string& config_file);

    /// @brief Callback function for the timer.
    ///
    /// Estimates joint states based on current marker poses and publishes them.
    void timer_callback();

    /// @brief Computes the weighted average transform from a vector of transforms and their weights.
    /// @param transforms Vector of transforms to average.
    /// @param weights Vector of weights corresponding to each transform.
    /// @return The weighted average transform.
    tf2::Transform weighted_average_transforms(const std::vector<tf2::Transform>& transforms, const std::vector<double>& weights);

    /// @brief Computes the rotation angle about a given axis.
    /// @param q The rotation as a quaternion.
    /// @param axis The axis of rotation.
    /// @return The rotation angle in radians.
    double get_rotation_angle_about_axis(const tf2::Quaternion& q, const tf2::Vector3& axis);

    /// @brief Computes a weight based on how directly the marker faces the camera.
    /// @param transform The transform of the marker.
    /// @return A weight value (higher for markers directly facing the camera).
    double compute_weight(const tf2::Transform& transform);
};

VisualJointStatePublisher::VisualJointStatePublisher()
: Node("visual_joint_state_publisher")
{
    // Declare and get parameters
    this->declare_parameter<std::string>("config_file", "src/robot_control/config/aruco_to_link.yaml");
    this->declare_parameter<std::string>("camera_config_file", "src/robot_control/config/transform.yaml");
    this->declare_parameter<std::vector<std::string>>("joint_names",
        std::vector<std::string>{"R0_Yaw", "R1_Pitch", "R2_Pitch", "R3_Yaw", "R4_Pitch"});

    std::string config_file = this->get_parameter("config_file").as_string();
    std::string camera_config_file = this->get_parameter("camera_config_file").as_string();
    joint_names_ = this->get_parameter("joint_names").as_string_array();

    // Load marker configurations
    load_config(config_file);

    // Load camera transform
    load_camera_transform(camera_config_file);

    // Initialize TF buffer and listener
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // Publisher for joint states
    joint_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("visual_joint_states", 10);

    // Timer to update at regular intervals (10 Hz)
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100),
        std::bind(&VisualJointStatePublisher::timer_callback, this));
}

void VisualJointStatePublisher::load_config(const std::string& config_file)
{
    try
    {
        YAML::Node config = YAML::LoadFile(config_file);
        const YAML::Node& markers = config["aruco_markers"];
        for (const auto& marker_node : markers)
        {
            MarkerInfo marker_info;
            marker_info.id = marker_node["id"].as<int>();
            marker_info.parent_link = marker_node["parent_link"].as<std::string>();

            std::vector<double> translation = marker_node["translation"].as<std::vector<double>>();
            std::vector<double> rotation = marker_node["rotation"].as<std::vector<double>>();

            if (translation.size() != 3 || rotation.size() != 3)
            {
                throw std::runtime_error("Invalid translation or rotation data for marker " + std::to_string(marker_info.id));
            }

            tf2::Vector3 trans(translation[0], translation[1], translation[2]);
            tf2::Quaternion rot;
            rot.setRPY(rotation[0], rotation[1], rotation[2]);
            rot.normalize();

            marker_info.marker_to_link_tf.setOrigin(trans);
            marker_info.marker_to_link_tf.setRotation(rot);

            marker_info_map_[marker_info.id] = marker_info;
            link_to_markers_[marker_info.parent_link].push_back(marker_info.id);
        }

        RCLCPP_INFO(this->get_logger(), "Loaded %zu ArUco marker configurations.", marker_info_map_.size());
    }
    catch (const YAML::Exception& e)
    {
        RCLCPP_ERROR(this->get_logger(), "YAML parsing error in '%s': %s", config_file.c_str(), e.what());
        throw;
    }
    catch (const std::exception& e)
    {
        RCLCPP_ERROR(this->get_logger(), "Error loading configuration from '%s': %s", config_file.c_str(), e.what());
        throw;
    }
}

void VisualJointStatePublisher::load_camera_transform(const std::string& config_file)
{
    try
    {
        YAML::Node config = YAML::LoadFile(config_file);
        const YAML::Node& camera_node = config["camera"][0];
        const YAML::Node& transform_matrix = camera_node["transform"];

        if (transform_matrix.size() != 4 || transform_matrix[0].size() != 4)
        {
            throw std::runtime_error("Invalid transformation matrix for the camera.");
        }

        tf2::Matrix3x3 rotation(
            transform_matrix[0][0].as<double>(), transform_matrix[0][1].as<double>(), transform_matrix[0][2].as<double>(),
            transform_matrix[1][0].as<double>(), transform_matrix[1][1].as<double>(), transform_matrix[1][2].as<double>(),
            transform_matrix[2][0].as<double>(), transform_matrix[2][1].as<double>(), transform_matrix[2][2].as<double>());
        tf2::Vector3 translation(
            transform_matrix[0][3].as<double>(),
            transform_matrix[1][3].as<double>(),
            transform_matrix[2][3].as<double>());

        camera_transform_.setBasis(rotation);
        camera_transform_.setOrigin(translation);

        RCLCPP_INFO(this->get_logger(), "Loaded camera transform from configuration.");
    }
    catch (const YAML::Exception& e)
    {
        RCLCPP_ERROR(this->get_logger(), "YAML parsing error in '%s': %s", config_file.c_str(), e.what());
        throw;
    }
    catch (const std::exception& e)
    {
        RCLCPP_ERROR(this->get_logger(), "Error loading camera transform from '%s': %s", config_file.c_str(), e.what());
        throw;
    }
}

void VisualJointStatePublisher::timer_callback()
{
    std::map<std::string, std::vector<tf2::Transform>> link_poses;
    std::map<std::string, std::vector<double>> link_weights;
    rclcpp::Time now = this->get_clock()->now();

    // Gather all link poses from markers
    for (const auto& [marker_id, marker_info] : marker_info_map_)
    {
        std::string marker_frame = "aruco_" + std::to_string(marker_id);
        geometry_msgs::msg::TransformStamped world_to_marker_msg;
        
        try
        {
            world_to_marker_msg = tf_buffer_->lookupTransform("world", marker_frame, tf2::TimePointZero);
        }
        catch (tf2::TransformException& ex)
        {
            continue;
        }

        tf2::Transform world_to_marker_tf;
        tf2::fromMsg(world_to_marker_msg.transform, world_to_marker_tf);
        tf2::Transform world_to_link_tf = world_to_marker_tf * marker_info.marker_to_link_tf;

        double weight = compute_weight(world_to_marker_tf);
        link_poses[marker_info.parent_link].push_back(world_to_link_tf);
        link_weights[marker_info.parent_link].push_back(weight);

        // Debugging log
        RCLCPP_DEBUG(this->get_logger(), "Marker %d weight: %f", marker_id, weight);
    }

    // Average poses for each link
    std::map<std::string, tf2::Transform> link_average_poses;
    for (const auto& [link_name, poses] : link_poses)
    {
        if (!poses.empty())
        {
            link_average_poses[link_name] = weighted_average_transforms(poses, link_weights[link_name]);
        }
    }

    // Define required links for joint angle computation
    std::vector<std::string> required_links = {"R5A_link1", "R5A_link2", "R5A_link3", "R5A_link4"};
    std::vector<tf2::Transform> link_transforms;

    // Check and collect transforms for required links
    bool all_required_links_available = true;
    for (const auto& link : required_links)
    {
        auto it = link_average_poses.find(link);
        if (it != link_average_poses.end())
        {
            link_transforms.push_back(it->second);
        }
        else
        {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                "Pose for link '%s' is not available.", link.c_str());
            all_required_links_available = false;
            break; // Exit early if a required link pose is missing
        }
    }

    // Proceed only if all required link poses are available
    if (all_required_links_available && link_transforms.size() == required_links.size())
    {
        tf2::Transform world_to_base_link;
        world_to_base_link.setIdentity(); // Assuming base_link is fixed to world

        // Compute relative transforms between consecutive links
        tf2::Transform base_to_link1 = world_to_base_link.inverse() * link_transforms[0];
        tf2::Transform link1_to_link2 = link_transforms[0].inverse() * link_transforms[1];
        tf2::Transform link2_to_link3 = link_transforms[1].inverse() * link_transforms[2];
        tf2::Transform link3_to_link4 = link_transforms[2].inverse() * link_transforms[3];

        // Compute joint angles constrained to specific axes
        double R0_Yaw_angle = get_rotation_angle_about_axis(base_to_link1.getRotation(), tf2::Vector3(0, 1, 0));
        double R1_Pitch_angle = get_rotation_angle_about_axis(link1_to_link2.getRotation(), tf2::Vector3(1, 0, 0));
        double R2_Pitch_angle = get_rotation_angle_about_axis(link2_to_link3.getRotation(), tf2::Vector3(1, 0, 0));
        double R3_Yaw_angle = get_rotation_angle_about_axis(link3_to_link4.getRotation(), tf2::Vector3(0, -1, 0));

        // Correct R0_Yaw angle by adding Pi
        R0_Yaw_angle += M_PI;
        if (R0_Yaw_angle > M_PI)
        {
            R0_Yaw_angle -= 2 * M_PI; // Normalize the angle to the range [-π, π]
        }

        // Assign joint angles
        std::vector<double> joint_positions(joint_names_.size(), 0.0); // Initialize with zeros
        if (joint_names_.size() >= 4)
        {
            joint_positions[0] = R0_Yaw_angle;
            joint_positions[1] = R1_Pitch_angle;
            joint_positions[2] = R2_Pitch_angle;
            joint_positions[3] = R3_Yaw_angle;
            // R4_Pitch remains 0.0 as there are no markers for it
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Joint names vector is smaller than expected.");
            return;
        }

        // Create and publish JointState message
        sensor_msgs::msg::JointState joint_state_msg;
        joint_state_msg.header.stamp = now;
        joint_state_msg.name = joint_names_;
        joint_state_msg.position = joint_positions;
        // Velocity and effort are left empty as per requirements

        joint_state_pub_->publish(joint_state_msg);

        // Optional: Debugging log
        RCLCPP_DEBUG(this->get_logger(), "Published joint states:");
        for (size_t i = 0; i < joint_names_.size(); ++i)
        {
            RCLCPP_DEBUG(this->get_logger(), "  %s: %f", joint_names_[i].c_str(), joint_positions[i]);
        }
    }
    else
    {
        RCLCPP_WARN(this->get_logger(), "Required link poses are not fully available. Joint angles not updated.");
    }
}

tf2::Transform VisualJointStatePublisher::weighted_average_transforms(const std::vector<tf2::Transform>& transforms, const std::vector<double>& weights)
{
    if (transforms.empty())
    {
        return tf2::Transform::getIdentity();
    }

    // Weighted average positions
    tf2::Vector3 avg_origin(0, 0, 0);
    double total_weight = 0.0;
    for (size_t i = 0; i < transforms.size(); ++i)
    {
        avg_origin += weights[i] * transforms[i].getOrigin();
        total_weight += weights[i];
    }
    avg_origin /= total_weight;

    // Weighted average quaternions with hemisphere consistency
    double qx = 0.0, qy = 0.0, qz = 0.0, qw = 0.0;
    tf2::Quaternion first_q = transforms[0].getRotation();
    for (size_t i = 0; i < transforms.size(); ++i)
    {
        tf2::Quaternion q = transforms[i].getRotation();

        // Ensure all quaternions are on the same hemisphere
        if (q.dot(first_q) < 0.0)
        {
            q = tf2::Quaternion(-q.x(), -q.y(), -q.z(), -q.w());
        }

        qx += weights[i] * q.x();
        qy += weights[i] * q.y();
        qz += weights[i] * q.z();
        qw += weights[i] * q.w();
    }

    double norm = std::sqrt(qx*qx + qy*qy + qz*qz + qw*qw);
    if (norm == 0.0)
    {
        norm = 1.0; // Prevent division by zero
    }
    tf2::Quaternion avg_quat(qx / norm, qy / norm, qz / norm, qw / norm);
    avg_quat.normalize();

    tf2::Transform avg_tf;
    avg_tf.setOrigin(avg_origin);
    avg_tf.setRotation(avg_quat);

    return avg_tf;
}

double VisualJointStatePublisher::get_rotation_angle_about_axis(const tf2::Quaternion& q, const tf2::Vector3& axis)
{
    // Ensure the axis is normalized
    tf2::Vector3 a = axis.normalized();

    // Extract the vector part of the quaternion
    tf2::Vector3 q_vec(q.x(), q.y(), q.z());

    // Compute the projection of q_vec onto the axis
    double projection = q_vec.dot(a);

    // Compute the rotation angle around the axis
    double theta = 2.0 * std::atan2(projection, q.w());

    // Normalize the angle to the range [-pi, pi]
    theta = std::fmod(theta + M_PI, 2.0 * M_PI);
    if (theta < 0)
        theta += 2.0 * M_PI;
    theta -= M_PI;

    return theta;
}

double VisualJointStatePublisher::compute_weight(const tf2::Transform& transform)
{
    // Compute weight based on how directly the marker faces the camera
    tf2::Vector3 camera_z_axis = camera_transform_.getBasis().getColumn(2).normalized();
    tf2::Vector3 marker_direction = transform.getBasis().getColumn(2).normalized(); // Z direction of the marker

    // Calculate the dot product and ensure it's within a valid range
    double dot_product = marker_direction.dot(camera_z_axis);
    double weight = std::max(0.1, dot_product); // Set a minimum weight of 0.1

    // Log marker and camera direction for debugging
    RCLCPP_DEBUG(this->get_logger(), "Marker direction: (%f, %f, %f)", marker_direction.x(), marker_direction.y(), marker_direction.z());
    RCLCPP_DEBUG(this->get_logger(), "Camera Z-axis direction: (%f, %f, %f)", camera_z_axis.x(), camera_z_axis.y(), camera_z_axis.z());
    RCLCPP_DEBUG(this->get_logger(), "Computed weight: %f", weight);

    return weight;
}

/// @brief Main function.
///
/// Initializes the node and starts spinning.
/// @param argc Number of command-line arguments.
/// @param argv Array of command-line arguments.
/// @return 0 on successful execution.
int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<VisualJointStatePublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
