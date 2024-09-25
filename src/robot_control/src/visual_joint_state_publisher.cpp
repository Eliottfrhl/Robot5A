// File: visual_joint_state_publisher.cpp

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2/LinearMath/Transform.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "yaml-cpp/yaml.h"

#include <string>
#include <vector>
#include <map>

class VisualJointStatePublisher : public rclcpp::Node
{
public:
    VisualJointStatePublisher()
    : Node("visual_joint_state_publisher")
    {
        // Load configuration file
        std::string config_file = this->declare_parameter<std::string>("config_file", "src/robot_control/config/aruco_to_link.yaml");
        load_config(config_file);

        // Initialize TF buffer and listener
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        // Publisher
        joint_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("visual_joint_states", 10);

        // Timer to update at regular intervals
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&VisualJointStatePublisher::timer_callback, this));
    }

private:
    struct MarkerInfo
    {
        int id;
        std::string parent_link;
        tf2::Transform marker_to_link_tf;
    };

    void load_config(const std::string& config_file)
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

                tf2::Vector3 trans(translation[0], translation[1], translation[2]);

                tf2::Quaternion rot;
                rot.setRPY(rotation[0], rotation[1], rotation[2]);

                marker_info.marker_to_link_tf.setOrigin(trans);
                marker_info.marker_to_link_tf.setRotation(rot);

                marker_info_map_[marker_info.id] = marker_info;

                // Add to link to markers map
                link_to_markers_[marker_info.parent_link].push_back(marker_info.id);
            }
        }
        catch (const std::exception& e)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to load configuration file: %s", e.what());
        }
    }

    void timer_callback()
    {
        // For each link, collect poses from detected markers
        std::map<std::string, std::vector<tf2::Transform>> link_poses;

        rclcpp::Time now = this->get_clock()->now();

        for (const auto& marker_pair : marker_info_map_)
        {
            int marker_id = marker_pair.first;
            const MarkerInfo& marker_info = marker_pair.second;

            std::string marker_frame = "aruco_" + std::to_string(marker_id);

            // Try to get the transform from world to marker
            geometry_msgs::msg::TransformStamped world_to_marker_msg;
            try
            {
                world_to_marker_msg = tf_buffer_->lookupTransform("world", marker_frame, tf2::TimePointZero);
            }
            catch (tf2::TransformException& ex)
            {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                    "Could not transform from world to %s: %s", marker_frame.c_str(), ex.what());
                continue;
            }

            // Convert to tf2 Transform
            tf2::Transform world_to_marker_tf;
            tf2::fromMsg(world_to_marker_msg.transform, world_to_marker_tf);

            // Compute world to link transform
            tf2::Transform marker_to_link_tf = marker_info.marker_to_link_tf;
            tf2::Transform world_to_link_tf = world_to_marker_tf * marker_to_link_tf;

            // Collect the link pose
            link_poses[marker_info.parent_link].push_back(world_to_link_tf);
        }

        // For each link, average the poses if multiple markers
        std::map<std::string, tf2::Transform> link_average_poses;
        for (const auto& link_pose_pair : link_poses)
        {
            const std::string& link_name = link_pose_pair.first;
            const std::vector<tf2::Transform>& poses = link_pose_pair.second;

            if (poses.empty())
            {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                    "No poses for link %s", link_name.c_str());
                continue;
            }

            tf2::Transform average_pose = average_transforms(poses);
            link_average_poses[link_name] = average_pose;
        }

        // Compute joint angles from link poses
        // Since base_link is fixed to world, we can assume base_link pose is identity
        tf2::Transform world_to_base_link;
        world_to_base_link.setIdentity();

        // Initialize joint names and positions
        std::vector<std::string> joint_names = {"R0_Yaw", "R1_Pitch", "R2_Pitch", "R3_Yaw"};
        std::vector<double> joint_positions;

        // Check if all required links are available
        if (link_average_poses.size() < 4)
        {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                "Not all link poses are available to compute joint angles.");
            return;
        }

        // Retrieve the poses of the links
        tf2::Transform world_to_link1 = link_average_poses["R5A_link1"];
        tf2::Transform world_to_link2 = link_average_poses["R5A_link2"];
        tf2::Transform world_to_link3 = link_average_poses["R5A_link3"];
        tf2::Transform world_to_link4 = link_average_poses["R5A_link4"];

        // Now compute the joint angles
        double R0_Yaw_angle = compute_R0_Yaw(world_to_base_link, world_to_link1);
        double R1_Pitch_angle = compute_R1_Pitch(world_to_link1, world_to_link2);
        double R2_Pitch_angle = compute_R2_Pitch(world_to_link2, world_to_link3);
        double R3_Yaw_angle = compute_R3_Yaw(world_to_link3, world_to_link4);

        joint_positions.push_back(R0_Yaw_angle);
        joint_positions.push_back(R1_Pitch_angle);
        joint_positions.push_back(R2_Pitch_angle);
        joint_positions.push_back(R3_Yaw_angle);

        // Create JointState message
        sensor_msgs::msg::JointState joint_state_msg;
        joint_state_msg.header.stamp = now;
        joint_state_msg.name = joint_names;
        joint_state_msg.position = joint_positions;
        // Leave velocity and effort empty

        // Publish the message
        joint_state_pub_->publish(joint_state_msg);
    }

    tf2::Transform average_transforms(const std::vector<tf2::Transform>& transforms)
    {
        // Simple averaging of positions and orientations
        tf2::Vector3 avg_origin(0, 0, 0);
        tf2::Quaternion avg_quat(0, 0, 0, 0);
        double total_weight = 0.0;

        for (size_t i = 0; i < transforms.size(); ++i)
        {
            const auto& tf = transforms[i];
            avg_origin += tf.getOrigin();

            tf2::Quaternion q = tf.getRotation();
            if (i == 0)
            {
                avg_quat = q;
            }
            else
            {
                avg_quat = tf2::slerp(avg_quat, q, 1.0 / (i + 1));
            }
            total_weight += 1.0;
        }

        avg_origin *= (1.0 / total_weight);
        avg_quat.normalize();

        tf2::Transform avg_tf;
        avg_tf.setOrigin(avg_origin);
        avg_tf.setRotation(avg_quat);

        return avg_tf;
    }

    double compute_R0_Yaw(const tf2::Transform& world_to_base_link, const tf2::Transform& world_to_link1)
    {
        // Compute relative transform
        tf2::Transform base_link_to_link1 = world_to_base_link.inverse() * world_to_link1;

        // The joint axis is (0, 1, 0) in base_link frame
        // Extract rotation around Y axis
        tf2::Matrix3x3 rotation_matrix(base_link_to_link1.getRotation());
        double roll, pitch, yaw;
        rotation_matrix.getRPY(roll, pitch, yaw);

        // Return the rotation around Y axis
        return pitch;
    }

    double compute_R1_Pitch(const tf2::Transform& world_to_link1, const tf2::Transform& world_to_link2)
    {
        // Compute relative transform
        tf2::Transform link1_to_link2 = world_to_link1.inverse() * world_to_link2;

        // The joint axis is (-1, 0, 0) in link1 frame
        // Extract rotation around X axis and invert sign
        tf2::Matrix3x3 rotation_matrix(link1_to_link2.getRotation());
        double roll, pitch, yaw;
        rotation_matrix.getRPY(roll, pitch, yaw);

        // Return negative rotation around X axis
        return -roll;
    }

    double compute_R2_Pitch(const tf2::Transform& world_to_link2, const tf2::Transform& world_to_link3)
    {
        // Compute relative transform
        tf2::Transform link2_to_link3 = world_to_link2.inverse() * world_to_link
