/// @file aruco_detector_single.cpp
/// @brief Node that detects ArUco markers in images and broadcasts their poses as TF transforms.

#include "ament_index_cpp/get_package_share_directory.hpp"  // Include ament_index_cpp
#include "cv_bridge/cv_bridge.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include <Eigen/Dense>
#include <opencv2/aruco.hpp>
#include <opencv2/opencv.hpp>
#include <yaml-cpp/yaml.h>

/// @class ArucoDetectorSingle
/// @brief Detects ArUco markers in images and publishes their poses using TF.
class ArucoDetectorSingle : public rclcpp::Node {
public:
  /// @brief Constructor for ArucoDetectorSingle.
  ArucoDetectorSingle() : Node("aruco_detector_single"), tf_broadcaster_(this) {
    // Get the package share directory
    std::string package_share_directory = ament_index_cpp::get_package_share_directory("robot_control");

    // Construct the full paths to the configuration files
    std::string camera_calibration_file = package_share_directory + "/config/camera_calibration.yaml";
    std::string camera_transform_file = package_share_directory + "/config/transform.yaml";

    // Read camera calibration parameters
    readCameraCalibration(camera_calibration_file, camMatrix_, distCoeffs_);
    readTransforms(camera_transform_file);

    // Set the marker length in meters
    marker_length_ = 0.0425;

    // Create a subscription for receiving images
    image_subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
        "camera1/image_raw", 10,
        std::bind(&ArucoDetectorSingle::imageCallback, this, std::placeholders::_1));
  }

private:
  /// @brief Reads camera calibration parameters from a file.
  /// @param filename Path to the calibration file.
  /// @param camMatrix Output camera matrix.
  /// @param distCoeffs Output distortion coefficients.
  void readCameraCalibration(const std::string &filename, cv::Mat &camMatrix,
                             cv::Mat &distCoeffs) {
    cv::FileStorage fs(filename, cv::FileStorage::READ);
    if (!fs.isOpened()) {
      RCLCPP_ERROR(this->get_logger(),
                   "Failed to open camera calibration file: %s",
                   filename.c_str());
      return;
    }
    fs["camera_matrix"] >> camMatrix;
    fs["distortion_coefficients"] >> distCoeffs;
    fs.release();
  }

  /// @brief Reads transformation matrices from a YAML file.
  /// @param filename Path to the YAML file.
  void readTransforms(const std::string &filename) {
    try {
      YAML::Node config = YAML::LoadFile(filename);
      if (config["camera"]) {
        // Find the camera with id 1
        for (const auto &camera_node : config["camera"]) {
          if (camera_node["id"].as<int>() == 1) {
            camera_transform_ = parseTransform(camera_node["transform"]);
            break;
          }
        }
      } else {
        RCLCPP_ERROR(this->get_logger(), "No camera transforms found in the file: %s",
                     filename.c_str());
      }
    } catch (const YAML::Exception &e) {
      RCLCPP_ERROR(this->get_logger(), "YAML parsing error in '%s': %s",
                   filename.c_str(), e.what());
    } catch (const std::exception &e) {
      RCLCPP_ERROR(this->get_logger(),
                   "Error loading camera transforms from '%s': %s",
                   filename.c_str(), e.what());
    }
  }

  /// @brief Parses a transformation matrix from a YAML node.
  /// @param node YAML node containing the transform.
  /// @return The transformation matrix as an Eigen::Matrix4d.
  Eigen::Matrix4d parseTransform(const YAML::Node &node) {
    Eigen::Matrix4d transform = Eigen::Matrix4d::Identity();
    for (int i = 0; i < node.size(); ++i) {
      for (int j = 0; j < node[i].size(); ++j) {
        transform(i, j) = node[i][j].as<double>();
      }
    }
    return transform;
  }

  /// @brief Callback function for processing received images.
  /// @param msg The image message received.
  void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
    cv::Mat frame;
    try {
      frame = cv_bridge::toCvShare(msg, "bgr8")->image;
    } catch (cv_bridge::Exception &e) {
      RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
      return;
    }

    // Detect ArUco markers
    std::vector<int> markerIds;
    std::vector<std::vector<cv::Point2f>> markerCorners, rejectedCandidates;
    cv::Ptr<cv::aruco::DetectorParameters> detectorParams =
        cv::aruco::DetectorParameters::create();
    cv::Ptr<cv::aruco::Dictionary> dictionary =
        cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
    cv::aruco::detectMarkers(frame, dictionary, markerCorners, markerIds,
                             detectorParams, rejectedCandidates);

    // Refine corner locations to sub-pixel accuracy (optional)
    if (!markerCorners.empty()) {
      cv::Mat gray;
      cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
      cv::TermCriteria criteria(
          cv::TermCriteria::EPS + cv::TermCriteria::MAX_ITER, 100, 0.001);
      for (auto &corners : markerCorners) {
        cv::cornerSubPix(gray, corners, cv::Size(5, 5), cv::Size(-1, -1),
                         criteria);
      }
    }

    cv::Mat outputImage = frame.clone();
    if (!markerIds.empty()) {
      size_t nMarkers = markerIds.size();
      std::vector<cv::Vec3d> rvecs, tvecs;

      // Estimate pose of markers
      cv::aruco::estimatePoseSingleMarkers(
          markerCorners, marker_length_, camMatrix_, distCoeffs_, rvecs, tvecs);

      for (size_t i = 0; i < nMarkers; i++) {
        int marker_id = markerIds[i];

        // Convert rotation vector to rotation matrix
        cv::Mat rotation_matrix;
        cv::Rodrigues(rvecs[i], rotation_matrix);

        // Draw axis for each marker
        cv::aruco::drawAxis(outputImage, camMatrix_, distCoeffs_, rvecs[i],
                            tvecs[i], marker_length_ * 1.5f);

        // Build the transformation matrix from camera to marker
        Eigen::Matrix4d camera_to_marker = Eigen::Matrix4d::Identity();
        for (int row = 0; row < 3; ++row) {
          for (int col = 0; col < 3; ++col) {
            camera_to_marker(row, col) = rotation_matrix.at<double>(row, col);
          }
          camera_to_marker(row, 3) = tvecs[i][row];
        }

        // Compute the transformation from the fixed frame to the ArUco marker
        Eigen::Matrix4d fixed_to_marker = camera_transform_ * camera_to_marker;

        // Prepare the TransformStamped message
        geometry_msgs::msg::TransformStamped transformStamped;
        transformStamped.header.stamp = this->now();
        transformStamped.header.frame_id = "world";
        transformStamped.child_frame_id =
            "aruco_" + std::to_string(marker_id);
        transformStamped.transform.translation.x = fixed_to_marker(0, 3);
        transformStamped.transform.translation.y = fixed_to_marker(1, 3);
        transformStamped.transform.translation.z = fixed_to_marker(2, 3);

        // Convert rotation matrix to quaternion
        Eigen::Matrix3d rotation = fixed_to_marker.block<3, 3>(0, 0);
        Eigen::Quaterniond quaternion(rotation);
        transformStamped.transform.rotation.x = quaternion.x();
        transformStamped.transform.rotation.y = quaternion.y();
        transformStamped.transform.rotation.z = quaternion.z();
        transformStamped.transform.rotation.w = quaternion.w();

        // Broadcast the transformation
        tf_broadcaster_.sendTransform(transformStamped);
      }

      // Draw detected markers on the image
      cv::aruco::drawDetectedMarkers(outputImage, markerCorners, markerIds);
    }

    // Display the image for verification (optional)
    cv::imshow("Detected ArUco markers", outputImage);
    cv::waitKey(1);
  }

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr
      image_subscription_;         ///< Subscription to the image topic.
  cv::Mat camMatrix_, distCoeffs_; ///< Camera calibration parameters.
  tf2_ros::TransformBroadcaster
      tf_broadcaster_; ///< TF broadcaster for publishing transforms.
  Eigen::Matrix4d
      camera_transform_; ///< Transformation from camera to fixed frame.
  double marker_length_; ///< Marker length in meters.
};

/// @brief Main function that initializes and spins the ArucoDetectorSingle node.
/// @param argc Argument count.
/// @param argv Argument vector.
/// @return Exit status code.
int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ArucoDetectorSingle>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
