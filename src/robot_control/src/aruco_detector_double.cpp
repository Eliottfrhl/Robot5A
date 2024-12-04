/// @file aruco_detector_double.cpp
/// @brief ROS2 node for detecting ArUco markers using two cameras and fusing their detections.

#include "ament_index_cpp/get_package_share_directory.hpp"  // Include ament_index_cpp
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <Eigen/Dense>
#include <tf2_eigen/tf2_eigen.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <yaml-cpp/yaml.h>

/// @class ArucoDetectorDouble
/// @brief Detects ArUco markers using two camera feeds and fuses the detections.
class ArucoDetectorDouble : public rclcpp::Node {
public:
  /// @brief Constructor for ArucoDetectorDouble.
  ArucoDetectorDouble();

private:
  /// @brief Callback when images from both cameras are received.
  /// @param img_msg1 Image message from camera 1.
  /// @param img_msg2 Image message from camera 2.
  void imageCallback(
      const sensor_msgs::msg::Image::ConstSharedPtr &img_msg1,
      const sensor_msgs::msg::Image::ConstSharedPtr &img_msg2);

  /// @brief Processes images and detects markers.
  /// @param image1 OpenCV image from camera 1.
  /// @param image2 OpenCV image from camera 2.
  void processImages(const cv::Mat &image1, const cv::Mat &image2);

  /// @brief Fuses marker detections from both cameras and broadcasts transforms.
  void fuseDetections();

  /// @brief Helper function to average quaternions.
  /// @param quaternions Vector of quaternions to average.
  /// @return Averaged quaternion.
  Eigen::Quaterniond averageQuaternions(
      const std::vector<Eigen::Quaterniond> &quaternions);

  // Subscribers for camera images
  message_filters::Subscriber<sensor_msgs::msg::Image> image_sub1_;
  message_filters::Subscriber<sensor_msgs::msg::Image> image_sub2_;

  // Synchronizer for image topics
  typedef message_filters::sync_policies::ApproximateTime<
      sensor_msgs::msg::Image, sensor_msgs::msg::Image>
      SyncPolicy;
  typedef message_filters::Synchronizer<SyncPolicy> Synchronizer;
  std::shared_ptr<Synchronizer> sync_;

  // TF broadcaster
  tf2_ros::TransformBroadcaster tf_broadcaster_;

  // Camera calibration parameters
  cv::Mat camMatrix_, distCoeffs_;

  // ArUco detection parameters
  cv::Ptr<cv::aruco::Dictionary> dictionary_;
  cv::Ptr<cv::aruco::DetectorParameters> detectorParams_;

  // Storage for detections
  std::map<int, std::vector<Eigen::Matrix4d>> marker_poses_; // marker_id -> list of poses

  // Camera to fixed frame transforms
  Eigen::Matrix4d camera_transform1_;
  Eigen::Matrix4d camera_transform2_;

  // Marker size
  double marker_length_;
};

ArucoDetectorDouble::ArucoDetectorDouble()
    : Node("aruco_detector_double"), tf_broadcaster_(this) {

  // Get the package share directory
  std::string package_share_directory = ament_index_cpp::get_package_share_directory("robot_control");

  // Construct the full paths to the configuration files
  std::string camera_calibration_file = package_share_directory + "/config/camera_calibration.yaml";
  std::string camera_transform_file = package_share_directory + "/config/transform.yaml";

  // Set the marker length in meters
  marker_length_ = 0.0425;

  // Load camera calibration parameters
  cv::FileStorage fs(camera_calibration_file, cv::FileStorage::READ);
  if (!fs.isOpened()) {
    RCLCPP_ERROR(this->get_logger(), "Failed to open camera calibration file: %s",
                 camera_calibration_file.c_str());
    throw std::runtime_error("Failed to open camera calibration file");
  }
  fs["camera_matrix"] >> camMatrix_;
  fs["distortion_coefficients"] >> distCoeffs_;
  fs.release();

  // Load camera transforms from YAML file
  try {
    YAML::Node config = YAML::LoadFile(camera_transform_file);

    if (config["camera"]) {
      const YAML::Node &cameras = config["camera"];
      for (const auto &camera_node : cameras) {
        int id = camera_node["id"].as<int>();
        const YAML::Node &transform_matrix = camera_node["transform"];

        if (transform_matrix.size() != 4 || transform_matrix[0].size() != 4) {
          throw std::runtime_error(
              "Invalid transformation matrix for camera id " + std::to_string(id));
        }

        Eigen::Matrix4d camera_transform = Eigen::Matrix4d::Identity();
        for (int i = 0; i < 4; ++i) {
          for (int j = 0; j < 4; ++j) {
            camera_transform(i, j) = transform_matrix[i][j].as<double>();
          }
        }

        if (id == 1) {
          camera_transform1_ = camera_transform;
        } else if (id == 2) {
          camera_transform2_ = camera_transform;
        } else {
          RCLCPP_WARN(this->get_logger(), "Unknown camera id: %d", id);
        }
      }
    } else {
      throw std::runtime_error("No camera transforms found in configuration.");
    }

  } catch (const YAML::Exception &e) {
    RCLCPP_ERROR(this->get_logger(), "YAML parsing error in '%s': %s",
                 camera_transform_file.c_str(), e.what());
    throw;
  } catch (const std::exception &e) {
    RCLCPP_ERROR(this->get_logger(),
                 "Error loading camera transforms from '%s': %s",
                 camera_transform_file.c_str(), e.what());
    throw;
  }

  // Initialize ArUco dictionary and detector parameters
  dictionary_ = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
  detectorParams_ = cv::aruco::DetectorParameters::create();

  // Initialize subscribers and synchronizer
  image_sub1_.subscribe(this, "camera1/image_raw");
  image_sub2_.subscribe(this, "camera2/image_raw");

  sync_.reset(new Synchronizer(SyncPolicy(10), image_sub1_, image_sub2_));
  sync_->registerCallback(std::bind(&ArucoDetectorDouble::imageCallback, this,
                                    std::placeholders::_1,
                                    std::placeholders::_2));
}

void ArucoDetectorDouble::imageCallback(
    const sensor_msgs::msg::Image::ConstSharedPtr &img_msg1,
    const sensor_msgs::msg::Image::ConstSharedPtr &img_msg2) {
  // Convert ROS images to OpenCV images
  cv::Mat image1, image2;

  try {
    image1 = cv_bridge::toCvShare(img_msg1, "bgr8")->image;
    image2 = cv_bridge::toCvShare(img_msg2, "bgr8")->image;
  } catch (cv_bridge::Exception &e) {
    RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
    return;
  }

  // Process images
  processImages(image1, image2);
}

void ArucoDetectorDouble::processImages(const cv::Mat &image1,
                                        const cv::Mat &image2) {
  // Detect markers in image1
  std::vector<int> markerIds1;
  std::vector<std::vector<cv::Point2f>> markerCorners1, rejectedCandidates1;
  cv::aruco::detectMarkers(image1, dictionary_, markerCorners1, markerIds1,
                           detectorParams_, rejectedCandidates1);

  // Estimate pose of markers in image1
  std::vector<cv::Vec3d> rvecs1, tvecs1;
  if (!markerIds1.empty()) {
    cv::aruco::estimatePoseSingleMarkers(markerCorners1, marker_length_,
                                         camMatrix_, distCoeffs_, rvecs1,
                                         tvecs1);
  }

  // Detect markers in image2
  std::vector<int> markerIds2;
  std::vector<std::vector<cv::Point2f>> markerCorners2, rejectedCandidates2;
  cv::aruco::detectMarkers(image2, dictionary_, markerCorners2, markerIds2,
                           detectorParams_, rejectedCandidates2);

  // Estimate pose of markers in image2
  std::vector<cv::Vec3d> rvecs2, tvecs2;
  if (!markerIds2.empty()) {
    cv::aruco::estimatePoseSingleMarkers(markerCorners2, marker_length_,
                                         camMatrix_, distCoeffs_, rvecs2,
                                         tvecs2);
  }

  // Clear previous poses
  marker_poses_.clear();

  // Process detections from camera 1
  for (size_t i = 0; i < markerIds1.size(); ++i) {
    int id = markerIds1[i];
    cv::Vec3d rvec = rvecs1[i];
    cv::Vec3d tvec = tvecs1[i];

    // Convert rotation vector to rotation matrix
    cv::Mat rotation_matrix;
    cv::Rodrigues(rvec, rotation_matrix);

    // Build the transformation matrix from camera to marker
    Eigen::Matrix4d camera_to_marker = Eigen::Matrix4d::Identity();
    for (int row = 0; row < 3; ++row) {
      for (int col = 0; col < 3; ++col) {
        camera_to_marker(row, col) = rotation_matrix.at<double>(row, col);
      }
      camera_to_marker(row, 3) = tvec[row];
    }

    // Transform to fixed frame
    Eigen::Matrix4d fixed_to_marker =
        camera_transform1_ * camera_to_marker;

    // Store the pose
    marker_poses_[id].push_back(fixed_to_marker);
  }

  // Process detections from camera 2
  for (size_t i = 0; i < markerIds2.size(); ++i) {
    int id = markerIds2[i];
    cv::Vec3d rvec = rvecs2[i];
    cv::Vec3d tvec = tvecs2[i];

    // Convert rotation vector to rotation matrix
    cv::Mat rotation_matrix;
    cv::Rodrigues(rvec, rotation_matrix);

    // Build the transformation matrix from camera to marker
    Eigen::Matrix4d camera_to_marker = Eigen::Matrix4d::Identity();
    for (int row = 0; row < 3; ++row) {
      for (int col = 0; col < 3; ++col) {
        camera_to_marker(row, col) = rotation_matrix.at<double>(row, col);
      }
      camera_to_marker(row, 3) = tvec[row];
    }

    // Transform to fixed frame
    Eigen::Matrix4d fixed_to_marker =
        camera_transform2_ * camera_to_marker;

    // Store the pose
    marker_poses_[id].push_back(fixed_to_marker);
  }

  // Fuse detections and broadcast transforms
  fuseDetections();
}

void ArucoDetectorDouble::fuseDetections() {
  for (const auto &marker_pose_pair : marker_poses_) {
    int marker_id = marker_pose_pair.first;
    const std::vector<Eigen::Matrix4d> &poses = marker_pose_pair.second;

    // If only one pose, use it directly
    if (poses.size() == 1) {
      const Eigen::Matrix4d &pose = poses[0];

      // Convert pose to TransformStamped and broadcast
      geometry_msgs::msg::TransformStamped transformStamped;
      transformStamped.header.stamp = this->now();
      transformStamped.header.frame_id = "world";
      transformStamped.child_frame_id = "aruco_" + std::to_string(marker_id);

      transformStamped.transform.translation.x = pose(0, 3);
      transformStamped.transform.translation.y = pose(1, 3);
      transformStamped.transform.translation.z = pose(2, 3);

      Eigen::Matrix3d rotation = pose.block<3, 3>(0, 0);
      Eigen::Quaterniond quaternion(rotation);

      transformStamped.transform.rotation.x = quaternion.x();
      transformStamped.transform.rotation.y = quaternion.y();
      transformStamped.transform.rotation.z = quaternion.z();
      transformStamped.transform.rotation.w = quaternion.w();

      tf_broadcaster_.sendTransform(transformStamped);

    } else if (poses.size() > 1) {
      // Fuse the poses

      // Average positions
      Eigen::Vector3d avg_position(0, 0, 0);
      for (const auto &pose : poses) {
        avg_position += pose.block<3, 1>(0, 3);
      }
      avg_position /= poses.size();

      // Average rotations using quaternion averaging
      std::vector<Eigen::Quaterniond> quaternions;
      for (const auto &pose : poses) {
        Eigen::Matrix3d rotation = pose.block<3, 3>(0, 0);
        Eigen::Quaterniond quaternion(rotation);
        quaternions.push_back(quaternion);
      }

      Eigen::Quaterniond avg_quaternion = averageQuaternions(quaternions);

      // Build the fused transform
      geometry_msgs::msg::TransformStamped transformStamped;
      transformStamped.header.stamp = this->now();
      transformStamped.header.frame_id = "world";
      transformStamped.child_frame_id = "aruco_" + std::to_string(marker_id);

      transformStamped.transform.translation.x = avg_position.x();
      transformStamped.transform.translation.y = avg_position.y();
      transformStamped.transform.translation.z = avg_position.z();

      transformStamped.transform.rotation.x = avg_quaternion.x();
      transformStamped.transform.rotation.y = avg_quaternion.y();
      transformStamped.transform.rotation.z = avg_quaternion.z();
      transformStamped.transform.rotation.w = avg_quaternion.w();

      tf_broadcaster_.sendTransform(transformStamped);
    }
  }
}

Eigen::Quaterniond ArucoDetectorDouble::averageQuaternions(
    const std::vector<Eigen::Quaterniond> &quaternions) {
  // Compute the average quaternion
  Eigen::Vector4d avg_q(0, 0, 0, 0);
  Eigen::Quaterniond first_q = quaternions[0];

  for (const auto &q : quaternions) {
    Eigen::Quaterniond q_adj = q;
    // Ensure quaternions are in the same hemisphere
    if (first_q.dot(q) < 0.0) {
      q_adj.coeffs() *= -1.0;
    }
    avg_q += q_adj.coeffs();
  }
  avg_q /= quaternions.size();
  avg_q.normalize();

  return Eigen::Quaterniond(avg_q);
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ArucoDetectorDouble>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
