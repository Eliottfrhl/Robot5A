/// @file cam_calibrator.cpp
/// @brief Node that calibrates the camera using chessboard images.

#include "cv_bridge/cv_bridge.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include <fstream>
#include <iostream>
#include <opencv2/aruco.hpp>
#include <opencv2/opencv.hpp>
#include <string>
#include <vector>

/// @class CameraCalibrator
/// @brief A ROS2 node that calibrates the camera using images of a chessboard
/// pattern.
class CameraCalibrator : public rclcpp::Node {
public:
  /// @brief Constructor for the CameraCalibrator class.
  CameraCalibrator()
      : Node("camera_calibrator"), detected_chessboard_count_(0) {
    // Subscribe to the image topic
    image_subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
        "camera1/image_raw", 10,
        std::bind(&CameraCalibrator::imageCallback, this,
                  std::placeholders::_1));

    // Initialize object points for the chessboard
    objp_.resize(num_corners_x * num_corners_y);
    for (int i = 0; i < num_corners_y; i++) {
      for (int j = 0; j < num_corners_x; j++) {
        objp_[i * num_corners_x + j] = cv::Point3f(j, i, 0);
      }
    }

    // Debug: Inform that the constructor has completed
    RCLCPP_INFO(this->get_logger(), "CameraCalibrator node initialized.");
  }

private:
  /// @brief Callback function for receiving images.
  /// @param msg The image message received.
  void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
    RCLCPP_INFO(this->get_logger(), "Image received.");

    cv::Mat frame;
    try {
      // Convert the ROS message to an OpenCV image
      frame = cv_bridge::toCvCopy(msg, "bgr8")->image;
      if (frame.empty()) {
        RCLCPP_ERROR(
            this->get_logger(),
            "Failed to convert ROS image to OpenCV format: Empty frame.");
        return;
      }
    } catch (cv_bridge::Exception &e) {
      RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
      return;
    } catch (std::exception &e) {
      RCLCPP_ERROR(this->get_logger(), "Unknown error in image conversion: %s",
                   e.what());
      return;
    }

    // Resize the image for faster processing (optional)
    // cv::resize(frame, frame, cv::Size(960, 540));

    // Detect chessboard corners
    std::vector<cv::Point2f> corners;
    bool found = cv::findChessboardCorners(
        frame, cv::Size(num_corners_x, num_corners_y), corners);

    if (found) {
      RCLCPP_INFO(this->get_logger(), "Chessboard corners found.");
      detected_chessboard_count_++;
      obj_points_.push_back(objp_);
      img_points_.push_back(corners);

      // Draw and display the detected chessboard corners
      cv::drawChessboardCorners(frame, cv::Size(num_corners_x, num_corners_y),
                                corners, found);
      cv::imshow("Chessboard Corners", frame);
      cv::waitKey(1000); // Wait for 1 second to visualize the detected corners
    } else {
      RCLCPP_WARN(this->get_logger(), "Chessboard corners not found.");
    }

    // If enough chessboards have been detected, launch calibration
    if (detected_chessboard_count_ > 50) {
      RCLCPP_INFO(this->get_logger(),
                  "Detected %d chessboards. Starting calibration.",
                  detected_chessboard_count_);
      calibrateCamera();
    } else {
      RCLCPP_INFO(this->get_logger(),
                  "Chessboard count: %d. Waiting for more detections.",
                  detected_chessboard_count_);
    }
  }

  /// @brief Function to calibrate the camera using the collected image points
  /// and object points.
  void calibrateCamera() {
    if (obj_points_.empty() || img_points_.empty()) {
      RCLCPP_ERROR(
          this->get_logger(),
          "Insufficient data for calibration. No chessboards were detected.");
      return;
    }

    cv::Mat camMatrix, distCoeffs;
    std::vector<cv::Mat> rvecs, tvecs;

    try {
      // Camera calibration
      double rms = cv::calibrateCamera(obj_points_, img_points_,
                                       cv::Size(img_width_, img_height_),
                                       camMatrix, distCoeffs, rvecs, tvecs);

      // Debug: Inform calibration details
      RCLCPP_INFO(this->get_logger(),
                  "Calibration successful with RMS error: %.6f", rms);

      // Display the camera matrix and distortion coefficients
      std::cout << "Camera Matrix:" << std::endl << camMatrix << std::endl;
      std::cout << "Distortion Coefficients:" << std::endl
                << distCoeffs << std::endl;

      // Save calibration results to a YAML file
      cv::FileStorage fs("src/robot_control/config/camera_calibration.yaml",
                         cv::FileStorage::WRITE);
      if (!fs.isOpened()) {
        RCLCPP_ERROR(this->get_logger(),
                     "Failed to open the file for writing camera calibration.");
        return;
      }
      fs << "camera_matrix" << camMatrix;
      fs << "distortion_coefficients" << distCoeffs;
      fs.release();

      RCLCPP_INFO(
          this->get_logger(),
          "Camera calibration completed and saved to camera_calibration.yaml.");
    } catch (const cv::Exception &e) {
      RCLCPP_ERROR(this->get_logger(),
                   "OpenCV error during camera calibration: %s", e.what());
    } catch (const std::exception &e) {
      RCLCPP_ERROR(this->get_logger(),
                   "Unknown error during camera calibration: %s", e.what());
    }

    rclcpp::shutdown(); ///< Shutdown the node after calibration
  }

  // Declaration of member variables
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr
      image_subscription_; ///< Subscription to the image topic.
  std::vector<std::vector<cv::Point3f>>
      obj_points_; ///< 3D points in real-world space for all images.
  std::vector<std::vector<cv::Point2f>>
      img_points_; ///< 2D points in image plane for all images.
  std::vector<cv::Point3f>
      objp_; ///< 3D points for a single chessboard pattern.
  int detected_chessboard_count_; ///< Counter for the number of detected
                                  ///< chessboards.
  const int num_corners_x =
      7; ///< Number of inner corners along the X-axis in the chessboard.
  const int num_corners_y =
      7; ///< Number of inner corners along the Y-axis in the chessboard.
  int img_width_ = 800;  ///< Width of the images used for calibration.
  int img_height_ = 800; ///< Height of the images used for calibration.
};

/// @brief Main function that initializes the CameraCalibrator node and starts
/// spinning.
/// @param argc Argument count.
/// @param argv Argument vector.
/// @return Exit status code.
int main(int argc, char **argv) {
  rclcpp::init(argc, argv); ///< ROS2 initialization

  try {
    auto node =
        std::make_shared<CameraCalibrator>(); ///< Create CameraCalibrator node
    RCLCPP_INFO(node->get_logger(), "Node started, waiting for images...");
    rclcpp::spin(node); ///< Execute the node
  } catch (const std::exception &e) {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),
                 "Error during node execution: %s", e.what());
  }

  rclcpp::shutdown(); ///< Shutdown ROS2
  return 0;
}
