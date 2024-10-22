/// @file aruco_detector.cpp
/// @brief Node that detects ArUco markers in images and broadcasts their poses as TF transforms.

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/video/tracking.hpp>
#include <vector>
#include <string>
#include <iostream>
#include <yaml-cpp/yaml.h>
#include <Eigen/Dense>
#include <map>

/// @class ArucoDetector
/// @brief Detects ArUco markers in images and publishes their poses using TF.
class ArucoDetector : public rclcpp::Node {
public:
    /// @brief Constructor for ArucoDetector.
    ArucoDetector() : Node("aruco_detector"), tf_broadcaster_(this) {
        // Create a subscription for receiving images
        image_subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
            "camera1/image_raw", 10, std::bind(&ArucoDetector::imageCallback, this, std::placeholders::_1));

        // Read camera calibration parameters
        readCameraCalibration("src/robot_control/config/camera_calibration.yaml", camMatrix_, distCoeffs_);
        readTransforms("src/robot_control/config/transform.yaml");

        // Initialize image dimensions
        image_width_ = 1920;
        image_height_ = 1080;
    }

private:
    /// @brief Reads camera calibration parameters from a file.
    /// @param filename Path to the calibration file.
    /// @param camMatrix Output camera matrix.
    /// @param distCoeffs Output distortion coefficients.
    void readCameraCalibration(const std::string& filename, cv::Mat& camMatrix, cv::Mat& distCoeffs) {
        cv::FileStorage fs(filename, cv::FileStorage::READ);
        if (!fs.isOpened()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open camera calibration file: %s", filename.c_str());
            return;
        }
        fs["camera_matrix"] >> camMatrix;
        fs["distortion_coefficients"] >> distCoeffs;
        fs.release();
    }

    /// @brief Reads transformation matrices from a YAML file.
    /// @param filename Path to the YAML file.
    void readTransforms(const std::string& filename) {
        YAML::Node config = YAML::LoadFile(filename);
        if (config["camera"]) {
            camera_transform_ = parseTransform(config["camera"][0]["transform"]);
        }
    }

    /// @brief Parses a transformation matrix from a YAML node.
    /// @param node YAML node containing the transform.
    /// @return The transformation matrix as an Eigen::Matrix4d.
    Eigen::Matrix4d parseTransform(const YAML::Node& node) {
        Eigen::Matrix4d transform = Eigen::Matrix4d::Identity();
        for (int i = 0; i < node.size(); ++i) {
            for (int j = 0; j < node[i].size(); ++j) {
                transform(i, j) = node[i][j].as<double>();
            }
        }
        return transform;
    }

    /// @brief Initializes the Kalman filter for a given marker ID.
    /// @param marker_id The ID of the ArUco marker.
    void initializeKalmanFilter(int marker_id) {
        // Initialize Kalman filter with state dimension 3 and measurement dimension 3
        cv::KalmanFilter kf(3, 3); 
        kf.transitionMatrix = cv::Mat::eye(3, 3, CV_32F); // Identity transition matrix
        kf.measurementMatrix = cv::Mat::eye(3, 3, CV_32F);
        kf.processNoiseCov = cv::Mat::eye(3, 3, CV_32F) * 1e-4; // Initial process noise covariance
        kf.measurementNoiseCov = cv::Mat::eye(3, 3, CV_32F) * 1e-4; // Initial measurement noise covariance
        kf.errorCovPost = cv::Mat::eye(3, 3, CV_32F);
        kalman_filters_[marker_id] = kf;
    }

    /// @brief Applies Kalman filter to smooth the rotation vector for a given marker.
    /// @param marker_id The ID of the ArUco marker.
    /// @param rvec The rotation vector to be smoothed.
    /// @return The smoothed rotation vector.
    cv::Vec3d applyKalmanFilter(int marker_id, const cv::Vec3d& rvec) {
        if (kalman_filters_.find(marker_id) == kalman_filters_.end()) {
            initializeKalmanFilter(marker_id);
        }

        auto& kf = kalman_filters_[marker_id];

        // Predict step
        cv::Mat prediction = kf.predict();

        // Measurement update
        cv::Mat measurement = (cv::Mat_<float>(3, 1) << rvec[0], rvec[1], rvec[2]);
        cv::Mat estimated = kf.correct(measurement);

        return cv::Vec3d(estimated.at<float>(0), estimated.at<float>(1), estimated.at<float>(2));
    }

    /// @brief Callback function for processing received images.
    /// @param msg The image message received.
    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
        cv::Mat frame;
        try {
            frame = cv_bridge::toCvCopy(msg, "bgr8")->image;
        } catch (cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
            return;
        }

        // Detect ArUco markers
        std::vector<int> markerIds;
        std::vector<std::vector<cv::Point2f>> markerCorners, rejectedCandidates;
        cv::Ptr<cv::aruco::DetectorParameters> detectorParams = cv::aruco::DetectorParameters::create();
        cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
        cv::aruco::detectMarkers(frame, dictionary, markerCorners, markerIds, detectorParams, rejectedCandidates);

        // Refine corner locations to sub-pixel accuracy using cornerSubPix
        if (!markerCorners.empty()) {
            cv::Mat gray;
            cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
            cv::TermCriteria criteria(cv::TermCriteria::EPS + cv::TermCriteria::MAX_ITER, 100, 0.001);
            cv::cornerSubPix(gray, markerCorners[0], cv::Size(5, 5), cv::Size(-1, -1), criteria);
        }

        // Set the marker length in meters
        float markerLength = 0.0425f;

        cv::Mat outputImage = frame.clone();
        if (!markerIds.empty()) {
            size_t nMarkers = markerIds.size();
            std::vector<cv::Vec3d> rvecs, tvecs;

            // Estimate pose of markers
            cv::aruco::estimatePoseSingleMarkers(markerCorners, markerLength, camMatrix_, distCoeffs_, rvecs, tvecs);

            for (size_t i = 0; i < nMarkers; i++) {
                int marker_id = markerIds[i];

                // Apply Kalman filter to smooth the rotation vector
                rvecs[i] = applyKalmanFilter(marker_id, rvecs[i]);

                // Convert rotation vector to rotation matrix
                cv::Mat rotation_matrix;
                cv::Rodrigues(rvecs[i], rotation_matrix);

                // Draw axis for each marker
                cv::aruco::drawAxis(outputImage, camMatrix_, distCoeffs_, rvecs[i], tvecs[i], markerLength * 1.5f);

                // Proceed with the corrected rotation matrix
                Eigen::Matrix4d camera_to_aruco = Eigen::Matrix4d::Identity();
                for (int row = 0; row < 3; ++row) {
                    for (int col = 0; col < 3; ++col) {
                        camera_to_aruco(row, col) = rotation_matrix.at<double>(row, col);
                    }
                    camera_to_aruco(row, 3) = tvecs[i][row];
                }

                // Compute the transformation from the fixed frame to the ArUco marker
                Eigen::Matrix4d fixed_to_aruco = camera_transform_ * camera_to_aruco;

                // Prepare the TransformStamped message
                geometry_msgs::msg::TransformStamped transformStamped;
                transformStamped.header.stamp = this->now();
                transformStamped.header.frame_id = "world";
                transformStamped.child_frame_id = "aruco_" + std::to_string(markerIds[i]);
                transformStamped.transform.translation.x = fixed_to_aruco(0, 3);
                transformStamped.transform.translation.y = fixed_to_aruco(1, 3);
                transformStamped.transform.translation.z = fixed_to_aruco(2, 3);

                // Convert rotation matrix to quaternion
                Eigen::Matrix3d rotation = fixed_to_aruco.block<3, 3>(0, 0);
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

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscription_; ///< Subscription to the image topic.
    cv::Mat camMatrix_, distCoeffs_; ///< Camera calibration parameters.
    tf2_ros::TransformBroadcaster tf_broadcaster_; ///< TF broadcaster for publishing transforms.
    Eigen::Matrix4d camera_transform_; ///< Transformation from camera to fixed frame.
    int image_width_, image_height_; ///< Dimensions of the image.

    /// @brief Kalman filters for each marker.
    std::map<int, cv::KalmanFilter> kalman_filters_;
};

/// @brief Main function that initializes and spins the ArucoDetector node.
/// @param argc Argument count.
/// @param argv Argument vector.
/// @return Exit status code.
int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ArucoDetector>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
