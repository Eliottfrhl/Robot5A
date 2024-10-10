#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <vector>
#include <string>
#include <iostream>
#include <yaml-cpp/yaml.h>
#include <Eigen/Dense>

// Define the class ArucoDetector that inherits from rclcpp::Node
class ArucoDetector : public rclcpp::Node {
public:
    ArucoDetector() : Node("aruco_detector"), tf_broadcaster_(this) {
        // Create a subscription for receiving images
        image_subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
            "camera1/image_raw", 10, std::bind(&ArucoDetector::imageCallback, this, std::placeholders::_1));

        // Read camera calibration parameters
        readCameraCalibration("src/robot_control/config/camera_calibration.yaml", camMatrix_, distCoeffs_);

        // Read the camera-to-fixed frame transformation
        readTransforms("src/robot_control/config/transform.yaml");
    }

private:
    // Function to read camera calibration from a YAML file
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

    // Function to read the camera-to-fixed frame transformation
    void readTransforms(const std::string& filename) {
        YAML::Node config = YAML::LoadFile(filename);
        if (config["camera"]) {
            camera_transform_ = parseTransform(config["camera"][0]["transform"]);
        }
    }

    // Function to parse a transformation from a YAML node
    Eigen::Matrix4d parseTransform(const YAML::Node& node) {
        Eigen::Matrix4d transform;
        for (int i = 0; i < 4; ++i) {
            for (int j = 0; j < 4; ++j) {
                transform(i, j) = node[i][j].as<double>();
            }
        }
        return transform;
    }

    // Image callback to process received images
    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
        cv::Mat frame;
        try {
            // Convert ROS message to OpenCV image
            frame = cv_bridge::toCvCopy(msg, "bgr8")->image;
        } catch (cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
            return;
        }

        // Resize image
        cv::Size newSize(640, frame.cols * 640 / frame.rows);
        cv::resize(frame, frame, newSize);

        // Detect ArUco markers
        std::vector<int> markerIds;
        std::vector<std::vector<cv::Point2f>> markerCorners, rejectedCandidates;
        cv::Ptr<cv::aruco::DetectorParameters> detectorParams = cv::aruco::DetectorParameters::create();
        cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
        cv::aruco::detectMarkers(frame, dictionary, markerCorners, markerIds, detectorParams, rejectedCandidates);

        float markerLength = 1.0; // Arbitrary unit for marker size
        cv::Mat objPoints(4, 1, CV_32FC3);
        objPoints.ptr<cv::Vec3f>(0)[0] = cv::Vec3f(-markerLength / 2.f, markerLength / 2.f, 0);
        objPoints.ptr<cv::Vec3f>(0)[1] = cv::Vec3f(markerLength / 2.f, markerLength / 2.f, 0);
        objPoints.ptr<cv::Vec3f>(0)[2] = cv::Vec3f(markerLength / 2.f, -markerLength / 2.f, 0);
        objPoints.ptr<cv::Vec3f>(0)[3] = cv::Vec3f(-markerLength / 2.f, -markerLength / 2.f, 0);

        size_t nMarkers = markerCorners.size();
        std::vector<cv::Vec3d> rvecs(nMarkers), tvecs(nMarkers);

        cv::Mat outputImage = frame.clone();
        if (!markerIds.empty()) {
            for (size_t i = 0; i < nMarkers; i++) {
                // Solve for pose of each marker
                cv::solvePnP(objPoints, markerCorners[i], camMatrix_, distCoeffs_, rvecs[i], tvecs[i]);
                cv::drawFrameAxes(outputImage, camMatrix_, distCoeffs_, rvecs[i], tvecs[i], markerLength * 1.5f, 2);

                // Convert rotation vector to rotation matrix
                cv::Mat rotation_matrix;
                cv::Rodrigues(rvecs[i], rotation_matrix);

                // Extract the column vectors of the rotation matrix
                cv::Mat col1 = rotation_matrix.col(0);
                cv::Mat col2 = rotation_matrix.col(1);
                cv::Mat col3 = rotation_matrix.col(2);

                cv::Vec3d v1(col1.at<double>(0, 0), col1.at<double>(1, 0), col1.at<double>(2, 0));
                cv::Vec3d v2(col2.at<double>(0, 0), col2.at<double>(1, 0), col2.at<double>(2, 0));
                cv::Vec3d v3(col3.at<double>(0, 0), col3.at<double>(1, 0), col3.at<double>(2, 0));

                // Compute the scalar triple product
                cv::Vec3d cross_product = v1.cross(v2);
                double scalar_triple_product = cross_product.dot(v3);

                // Determine the handedness and correct if necessary
                if (scalar_triple_product < 0) {
                    RCLCPP_WARN(this->get_logger(), "Negative scalar triple product detected for marker ID: %d", markerIds[i]);
                    // The coordinate system is left-handed; correct it
                    rotation_matrix.col(2) = -rotation_matrix.col(2);

                    // Recompute the rotation vector from the corrected rotation matrix
                    cv::Rodrigues(rotation_matrix, rvecs[i]);
                }

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
                transformStamped.header.frame_id = "world"; // Assuming the fixed frame is named "fixed_frame"
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

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscription_; // Subscription for image data
    cv::Mat camMatrix_, distCoeffs_; // Camera calibration matrices
    tf2_ros::TransformBroadcaster tf_broadcaster_; // TF broadcaster for transformations
    Eigen::Matrix4d camera_transform_; // Transformation from camera to fixed frame
};

// Main function
int main(int argc, char **argv) {
    rclcpp::init(argc, argv); // Initialize ROS2
    auto node = std::make_shared<ArucoDetector>(); // Create node
    rclcpp::spin(node); // Run node
    rclcpp::shutdown(); // Shutdown ROS2
    return 0;
}
