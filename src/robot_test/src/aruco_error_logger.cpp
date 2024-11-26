#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2/LinearMath/Quaternion.h>
#include <filesystem>
#include <fstream>
#include <string>
#include <chrono>
#include <iomanip>
#include <sstream>
#include <vector>

namespace fs = std::filesystem;

class ArucoErrorLogger : public rclcpp::Node {
public:
  ArucoErrorLogger()
      : Node("aruco_error_logger"), tf_buffer_(this->get_clock()), tf_listener_(tf_buffer_) {

    // Define the base directory as 'src/robot_test/data_analysis/logs'
    fs::path base_directory = fs::current_path() / "src" / "robot_test" / "data_analysis" / "logs";

    // Create the base directory if it doesn't exist
    try {
      if (!fs::exists(base_directory)) {
        fs::create_directories(base_directory);
      }
    } catch (const std::exception &e) {
      RCLCPP_ERROR(this->get_logger(), "Failed to create base directory: %s", e.what());
      throw;
    }

    // Get the current time for the folder name
    auto now_time = std::chrono::system_clock::now();
    auto in_time_t = std::chrono::system_clock::to_time_t(now_time);

    std::stringstream ss;
    ss << "Test_";
    ss << std::put_time(std::localtime(&in_time_t), "%Y-%m-%d_%H-%M-%S");

    std::string folder_name = ss.str();

    // Construct the full path to the test data folder inside the base directory
    fs::path data_folder = base_directory / folder_name;

    // Create the test data directory
    try {
      fs::create_directories(data_folder);
    } catch (const std::exception &e) {
      RCLCPP_ERROR(this->get_logger(), "Failed to create data directory: %s", e.what());
      throw;
    }

    // Open CSV files in the new directory
    fs::path error_file_path = data_folder / "aruco_pose_errors.csv";
    fs::path count_file_path = data_folder / "detected_markers.csv";

    error_file_.open(error_file_path);
    if (!error_file_.is_open()) {
      RCLCPP_ERROR(this->get_logger(), "Failed to open error file: %s", error_file_path.c_str());
      throw std::runtime_error("Failed to open error file");
    }

    count_file_.open(count_file_path);
    if (!count_file_.is_open()) {
      RCLCPP_ERROR(this->get_logger(), "Failed to open count file: %s", count_file_path.c_str());
      throw std::runtime_error("Failed to open count file");
    }

    // Write headers
    error_file_ << "timestamp,marker_id,dx,dy,dz,rx,ry,rz\n";
    count_file_ << "timestamp,detected_marker_ids\n";

    // Create the information file
    fs::path info_file_path = data_folder / "test_information.txt";
    info_file_.open(info_file_path);
    if (!info_file_.is_open()) {
      RCLCPP_ERROR(this->get_logger(), "Failed to open information file: %s", info_file_path.c_str());
      throw std::runtime_error("Failed to open information file");
    }

    // Get the 'test_type' parameter
    this->declare_parameter<std::string>("test_type", "");
    std::string test_type = this->get_parameter("test_type").as_string();

    if (test_type.empty()) {
      RCLCPP_ERROR(this->get_logger(), "Parameter 'test_type' is not set or empty.");
      throw std::runtime_error("Parameter 'test_type' is required.");
    }

    // Get the 'joint_positions' parameter
    this->declare_parameter<std::vector<double>>("joint_positions", std::vector<double>());
    std::vector<double> joint_positions_flat = this->get_parameter("joint_positions").as_double_array();


    // Write information to the file
    info_file_ << "Informations:\n\n";
    info_file_ << "Test: " << test_type << "\n\n";
    info_file_ << "Positions reached:\n";

    // Assuming the robot has 5 joints
    const size_t num_joints = 5;
    if (!joint_positions_flat.empty() && joint_positions_flat.size() % num_joints == 0) {
      for (size_t i = 0; i < joint_positions_flat.size(); i += num_joints) {
        info_file_ << "- [";
        for (size_t j = 0; j < num_joints; ++j) {
          info_file_ << joint_positions_flat[i + j];
          if (j != num_joints - 1) {
            info_file_ << ", ";
          }
        }
        info_file_ << "]\n";
      }
    } else {
      info_file_ << "No positions provided or incorrect format.\n";
    }

    info_file_ << "\nConditions:\n";
    info_file_ << "[Please fill in any test conditions or notes here]\n";

    // Close the information file
    info_file_.close();

    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100),
        std::bind(&ArucoErrorLogger::timer_callback, this));
  }

  ~ArucoErrorLogger() {
    if (error_file_.is_open()) {
      error_file_.close();
    }
    if (count_file_.is_open()) {
      count_file_.close();
    }
  }

private:
  void timer_callback() {
    auto now = this->now();
    std::vector<int> detected_marker_ids;

    // Define maximum age for acceptable transforms (in seconds)
    double max_age_allowed = 0.1; // 100 milliseconds

    for (int marker_id = 0; marker_id <= 15; ++marker_id) {
      std::string estimated_frame = "aruco_" + std::to_string(marker_id);
      std::string ground_truth_frame = "aruco_plane_" + std::to_string(marker_id);

      try {
        // Lookup estimated transform
        geometry_msgs::msg::TransformStamped estimated_transform =
            tf_buffer_.lookupTransform("world", estimated_frame, tf2::TimePointZero);

        // Check the age of the transform
        rclcpp::Time transform_time = estimated_transform.header.stamp;
        rclcpp::Duration age = now - transform_time;

        if (age.seconds() > max_age_allowed) {
          // The transform is too old; skip this marker
          continue;
        }

        // Lookup ground truth transform
        geometry_msgs::msg::TransformStamped ground_truth_transform =
            tf_buffer_.lookupTransform("world", ground_truth_frame, tf2::TimePointZero);

        // Compute translation errors
        double dx = estimated_transform.transform.translation.x -
                    ground_truth_transform.transform.translation.x;
        double dy = estimated_transform.transform.translation.y -
                    ground_truth_transform.transform.translation.y;
        double dz = estimated_transform.transform.translation.z -
                    ground_truth_transform.transform.translation.z;

        // Compute rotation error
        tf2::Quaternion q_est(
            estimated_transform.transform.rotation.x,
            estimated_transform.transform.rotation.y,
            estimated_transform.transform.rotation.z,
            estimated_transform.transform.rotation.w);

        tf2::Quaternion q_gt(
            ground_truth_transform.transform.rotation.x,
            ground_truth_transform.transform.rotation.y,
            ground_truth_transform.transform.rotation.z,
            ground_truth_transform.transform.rotation.w);

        // Compute relative rotation
        tf2::Quaternion q_error = q_gt.inverse() * q_est;

        // Normalize the quaternion
        q_error.normalize();

        // Convert to rotation vector (axis-angle representation)
        double angle = q_error.getAngle(); // Angle in radians
        tf2::Vector3 axis = q_error.getAxis(); // Unit rotation axis

        // Handle potential zero rotation
        if (angle > M_PI) {
          angle -= 2 * M_PI;
        }

        // Rotation vector components
        double rx = angle * axis.x();
        double ry = angle * axis.y();
        double rz = angle * axis.z();

        // Log errors to CSV
        error_file_ << now.seconds() << "," << marker_id << ","
                    << dx << "," << dy << "," << dz << ","
                    << rx << "," << ry << "," << rz << "\n";

        detected_marker_ids.push_back(marker_id);

      } catch (const tf2::TransformException &ex) {
        // Marker not detected; continue to next
        continue;
      }
    }

    // Log detected_marker_ids with timestamp to CSV
    count_file_ << now.seconds() << ",";
    if (!detected_marker_ids.empty()) {
      // Join marker IDs into a single string separated by semicolons
      for (size_t i = 0; i < detected_marker_ids.size(); ++i) {
        count_file_ << detected_marker_ids[i];
        if (i != detected_marker_ids.size() - 1) {
          count_file_ << ";";
        }
      }
    }
    count_file_ << "\n";

    // Flush the streams to ensure data is written to the files
    error_file_.flush();
    count_file_.flush();
  }

  rclcpp::TimerBase::SharedPtr timer_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  std::ofstream error_file_;
  std::ofstream count_file_;
  std::ofstream info_file_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ArucoErrorLogger>());
  rclcpp::shutdown();
  return 0;
}
