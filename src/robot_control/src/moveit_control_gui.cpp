#include <QApplication>
#include <QComboBox>
#include <QLabel>
#include <QPushButton>
#include <QVBoxLayout>
#include <QWidget>
#include <geometry_msgs/msg/pose_stamped.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <rclcpp/rclcpp.hpp>
#include <sstream>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/transform_listener.h>

class MoveItControlGui : public QWidget {
public:
  MoveItControlGui(rclcpp::Node::SharedPtr node)
      : node_(node), move_group_interface_(node_, "arm"),
        tf_buffer_(node_->get_clock(),
                   tf2::Duration(0)), // Pass clock and zero duration
        tf_listener_(tf_buffer_) {
    // GUI Setup
    QVBoxLayout *layout = new QVBoxLayout(this);

    QPushButton *move_to_home_btn =
        new QPushButton("Move to Home Position", this);
    QPushButton *move_to_predefined_btn =
        new QPushButton("Move to Predefined Position", this);
    QPushButton *move_to_tf_btn = new QPushButton("Move to TF Position", this);
    QPushButton *refresh_frames_btn =
        new QPushButton("Refresh TF Frames", this);

    // Dropdown to select TF frames
    tf_frame_selector_ = new QComboBox(this);
    layout->addWidget(tf_frame_selector_);

    // Add status label to show the result of the planning
    status_label_ = new QLabel("Status: Waiting for action", this);
    layout->addWidget(status_label_);

    layout->addWidget(move_to_home_btn);
    layout->addWidget(move_to_predefined_btn);
    layout->addWidget(move_to_tf_btn);
    layout->addWidget(refresh_frames_btn);

    // Connect buttons to a single function, passing different parameters
    connect(move_to_home_btn, &QPushButton::clicked, this, [this]() {
      moveToPosition(0.006755, 0.000006, 0.777373, 0.0, 0.0, 1.0,
                     0.0); // Home position
    });

    connect(move_to_predefined_btn, &QPushButton::clicked, this, [this]() {
      moveToPosition(-0.018829, -0.252053, 0.578420, -0.303061, -0.566473,
                     0.624680, 0.443889); // Predefined position
    });

    connect(move_to_tf_btn, &QPushButton::clicked, this,
            &MoveItControlGui::moveToTf);
    connect(refresh_frames_btn, &QPushButton::clicked, this,
            &MoveItControlGui::refreshTfFrames);

    setLayout(layout);

    // Refresh frames on startup
    refreshTfFrames();
  }

private:
  rclcpp::Node::SharedPtr node_;
  moveit::planning_interface::MoveGroupInterface move_group_interface_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  QComboBox *tf_frame_selector_;
  QLabel *status_label_; // Add a QLabel to show the status of the movement

  // Generic function to move to a position
  void moveToPosition(double x, double y, double z, double qx, double qy,
                      double qz, double qw) {
    RCLCPP_INFO(node_->get_logger(),
                "Moving to position: x=%.2f, y=%.2f, z=%.2f", x, y, z);

    geometry_msgs::msg::Pose target_pose;
    target_pose.position.x = x;
    target_pose.position.y = y;
    target_pose.position.z = z;
    target_pose.orientation.x = qx;
    target_pose.orientation.y = qy;
    target_pose.orientation.z = qz;
    target_pose.orientation.w = qw;

    move_group_interface_.setPoseTarget(target_pose, "R5A_link5");

    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    bool success = (move_group_interface_.plan(my_plan) ==
                    moveit::core::MoveItErrorCode::SUCCESS);
    if (success) {
      RCLCPP_INFO(node_->get_logger(), "Plan successful! Executing...");
      status_label_->setText("Status: Plan successful! Executing...");
      move_group_interface_.execute(my_plan);
    } else {
      RCLCPP_ERROR(node_->get_logger(), "Planning failed!");
      status_label_->setText("Status: Planning failed!");
    }
  }

  // Function to move to a position broadcasted by a TF frame
  void moveToTf() {
    QString selected_frame = tf_frame_selector_->currentText();
    if (selected_frame.isEmpty()) {
      RCLCPP_ERROR(node_->get_logger(), "No TF frame selected.");
      status_label_->setText("Status: No TF frame selected.");
      return;
    }

    RCLCPP_INFO(node_->get_logger(),
                "Moving to position from TF broadcaster: %s",
                selected_frame.toStdString().c_str());

    try {
      geometry_msgs::msg::TransformStamped transformStamped =
          tf_buffer_.lookupTransform(
              "base_link",                  // Target frame
              selected_frame.toStdString(), // Source frame from dropdown
              tf2::TimePointZero);

      moveToPosition(transformStamped.transform.translation.x,
                     transformStamped.transform.translation.y,
                     transformStamped.transform.translation.z,
                     transformStamped.transform.rotation.x,
                     transformStamped.transform.rotation.y,
                     transformStamped.transform.rotation.z,
                     transformStamped.transform.rotation.w);
    } catch (tf2::TransformException &ex) {
      RCLCPP_ERROR(node_->get_logger(), "TF Exception: %s", ex.what());
      status_label_->setText("Status: TF Exception occurred.");
    }
  }

  // Function to refresh the list of available TF frames
  void refreshTfFrames() {
    RCLCPP_INFO(node_->get_logger(), "Refreshing list of TF frames...");

    // Get all frames from the TF buffer
    std::vector<std::string> frame_list;
    tf_buffer_._getFrameStrings(frame_list); // Fetch frames into vector
    tf_frame_selector_->clear();

    for (const auto &frame : frame_list) {
      tf_frame_selector_->addItem(QString::fromStdString(frame));
    }

    if (frame_list.empty()) {
      RCLCPP_WARN(node_->get_logger(), "No TF frames found.");
      status_label_->setText("Status: No TF frames found.");
    } else {
      RCLCPP_INFO(node_->get_logger(), "TF frames updated.");
      status_label_->setText("Status: TF frames updated.");
    }
  }
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);

  auto node = std::make_shared<rclcpp::Node>(
      "moveit_control_gui",
      rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(
          true));

  QApplication app(argc, argv);
  MoveItControlGui gui(node);
  gui.show();

  return app.exec();
}
