/// @file moveit_control_gui.cpp
/// @brief A ROS2 node providing a GUI to control a robotic arm and gripper using MoveIt.

#include <QApplication>
#include <QComboBox>
#include <QLabel>
#include <QPushButton>
#include <QVBoxLayout>
#include <QWidget>
#include <cmath> // For M_PI
#include <geometry_msgs/msg/pose_stamped.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <rclcpp/executors/single_threaded_executor.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/transform_listener.h>
#include <thread>
#include <map>
#include <vector>

/**
 * @brief A GUI class to control a robotic arm and gripper using MoveIt.
 */
class MoveItControlGui : public QWidget {
public:
    MoveItControlGui(rclcpp::Node::SharedPtr node)
        : node_(node),
          move_group_interface_(node_, "arm"),
          gripper_move_group_(node_, "gripper"),
          tf_buffer_(node_->get_clock()),
          tf_listener_(tf_buffer_),
          executor_(std::make_shared<rclcpp::executors::SingleThreadedExecutor>()),

          // Start the executor in a separate thread
          executor_thread_([this]() { executor_->spin(); }) {

        // GUI Setup
        QVBoxLayout *layout = new QVBoxLayout(this);

        QPushButton *move_to_home_btn = new QPushButton("Move to Home Position", this);
        QPushButton *move_to_predefined_btn = new QPushButton("Move to Predefined Position", this);
        QPushButton *move_to_random_pose_btn = new QPushButton("Move to Random Pose", this);
        QPushButton *move_to_tf_btn = new QPushButton("Move to TF Position", this);
        QPushButton *refresh_frames_btn = new QPushButton("Refresh TF Frames", this);

        QPushButton *open_gripper_btn = new QPushButton("Open Gripper", this);
        QPushButton *close_gripper_btn = new QPushButton("Close Gripper", this);

        tf_frame_selector_ = new QComboBox(this);
        layout->addWidget(tf_frame_selector_);

        status_label_ = new QLabel("Status: Waiting for action", this);
        layout->addWidget(status_label_);

        layout->addWidget(move_to_home_btn);
        layout->addWidget(move_to_predefined_btn);
        layout->addWidget(move_to_random_pose_btn);
        layout->addWidget(move_to_tf_btn);
        layout->addWidget(refresh_frames_btn);
        layout->addWidget(open_gripper_btn);
        layout->addWidget(close_gripper_btn);

        connect(move_to_home_btn, &QPushButton::clicked, this, &MoveItControlGui::moveToHomePosition);
        connect(move_to_predefined_btn, &QPushButton::clicked, this, &MoveItControlGui::moveToPredefinedPosition);
        connect(move_to_random_pose_btn, &QPushButton::clicked, this, &MoveItControlGui::moveToRandomPose);
        connect(move_to_tf_btn, &QPushButton::clicked, this, &MoveItControlGui::moveToTf);
        connect(refresh_frames_btn, &QPushButton::clicked, this, &MoveItControlGui::refreshTfFrames);
        connect(open_gripper_btn, &QPushButton::clicked, this, &MoveItControlGui::openGripper);
        connect(close_gripper_btn, &QPushButton::clicked, this, &MoveItControlGui::closeGripper);

        setLayout(layout);
        refreshTfFrames(); // Refresh frames on startup
    }

    ~MoveItControlGui() {
        executor_->cancel();
        if (executor_thread_.joinable()) {
            executor_thread_.join();
        }
    }

private:
    rclcpp::Node::SharedPtr node_;
    moveit::planning_interface::MoveGroupInterface move_group_interface_;
    moveit::planning_interface::MoveGroupInterface gripper_move_group_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    QComboBox *tf_frame_selector_;
    QLabel *status_label_;
    rclcpp::executors::SingleThreadedExecutor::SharedPtr executor_;
    std::thread executor_thread_;

    void moveToHomePosition() {
        moveToPosition(0.006755, 0.000006, 0.777373, 0.0, 0.0, 1.0, 0.0);
    }

    void moveToPredefinedPosition() {
        moveToPosition(0.298652, 0.020795, 0.777373, 0.0, 0.0, 1.0, 0.0);
    }

    void moveToRandomPose() {
        // Generate random pose logic here
        double random_x = static_cast<double>(rand()) / RAND_MAX;
        double random_y = static_cast<double>(rand()) / RAND_MAX;
        double random_z = static_cast<double>(rand()) / RAND_MAX;
        moveToPosition(random_x, random_y, random_z, 0.0, 0.0, 1.0, 0.0);
    }

    void moveToTf() {
        // Logic to move to a pose based on TF frame selection
        std::string selected_frame = tf_frame_selector_->currentText().toStdString();
        // Retrieve the transform and move to the corresponding pose
    }

    void refreshTfFrames() {
        // Logic to refresh the TF frames in the combo box
        tf_frame_selector_->clear();
        // Populate tf_frame_selector_ with available frames
    }

    // Constants for gripper positions
    const double GRIPPER_OPEN_POSITION = 0.0;
    const double GRIPPER_CLOSED_POSITION = 0.99;

    void openGripper() {
        RCLCPP_INFO(node_->get_logger(), "Opening gripper...");
        moveGripperToPosition(GRIPPER_OPEN_POSITION);
    }

    void closeGripper() {
        RCLCPP_INFO(node_->get_logger(), "Closing gripper...");
        moveGripperToPosition(GRIPPER_CLOSED_POSITION);
    }

    void moveGripperToPosition(double position) {
        // Clamp the position to ensure it is within bounds (assuming 0.0 is open and 1.0 is closed)
        position = std::clamp(position, 0.0, 1.0);
        RCLCPP_INFO(node_->get_logger(), "Moving gripper to position: %.2f", position);

        // Set the target position for the ServoGear joint
        std::vector<std::string> joint_names = {"ServoGear"}; // Only the ServoGear joint
        std::vector<double> joint_positions = {position}; // Position to move to

        // Set the joint value target for the gripper
        gripper_move_group_.setJointValueTarget(joint_names, joint_positions);

        // Execute the plan for moving the gripper
        if (executeGripperPlan()) {
            RCLCPP_INFO(node_->get_logger(), "Gripper moved to position: %.2f", position);
            status_label_->setText(QString("Status: Gripper moved to position: %1").arg(position));
        } else {
            RCLCPP_ERROR(node_->get_logger(), "Gripper planning failed!");
            status_label_->setText("Status: Gripper planning failed!");
        }
    }

    bool executeGripperPlan() {
        moveit::planning_interface::MoveGroupInterface::Plan gripper_plan;
        if (gripper_move_group_.plan(gripper_plan) == moveit::core::MoveItErrorCode::SUCCESS) {
            RCLCPP_INFO(node_->get_logger(), "Gripper plan successful! Executing...");
            gripper_move_group_.execute(gripper_plan);
            return true;
        }
        return false;
    }

    void moveToPosition(double x, double y, double z, double ox, double oy, double oz, double ow) {
        geometry_msgs::msg::Pose target_pose;
        target_pose.position.x = x;
        target_pose.position.y = y;
        target_pose.position.z = z;
        target_pose.orientation.x = ox;
        target_pose.orientation.y = oy;
        target_pose.orientation.z = oz;
        target_pose.orientation.w = ow;

        move_group_interface_.setPoseTarget(target_pose);
        move_group_interface_.move();
        status_label_->setText("Status: Moved to target position");
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    QApplication app(argc, argv);
    rclcpp::Node::SharedPtr node = rclcpp::Node::make_shared("moveit_control_gui");
    MoveItControlGui gui(node);
    gui.show();
    return app.exec();
}