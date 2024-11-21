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
        moveToPosition(0.298652, 0.020795, 0.166739, 0.684743, 0.728771, -0.001997, -0.004029);
    }

    void moveToPosition(double x, double y, double z, double qx, double qy, double qz, double qw) {
        RCLCPP_INFO(node_->get_logger(), "Moving to position: x=%.3f, y=%.3f, z=%.3f", x, y, z);

        geometry_msgs::msg::Pose target_pose;
        target_pose.position.x = x;
        target_pose.position.y = y;
        target_pose.position.z = z;
        target_pose.orientation.x = qx;
        target_pose.orientation.y = qy;
        target_pose.orientation.z = qz;
        target_pose.orientation.w = qw;

        move_group_interface_.setPoseTarget(target_pose);
        move_group_interface_.setStartStateToCurrentState();

        moveit::planning_interface::MoveGroupInterface::Plan my_plan;
        if (move_group_interface_.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS) {
            RCLCPP_INFO(node_->get_logger(), "Plan successful! Executing...");
            status_label_->setText("Status: Plan successful! Executing...");
            move_group_interface_.execute(my_plan);
        } else {
            RCLCPP_ERROR(node_->get_logger(), "Planning failed!");
            status_label_->setText("Status: Planning failed!");
        }
    }

    void moveToTf() {
        QString selected_frame = tf_frame_selector_->currentText();
        if (selected_frame.isEmpty()) {
            RCLCPP_ERROR(node_->get_logger(), "No TF frame selected.");
            status_label_->setText("Status: No TF frame selected.");
            return;
        }

        RCLCPP_INFO(node_->get_logger(), "Moving to position from TF broadcaster: %s", selected_frame.toStdString().c_str());

        try {
            geometry_msgs::msg::TransformStamped transformStamped = tf_buffer_.lookupTransform("base_link", selected_frame.toStdString(), tf2::TimePointZero);
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

    void moveToRandomPose() {
        RCLCPP_INFO(node_->get_logger(), "Moving to a random reachable pose...");
        move_group_interface_.setRandomTarget();

        moveit::planning_interface::MoveGroupInterface::Plan my_plan;
        if (move_group_interface_.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS) {
            RCLCPP_INFO(node_->get_logger(), "Plan successful! Executing...");
            status_label_->setText("Status: Plan successful! Executing...");
            move_group_interface_.execute(my_plan);
        } else {
            RCLCPP_ERROR(node_->get_logger(), "Planning failed!");
            status_label_->setText("Status: Planning failed!");
        }
    }

    void refreshTfFrames() {
        RCLCPP_INFO(node_->get_logger(), "Refreshing list of TF frames...");
        std::vector<std::string> frame_list = tf_buffer_.getAllFrameNames();
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
};

/**
 * @brief Main function to initialize the ROS node and start the GUI application.
 *
 * @param argc Argument count.
 * @param argv Argument vector.
 *
 * @return Exit code of the application.
 */
int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("moveit_control_gui", rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

    QApplication app(argc, argv);
    MoveItControlGui gui(node);
    gui.show();

    int result = app.exec();
    rclcpp::shutdown();
    return result;
}