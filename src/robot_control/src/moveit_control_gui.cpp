#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors/single_threaded_executor.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/msg/pose_stamped.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <QApplication>
#include <QPushButton>
#include <QVBoxLayout>
#include <QWidget>
#include <QComboBox>
#include <QLabel>
#include <thread>
#include <cmath>  // For M_PI

class MoveItControlGui : public QWidget {
public:
    MoveItControlGui(rclcpp::Node::SharedPtr node)
        : node_(node),
          move_group_interface_(node_, "arm"),
          tf_buffer_(node_->get_clock()),
          tf_listener_(tf_buffer_)
    {
        // Initialize the executor
        executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
        executor_->add_node(node_);

        // Start the executor in a separate thread
        executor_thread_ = std::thread([this]() { executor_->spin(); });

        // GUI Setup
        QVBoxLayout *layout = new QVBoxLayout(this);

        QPushButton *move_to_home_btn = new QPushButton("Move to Home Position", this);
        QPushButton *move_to_predefined_btn = new QPushButton("Move to Predefined Position", this);
        QPushButton *move_to_random_pose_btn = new QPushButton("Move to Random Pose", this);
        QPushButton *move_to_tf_btn = new QPushButton("Move to TF Position", this);
        QPushButton *refresh_frames_btn = new QPushButton("Refresh TF Frames", this);

        // Dropdown to select TF frames
        tf_frame_selector_ = new QComboBox(this);
        layout->addWidget(tf_frame_selector_);

        // Add status label to show the result of the planning
        status_label_ = new QLabel("Status: Waiting for action", this);
        layout->addWidget(status_label_);

        layout->addWidget(move_to_home_btn);
        layout->addWidget(move_to_predefined_btn);
        layout->addWidget(move_to_random_pose_btn);
        layout->addWidget(move_to_tf_btn);
        layout->addWidget(refresh_frames_btn);

        // Connect buttons
        connect(move_to_home_btn, &QPushButton::clicked, this, [this]() {
            moveToPosition(0.006755, 0.000006, 0.777373, 0.0, 0.0, 1.0, 0.0);  // Home position
        });

        connect(move_to_predefined_btn, &QPushButton::clicked, this, [this]() {
            moveToPosition(0.298652, 0.020795, 0.166739, 0.684743, 0.728771, -0.001997, -0.004029);  // Predefined position
        });

        connect(move_to_random_pose_btn, &QPushButton::clicked, this, &MoveItControlGui::moveToRandomPose);
        connect(move_to_tf_btn, &QPushButton::clicked, this, &MoveItControlGui::moveToTf);
        connect(refresh_frames_btn, &QPushButton::clicked, this, &MoveItControlGui::refreshTfFrames);

        setLayout(layout);

        // Refresh frames on startup
        refreshTfFrames();
    }

    ~MoveItControlGui()
    {
        // Shutdown the executor
        executor_->cancel();
        if (executor_thread_.joinable()) {
            executor_thread_.join();
        }
    }

private:
    rclcpp::Node::SharedPtr node_;
    moveit::planning_interface::MoveGroupInterface move_group_interface_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    QComboBox *tf_frame_selector_;
    QLabel *status_label_;

    // Executor and thread
    rclcpp::executors::SingleThreadedExecutor::SharedPtr executor_;
    std::thread executor_thread_;

    // Generic function to move to a position
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

        move_group_interface_.setPoseTarget(target_pose, "R5A_link5");
        move_group_interface_.setStartStateToCurrentState();
        // Uncomment and adjust tolerances if needed
        // move_group_interface_.setGoalPositionTolerance(0.01);
        // move_group_interface_.setGoalOrientationTolerance(0.1);
        RCLCPP_INFO(node_->get_logger(), "Orientation tolerance: %f", move_group_interface_.getGoalOrientationTolerance());
        RCLCPP_INFO(node_->get_logger(), "Position tolerance: %f", move_group_interface_.getGoalPositionTolerance());

        moveit::planning_interface::MoveGroupInterface::Plan my_plan;
        bool success = (move_group_interface_.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
        if (success) {
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
            // Look up the transform between "base_link" and the selected frame
            geometry_msgs::msg::TransformStamped transformStamped = tf_buffer_.lookupTransform(
                "base_link",  // Target frame (robot base frame)
                selected_frame.toStdString(),  // Source frame from dropdown
                tf2::TimePointZero);

            // Extract translation and add 0.1 units to the Z axis (move above the TF frame)
            double target_x = transformStamped.transform.translation.x;
            double target_y = transformStamped.transform.translation.y;
            double target_z = transformStamped.transform.translation.z + 0.1;  // Move 0.1 units above the frame

            // Set orientation to point downwards. Rotate 180 degrees around X-axis
            tf2::Quaternion orientation_down;
            orientation_down.setRPY(M_PI, 0, 0);  // 180-degree rotation around X-axis

            // Move the robot to the new position and orientation
            moveToPosition(
                target_x,
                target_y,
                target_z,
                orientation_down.x(),
                orientation_down.y(),
                orientation_down.z(),
                orientation_down.w());

        } catch (tf2::TransformException &ex) {
            RCLCPP_ERROR(node_->get_logger(), "TF Exception: %s", ex.what());
            status_label_->setText("Status: TF Exception occurred.");
        }
    }

    // Function to move to a random pose
    void moveToRandomPose() {
        RCLCPP_INFO(node_->get_logger(), "Moving to a random reachable pose...");

        // Set a random valid joint configuration
        move_group_interface_.setRandomTarget();

        // Get the target pose after setting the random target
        geometry_msgs::msg::PoseStamped random_pose = move_group_interface_.getCurrentPose();
        RCLCPP_INFO(node_->get_logger(), "Random target pose: x=%.3f, y=%.3f, z=%.3f",
                    random_pose.pose.position.x,
                    random_pose.pose.position.y,
                    random_pose.pose.position.z);

        // Plan and execute
        moveit::planning_interface::MoveGroupInterface::Plan my_plan;
        bool success = (move_group_interface_.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
        if (success) {
            RCLCPP_INFO(node_->get_logger(), "Plan successful! Executing...");
            status_label_->setText("Status: Plan successful! Executing...");
            move_group_interface_.execute(my_plan);
        } else {
            RCLCPP_ERROR(node_->get_logger(), "Planning failed!");
            status_label_->setText("Status: Planning failed!");
        }
    }

    // Function to refresh the list of available TF frames
    void refreshTfFrames() {
        RCLCPP_INFO(node_->get_logger(), "Refreshing list of TF frames...");

        // Get all frames from the TF buffer
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
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);

    auto node = std::make_shared<rclcpp::Node>(
        "moveit_control_gui",
        rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

    QApplication app(argc, argv);
    MoveItControlGui gui(node);
    gui.show();

    int result = app.exec();

    // Shutdown ROS after the GUI application exits
    rclcpp::shutdown();
    return result;
}
