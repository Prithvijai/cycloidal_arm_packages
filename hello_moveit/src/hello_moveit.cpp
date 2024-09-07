#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_model_loader/robot_model_loader.h>

int main(int argc, char* argv[])
{
    // Initialize ROS and create the Node
    rclcpp::init(argc, argv);
    auto const node = std::make_shared<rclcpp::Node>(
        "hello_moveit", rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

    // Create a ROS logger
    auto const logger = rclcpp::get_logger("hello_moveit");

    // Create the MoveIt MoveGroup Interface
    using moveit::planning_interface::MoveGroupInterface;
    auto move_group_interface = MoveGroupInterface(node, "cycloidal_arm");

    // Load the robot model
    auto robot_model_loader = std::make_shared<robot_model_loader::RobotModelLoader>(node, "robot_description");
    auto robot_model = robot_model_loader->getModel();

    // Create an object for the robot's current state
    auto robot_state = std::make_shared<moveit::core::RobotState>(robot_model);

    // Set the named target Pose as "Fready"
    move_group_interface.setNamedTarget("Fready");

    // Create a plan to that named target pose
    auto const [success, plan] = [&move_group_interface] {
        moveit::planning_interface::MoveGroupInterface::Plan msg;
        auto const ok = static_cast<bool>(move_group_interface.plan(msg));
        return std::make_pair(ok, msg);
    }();

    // Execute the plan
    if (success)
    {
        move_group_interface.execute(plan);
        RCLCPP_INFO(logger, "Moved to Fready successfully.");

        // Manually set the robot's state to "Fready"
        const moveit::core::JointModelGroup* joint_model_group = robot_model->getJointModelGroup("cycloidal_arm");
        robot_state->setToDefaultValues(joint_model_group, "Fready");
        move_group_interface.setStartState(*robot_state);
        RCLCPP_INFO(logger, "Set 'Fready' as the current state manually.");
    }
    else
    {
        RCLCPP_ERROR(logger, "Planning to Fready failed!");
    }

    // Shutdown ROS
    rclcpp::shutdown();
    return 0;
}
