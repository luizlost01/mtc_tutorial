#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>

int main(int argc, char * argv[])
{
  // Initialize ROS and create the Node
  rclcpp::init(argc, argv);
  auto const node = std::make_shared<rclcpp::Node>(
    "hello_moveit",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
  );

  // Create a ROS logger
  auto const logger = rclcpp::get_logger("hello_moveit");

  // Wait for robot_description_semantic parameter
  RCLCPP_INFO(logger, "Waiting for robot_description_semantic parameter...");
  while (!node->has_parameter("robot_description_semantic")) {
    if (!rclcpp::ok()) {
      return 1;
    }
    rclcpp::sleep_for(std::chrono::milliseconds(100));
  }
  RCLCPP_INFO(logger, "robot_description_semantic parameter found!");

  // Create the MoveIt MoveGroup Interface
  using moveit::planning_interface::MoveGroupInterface;
  auto move_group_interface = MoveGroupInterface(node, "xarm6");

  // Wait for current state to be available
  RCLCPP_INFO(logger, "Waiting for robot state...");
  move_group_interface.getCurrentState(10.0);
  RCLCPP_INFO(logger, "Got current robot state!");

  // Set a simple joint target (home position with small offset)
  std::vector<double> joint_values = {0.0, -0.5, 0.0, 0.5, 0.0, 0.0};
  move_group_interface.setJointValueTarget(joint_values);
  RCLCPP_INFO(logger, "Planning to joint target...");

  // Create a plan to that target pose
  auto const [success, plan] = [&move_group_interface]{
    moveit::planning_interface::MoveGroupInterface::Plan msg;
    auto const ok = static_cast<bool>(move_group_interface.plan(msg));
    return std::make_pair(ok, msg);
  }();

// Execute the plan
if(success) {
  move_group_interface.execute(plan);
} else {
  RCLCPP_ERROR(logger, "Planning failed!");
}

  // Shutdown ROS
  rclcpp::shutdown();
  return 0;
}