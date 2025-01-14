#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>

int main(int argc, char * argv[])
{
  // Initialize ROS and create the Node
  rclcpp::init(argc, argv);
  auto const node = std::make_shared<rclcpp::Node>("gripper_joint_value");

  // Create a ROS logger
  auto const logger = rclcpp::get_logger("gripper_joint_value");

  // Create the MoveIt Move Group Interface for panda hand (gripper)
  using moveit::planning_interface::MoveGroupInterface;
  auto move_group_interface = MoveGroupInterface(node, "gripper");

  // Set both gripper joints to same value
  move_group_interface.setJointValueTarget("robotiq_85_left_knuckle_joint", 0.803);

  // Create a plan to these joint values and check if that plan is successful
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  bool success = (move_group_interface.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);

  // If the plan is successful, execute the plan
  if(success) {
    move_group_interface.execute(my_plan);
  } else {
    RCLCPP_ERROR(logger, "Planing failed!");
  }

  // Shutdown
  rclcpp::shutdown();
  return 0;
}
