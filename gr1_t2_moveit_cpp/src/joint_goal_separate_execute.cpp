#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>

int main(int argc, char * argv[])
{
  // Initialize ROS and create the Node
  rclcpp::init(argc, argv);
  auto const node = std::make_shared<rclcpp::Node>("joint_goal");

  // Create a ROS logger
  auto const logger = rclcpp::get_logger("joint_goal");

  // We spin up a SingleThreadedExecutor so we can get current joint values later
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  auto spinner = std::thread([&executor]() { executor.spin(); });

  // Create the MoveIt Move Group Interface for group
  using moveit::planning_interface::MoveGroupInterface;

  //////////////////group list////////////////////
  // left_arm
  // left_index
  // left_middle
  // left_pinky
  // left_ring
  // left_thumb
  // right_arm
  // right_index
  // right_middle
  // right_ring
  // right_thumb

  /////////left arm plan///////////////////////////////////////////////////////////////////////////////
  auto move_group_interface = MoveGroupInterface(node, "left_arm");

  // Get all joint positions
  std::vector<double> joint_group_positions = move_group_interface.getCurrentJointValues();

  // Sets the second joint value
  joint_group_positions[1] = 0.785;  // radians
  move_group_interface.setJointValueTarget(joint_group_positions);

  // // Create a plan to these joint values and check if that plan is successful
  // moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  // bool success = (move_group_interface.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);

  // Create a plan to these joint values and check if that plan is successful
  moveit::planning_interface::MoveGroupInterface::Plan left_arm_plan;
  bool left_arm_success = (move_group_interface.plan(left_arm_plan) == moveit::core::MoveItErrorCode::SUCCESS);


  /////////left thumb plan///////////////////////////////////////////////////////////////////////////////
  move_group_interface = MoveGroupInterface(node, "left_thumb");

  // Get all joint positions
  joint_group_positions = move_group_interface.getCurrentJointValues();

  // Sets the first joint value
  joint_group_positions[0] = 0.785;  // radians
  move_group_interface.setJointValueTarget(joint_group_positions);

  // Create a plan to these joint values and check if that plan is successful
  moveit::planning_interface::MoveGroupInterface::Plan left_thumb_plan;
  bool left_thumb_success = (move_group_interface.plan(left_thumb_plan) == moveit::core::MoveItErrorCode::SUCCESS);

  /////////left index plan///////////////////////////////////////////////////////////////////////////////
  move_group_interface = MoveGroupInterface(node, "left_index");

  // Get all joint positions
  joint_group_positions = move_group_interface.getCurrentJointValues();

  // Sets the first and second joint value
  joint_group_positions[0] = 1.57;  // radians
  joint_group_positions[1] = 1.57;  // radians
  move_group_interface.setJointValueTarget(joint_group_positions);

  // Create a plan to these joint values and check if that plan is successful
  moveit::planning_interface::MoveGroupInterface::Plan left_index_plan;
  bool left_index_success = (move_group_interface.plan(left_index_plan) == moveit::core::MoveItErrorCode::SUCCESS);

  /////////left middle plan//////////////////////////////////////////////////////////////////////////////
  move_group_interface = MoveGroupInterface(node, "left_middle");

  // Get all joint positions
  joint_group_positions = move_group_interface.getCurrentJointValues();

  // Sets the first and second joint value
  joint_group_positions[0] = 1.57;  // radians
  joint_group_positions[1] = 1.57;  // radians
  move_group_interface.setJointValueTarget(joint_group_positions);

  // Create a plan to these joint values and check if that plan is successful
  moveit::planning_interface::MoveGroupInterface::Plan left_middle_plan;
  bool left_middle_success = (move_group_interface.plan(left_middle_plan) == moveit::core::MoveItErrorCode::SUCCESS);

  /////////left ring plan///////////////////////////////////////////////////////////////////////////////
  move_group_interface = MoveGroupInterface(node, "left_ring");

  // Get all joint positions
  joint_group_positions = move_group_interface.getCurrentJointValues();

  // Sets the first and second joint value
  joint_group_positions[0] = 1.57;  // radians
  joint_group_positions[1] = 1.57;  // radians
  move_group_interface.setJointValueTarget(joint_group_positions);

  // Create a plan to these joint values and check if that plan is successful
  moveit::planning_interface::MoveGroupInterface::Plan left_ring_plan;
  bool left_ring_success = (move_group_interface.plan(left_ring_plan) == moveit::core::MoveItErrorCode::SUCCESS);

  /////////left pinky plan///////////////////////////////////////////////////////////////////////////////
  move_group_interface = MoveGroupInterface(node, "left_pinky");

  // Get all joint positions
  joint_group_positions = move_group_interface.getCurrentJointValues();

  // Sets the first and second joint value
  joint_group_positions[0] = 1.57;  // radians
  joint_group_positions[1] = 1.57;  // radians
  move_group_interface.setJointValueTarget(joint_group_positions);

  // Create a plan to these joint values and check if that plan is successful
  moveit::planning_interface::MoveGroupInterface::Plan left_pinky_plan;
  bool left_pinky_success = (move_group_interface.plan(left_pinky_plan) == moveit::core::MoveItErrorCode::SUCCESS);


  // If the plan is successful, execute the plan
  if(left_arm_success) {
    move_group_interface.execute(left_arm_plan);
    move_group_interface.execute(left_thumb_plan);
    move_group_interface.execute(left_index_plan);
    move_group_interface.execute(left_middle_plan);
    move_group_interface.execute(left_pinky_plan);
    move_group_interface.execute(left_ring_plan);
  } else {
    RCLCPP_ERROR(logger, "Planing failed!");
  }

  // Shutdown
  rclcpp::shutdown();
  spinner.join();
  return 0;
}
