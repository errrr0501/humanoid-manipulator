#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>

int main(int argc, char * argv[])
{
  // Initialize ROS and create the Node
  rclcpp::init(argc, argv);
  auto const node = std::make_shared<rclcpp::Node>("pose_goal");

  // Create a ROS logger
  auto const logger = rclcpp::get_logger("pose_goal");

  // Create the MoveIt Move Group Interface for group
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

  // Create a target Pose for the end-link
  geometry_msgs::msg::Pose target_pose;
  target_pose.orientation.x = 1.0;
  target_pose.orientation.y = 0.0;
  target_pose.orientation.z = 0.0;
  target_pose.orientation.w = 0.0;
  target_pose.position.x = 0.83;
  target_pose.position.y = 0.27;
  target_pose.position.z = 0.35;

  // Set the target pose
  move_group_interface.setPoseTarget(target_pose);

  // Create a plan to these joint values and check if that plan is successful
  moveit::planning_interface::MoveGroupInterface::Plan left_arm_plan;
  bool left_arm_success = (move_group_interface.plan(left_arm_plan) == moveit::core::MoveItErrorCode::SUCCESS);


  /////////left thumb plan///////////////////////////////////////////////////////////////////////////////
  move_group_interface = MoveGroupInterface(node, "left_thumb");

  // Create a target Pose for the end-link
  // geometry_msgs::msg::Pose target_pose;
  target_pose.orientation.x = 1.0;
  target_pose.orientation.y = 0.0;
  target_pose.orientation.z = 0.0;
  target_pose.orientation.w = 0.0;
  target_pose.position.x = 0.83;
  target_pose.position.y = 0.27;
  target_pose.position.z = 0.35;

  // Set the target pose
  move_group_interface.setPoseTarget(target_pose);

  // Create a plan to these joint values and check if that plan is successful
  moveit::planning_interface::MoveGroupInterface::Plan left_thumb_plan;
  bool left_thumb_success = (move_group_interface.plan(left_thumb_plan) == moveit::core::MoveItErrorCode::SUCCESS);

  /////////left index plan///////////////////////////////////////////////////////////////////////////////
  move_group_interface = MoveGroupInterface(node, "left_index");

  // Create a target Pose for the end-link
  // geometry_msgs::msg::Pose target_pose;
  target_pose.orientation.x = 1.0;
  target_pose.orientation.y = 0.0;
  target_pose.orientation.z = 0.0;
  target_pose.orientation.w = 0.0;
  target_pose.position.x = 0.83;
  target_pose.position.y = 0.27;
  target_pose.position.z = 0.35;

  // Set the target pose
  move_group_interface.setPoseTarget(target_pose);

  // Create a plan to these joint values and check if that plan is successful
  moveit::planning_interface::MoveGroupInterface::Plan left_index_plan;
  bool left_index_success = (move_group_interface.plan(left_index_plan) == moveit::core::MoveItErrorCode::SUCCESS);

  /////////left middle plan//////////////////////////////////////////////////////////////////////////////
  move_group_interface = MoveGroupInterface(node, "left_middle");

  // Create a target Pose for the end-link
  // geometry_msgs::msg::Pose target_pose;
  target_pose.orientation.x = 1.0;
  target_pose.orientation.y = 0.0;
  target_pose.orientation.z = 0.0;
  target_pose.orientation.w = 0.0;
  target_pose.position.x = 0.83;
  target_pose.position.y = 0.27;
  target_pose.position.z = 0.35;

  // Set the target pose
  move_group_interface.setPoseTarget(target_pose);

  // Create a plan to these joint values and check if that plan is successful
  moveit::planning_interface::MoveGroupInterface::Plan left_middle_plan;
  bool left_middle_success = (move_group_interface.plan(left_middle_plan) == moveit::core::MoveItErrorCode::SUCCESS);

  /////////left ring plan///////////////////////////////////////////////////////////////////////////////
  move_group_interface = MoveGroupInterface(node, "left_ring");

  // Create a target Pose for the end-link
  // geometry_msgs::msg::Pose target_pose;
  target_pose.orientation.x = 1.0;
  target_pose.orientation.y = 0.0;
  target_pose.orientation.z = 0.0;
  target_pose.orientation.w = 0.0;
  target_pose.position.x = 0.83;
  target_pose.position.y = 0.27;
  target_pose.position.z = 0.35;

  // Set the target pose
  move_group_interface.setPoseTarget(target_pose);

  // Create a plan to these joint values and check if that plan is successful
  moveit::planning_interface::MoveGroupInterface::Plan left_ring_plan;
  bool left_ring_success = (move_group_interface.plan(left_ring_plan) == moveit::core::MoveItErrorCode::SUCCESS);

  /////////left pinky plan///////////////////////////////////////////////////////////////////////////////
  move_group_interface = MoveGroupInterface(node, "left_pinky");

  // Create a target Pose for the end-link
  // geometry_msgs::msg::Pose target_pose;
  target_pose.orientation.x = 1.0;
  target_pose.orientation.y = 0.0;
  target_pose.orientation.z = 0.0;
  target_pose.orientation.w = 0.0;
  target_pose.position.x = 0.83;
  target_pose.position.y = 0.27;
  target_pose.position.z = 0.35;

  // Set the target pose
  move_group_interface.setPoseTarget(target_pose);
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
  return 0;
}
