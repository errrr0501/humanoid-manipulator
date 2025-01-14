#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>

// A helper to merge multiple JointTrajectories into one
trajectory_msgs::msg::JointTrajectory
mergeTrajectories(const std::vector<trajectory_msgs::msg::JointTrajectory>& trajectories)
{
  // Output container
  trajectory_msgs::msg::JointTrajectory merged;

  // A map from joint_name -> index in merged.joint_names
  std::unordered_map<std::string, size_t> joint_name_to_index;

  // 1) Build the merged list of joint names
  //    (We union all joint names from each trajectory)
  for (auto const & traj : trajectories)
  {
    for (auto const & joint_name : traj.joint_names)
    {
      // If this joint name doesn't exist in the merged yet, add it
      if (joint_name_to_index.find(joint_name) == joint_name_to_index.end())
      {
        joint_name_to_index[joint_name] = merged.joint_names.size();
        merged.joint_names.push_back(joint_name);
      }
    }
  }

  // 2) Determine the "longest" trajectory duration among all sub-trajectories
  rclcpp::Duration max_duration(0, 0);
  for (auto const & traj : trajectories)
  {
    if (!traj.points.empty())
    {
      auto const & last_point = traj.points.back();
      rclcpp::Duration dur(
        last_point.time_from_start.sec,
        last_point.time_from_start.nanosec
      );
      if (dur > max_duration)
        max_duration = dur;
    }
  }

  // 3) For simplicity, create some time samples from 0 to max_duration
  //    In a real implementation, you may want to *union* all time points from each sub-trajectory
  //    or do a more sophisticated interpolation. We'll do a naive approach here.
  double total_time = (double)max_duration.nanoseconds() / 1e9;  // seconds
  const size_t NUM_STEPS = 10;  // Or more for smoother interpolation
  double time_step = total_time / static_cast<double>(NUM_STEPS);

  // Prepare merged trajectory points
  merged.points.resize(NUM_STEPS+1);
  for (size_t i = 0; i <= NUM_STEPS; ++i)
  {
    merged.points[i].time_from_start =
      rclcpp::Duration::from_seconds(static_cast<double>(i) * time_step);

    // Each point must have a position value for *every* joint in merged.joint_names
    merged.points[i].positions.resize(merged.joint_names.size(), 0.0);
  }

  // 4) Fill in the merged trajectory by sampling each sub-trajectory
  //    We'll do a naive zero-order hold for positions (no interpolation).
  auto sampleJointPosition = [&](const trajectory_msgs::msg::JointTrajectory& traj,
                                 double t, const std::string& joint_name)
  {
    // If no points, return 0
    if (traj.points.empty()) return 0.0;

    // If t is beyond the last point, return last position
    if (t >= (double)traj.points.back().time_from_start.sec +
              (double)traj.points.back().time_from_start.nanosec / 1e9)
    {
      auto const & last_pt = traj.points.back();
      auto it = std::find(traj.joint_names.begin(), traj.joint_names.end(), joint_name);
      if (it == traj.joint_names.end()) return 0.0;
      size_t j = std::distance(traj.joint_names.begin(), it);
      return last_pt.positions[j];
    }

    // Otherwise, find the first trajectory point whose time_from_start >= t
    for (size_t i = 0; i < traj.points.size(); ++i)
    {
      double pt_time = (double)traj.points[i].time_from_start.sec +
                       (double)traj.points[i].time_from_start.nanosec / 1e9;
      if (pt_time >= t)
      {
        // Return that point's position for zero-order hold
        auto it = std::find(traj.joint_names.begin(), traj.joint_names.end(), joint_name);
        if (it == traj.joint_names.end()) return 0.0;
        size_t j = std::distance(traj.joint_names.begin(), it);
        return traj.points[i].positions[j];
      }
    }

    return 0.0; // Fallback (shouldn't happen)
  };

  // Fill each point in merged with positions from each sub-trajectory
  for (size_t i = 0; i <= NUM_STEPS; ++i)
  {
    double current_time = static_cast<double>(i) * time_step;

    // For each joint in the merged trajectory
    for (size_t joint_i = 0; joint_i < merged.joint_names.size(); ++joint_i)
    {
      std::string const & jname = merged.joint_names[joint_i];

      // Look for jname in each sub-traj & sample
      double pos_value = 0.0;
      for (auto const & traj : trajectories)
      {
        // if this sub-traj has jname, sample it
        if (std::find(traj.joint_names.begin(), traj.joint_names.end(), jname)
            != traj.joint_names.end())
        {
          pos_value = sampleJointPosition(traj, current_time, jname);
          break; // found it in this trajectory
        }
      }

      merged.points[i].positions[joint_i] = pos_value;
    }
  }

  return merged;
}
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


  std::vector<trajectory_msgs::msg::JointTrajectory> sub_trajectories;
  /////////left arm plan///////////////////////////////////////////////////////////////////////////////
  {
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
    // bool left_arm_success = (move_group_interface.plan(left_arm_plan) == moveit::core::MoveItErrorCode::SUCCESS);
    if (move_group_interface.plan(left_arm_plan)== moveit::core::MoveItErrorCode::SUCCESS)
    {
      RCLCPP_INFO(logger, "left_arm plan success");
      sub_trajectories.push_back(left_arm_plan.trajectory_.joint_trajectory);
    }
    else
      RCLCPP_ERROR(logger, "left_arm plan failed");
  }

  /////////left thumb plan///////////////////////////////////////////////////////////////////////////////
  {
    auto move_group_interface = MoveGroupInterface(node, "left_thumb");

    // Get all joint positions
    std::vector<double> joint_group_positions = move_group_interface.getCurrentJointValues();

    // Sets the second joint value
    joint_group_positions[0] = 0.785;  // radians
    move_group_interface.setJointValueTarget(joint_group_positions);

    // // Create a plan to these joint values and check if that plan is successful
    // moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    // bool success = (move_group_interface.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);

    // Create a plan to these joint values and check if that plan is successful
    moveit::planning_interface::MoveGroupInterface::Plan left_thumb_plan;
    // bool left_arm_success = (move_group_interface.plan(left_arm_plan) == moveit::core::MoveItErrorCode::SUCCESS);
    if (move_group_interface.plan(left_thumb_plan)== moveit::core::MoveItErrorCode::SUCCESS)
    {
      RCLCPP_INFO(logger, "left_thumb plan success");
      sub_trajectories.push_back(left_thumb_plan.trajectory_.joint_trajectory);
    }
    else
      RCLCPP_ERROR(logger, "left_thumb plan failed");
  }  

  /////////left index plan///////////////////////////////////////////////////////////////////////////////
  {
    auto move_group_interface = MoveGroupInterface(node, "left_index");

    // Get all joint positions
    std::vector<double> joint_group_positions = move_group_interface.getCurrentJointValues();

    // Sets the first and second joint value
    joint_group_positions[0] = 1.57;  // radians
    joint_group_positions[1] = 1.57;  // radians
    move_group_interface.setJointValueTarget(joint_group_positions);

    // Create a plan to these joint values and check if that plan is successful
    moveit::planning_interface::MoveGroupInterface::Plan left_index_plan;
    if (move_group_interface.plan(left_index_plan)== moveit::core::MoveItErrorCode::SUCCESS)
    {
      RCLCPP_INFO(logger, "left_index plan success");
      sub_trajectories.push_back(left_index_plan.trajectory_.joint_trajectory);
    }
    else
      RCLCPP_ERROR(logger, "left_index plan failed");
  }  

  /////////left middle plan//////////////////////////////////////////////////////////////////////////////
  {
    auto move_group_interface = MoveGroupInterface(node, "left_middle");

    // Get all joint positions
    std::vector<double> joint_group_positions = move_group_interface.getCurrentJointValues();

    // Sets the first and second joint value
    joint_group_positions[0] = 1.57;  // radians
    joint_group_positions[1] = 1.57;  // radians
    move_group_interface.setJointValueTarget(joint_group_positions);

    // Create a plan to these joint values and check if that plan is successful
    moveit::planning_interface::MoveGroupInterface::Plan left_middle_plan;
    if (move_group_interface.plan(left_middle_plan)== moveit::core::MoveItErrorCode::SUCCESS)
    {
      RCLCPP_INFO(logger, "left_middle plan success");
      sub_trajectories.push_back(left_middle_plan.trajectory_.joint_trajectory);
    }
    else
      RCLCPP_ERROR(logger, "left_middle plan failed");
  }  
  
  /////////left ring plan///////////////////////////////////////////////////////////////////////////////
  {
    auto move_group_interface = MoveGroupInterface(node, "left_ring");

    // Get all joint positions
    std::vector<double> joint_group_positions = move_group_interface.getCurrentJointValues();

    // Sets the first and second joint value
    joint_group_positions[0] = 1.57;  // radians
    joint_group_positions[1] = 1.57;  // radians
    move_group_interface.setJointValueTarget(joint_group_positions);

    // Create a plan to these joint values and check if that plan is successful
    moveit::planning_interface::MoveGroupInterface::Plan left_ring_plan;
    if (move_group_interface.plan(left_ring_plan)== moveit::core::MoveItErrorCode::SUCCESS)
    {
      RCLCPP_INFO(logger, "left_ring plan success");
      sub_trajectories.push_back(left_ring_plan.trajectory_.joint_trajectory);
    }
    else
      RCLCPP_ERROR(logger, "left_ring plan failed");
  }  
  
  /////////left pinky plan///////////////////////////////////////////////////////////////////////////////
  {
    auto move_group_interface = MoveGroupInterface(node, "left_pinky");

    // Get all joint positions
    std::vector<double> joint_group_positions = move_group_interface.getCurrentJointValues();

    // Sets the first and second joint value
    joint_group_positions[0] = 1.57;  // radians
    joint_group_positions[1] = 1.57;  // radians
    move_group_interface.setJointValueTarget(joint_group_positions);

    // Create a plan to these joint values and check if that plan is successful
    moveit::planning_interface::MoveGroupInterface::Plan left_pinky_plan;
    if (move_group_interface.plan(left_pinky_plan)== moveit::core::MoveItErrorCode::SUCCESS)
    {
      RCLCPP_INFO(logger, "left_pinky plan success");
      sub_trajectories.push_back(left_pinky_plan.trajectory_.joint_trajectory);
    }
    else
      RCLCPP_ERROR(logger, "left_pinky plan failed");
  }  
  
  // // If the plan is successful, execute the plan
  // if(left_arm_success) {
  //   move_group_interface.execute(left_arm_plan);
  //   move_group_interface.execute(left_thumb_plan);
  //   move_group_interface.execute(left_index_plan);
  //   move_group_interface.execute(left_middle_plan);
  //   move_group_interface.execute(left_ring_plan);
  //   move_group_interface.execute(left_pinky_plan);
  // } else {
  //   RCLCPP_ERROR(logger, "Planing failed!");
  // }


  if (sub_trajectories.empty())
  {
    RCLCPP_ERROR(logger, "No successful sub-group plans! Nothing to execute.");
  }
  else
  {
    // Merge all sub-group trajectories into one
    trajectory_msgs::msg::JointTrajectory merged_traj = mergeTrajectories(sub_trajectories);

    // Create a MoveIt Plan from the merged trajectory
    moveit_msgs::msg::RobotTrajectory robot_traj;
    robot_traj.joint_trajectory = merged_traj;

    moveit::planning_interface::MoveGroupInterface::Plan merged_plan;
    merged_plan.trajectory_ = robot_traj;

    // For execution, we can pick *any* group that contains (at least) all the relevant joints
    // or you can create a "dummy" group in the SRDF with all the robot joints
    auto move_group_interface = MoveGroupInterface(node, "left_arm");
    move_group_interface.execute(merged_plan);

    RCLCPP_INFO(logger, "Merged plan executed successfully!");
  }
  // Shutdown
  rclcpp::shutdown();
  spinner.join();
  return 0;
}
