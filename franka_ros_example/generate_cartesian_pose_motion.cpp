#include <cmath>
#include <iostream>
#include <ros/ros.h>
#include <franka/exception.h>
#include <franka/robot.h>
#include "examples_common.h"

int main(int argc, char** argv) {
  // Initialize the ROS node
  ros::init(argc, argv, "motion_generator_pose");
  ros::NodeHandle nh;

  // Get the robot hostname from the ROS parameter server
  std::string robot_hostname;
  if (!nh.getParam("robot_hostname", robot_hostname)) {
    ROS_ERROR("Failed to get param 'robot_hostname'");
    return -1;
  }

  try {
    franka::Robot robot(robot_hostname);
    setDefaultBehavior(robot);

    // First move the robot to a suitable joint configuration
    std::array<double, 7> q_goal = {{0, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, M_PI_4}};
    MotionGenerator motion_generator(0.5, q_goal);
    ROS_WARN("This example will move the robot! Please make sure to have the user stop button at hand!");
    ROS_INFO("Press Enter to continue...");
    std::cin.ignore();
    robot.control(motion_generator);
    ROS_INFO("Finished moving to initial joint configuration.");

    // Set additional parameters always before the control loop, NEVER in the control loop!
    // Set collision behavior.
    robot.setCollisionBehavior(
        {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}}, {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
        {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}}, {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
        {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}}, {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}},
        {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}}, {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}});

    std::array<double, 16> initial_pose;
    double time = 0.0;
    robot.control([&time, &initial_pose](const franka::RobotState& robot_state,
                                         franka::Duration period) -> franka::CartesianPose {
      time += period.toSec();
      if (time == 0.0) {
        initial_pose = robot_state.O_T_EE_c;
      }
      constexpr double kRadius = 0.3;
      double angle = M_PI / 4 * (1 - std::cos(M_PI / 5.0 * time));
      double delta_x = kRadius * std::sin(angle);
      double delta_z = kRadius * (std::cos(angle) - 1);
      std::array<double, 16> new_pose = initial_pose;
      new_pose[12] += delta_x;
      new_pose[14] += delta_z;
      if (time >= 10.0) {
        ROS_INFO("Finished motion, shutting down example");
        return franka::MotionFinished(new_pose);
      }
      return new_pose;
    });
  } catch (const franka::Exception& e) {
    ROS_ERROR("%s", e.what());
    return -1;
  }
  return 0;
}
