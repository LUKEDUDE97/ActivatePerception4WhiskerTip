#include <cmath>
#include <iostream>
#include <mutex>
#include <franka/exception.h>
#include <franka/robot.h>
#include "examples_common.h"
#include <ros/ros.h>
#include <whisker_customed_msg/FrankaCtrl.h>

#define TOTAL_VEL 0.02 // !DYX! : need to coincide with the setting in master node

struct ControlCommands {
  double x_velocity = 0.0;
  double y_velocity = TOTAL_VEL;
  double rotation = 0.0; // !DYX! : turned into absolute target in radians
  std::mutex mtx;
}; // !DYX! : we could test the initial motion first before officially start transporting ctrl_cmds

ControlCommands control_commands;
std::array<double, 16> initial_pose;

void controlCommandCallback(const whisker_customed_msg::FrankaCtrl::ConstPtr& msg) {
  std::lock_guard<std::mutex> lock(control_commands.mtx);
  control_commands.x_velocity = msg->xvel;
  control_commands.y_velocity = msg->yvel;
  control_commands.rotation = msg->orientation; // !DYX! : need to transform into angular velocity on Z-axis
}

int main(int argc, char** argv) {

  ros::init(argc, argv, "robot_ctrl");
  ros::NodeHandle nh;

  if (argc != 2) {
    ROS_ERROR("Usage: %s <robot-hostname>", argv[0]);
    return -1;
  }

  ros::Subscriber control_command_sub = nh.subscribe("/Franka_Ctrl", 10, controlCommandCallback);

  try {
    franka::Robot robot(argv[1]);
    setDefaultBehavior(robot);

    // First move the robot to a suitable joint configuration
    std::array<double, 7> q_goal = {{0, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, M_PI_4}};
    MotionGenerator motion_generator(0.5, q_goal);
    ROS_WARN("This example will move the robot! Please make sure to have the user stop button at hand!");
    ROS_INFO("Press Enter to continue...");
    std::cin.ignore();
    robot.control(motion_generator);
    ROS_INFO("Finished moving to initial joint configuration.");

    robot.setJointImpedance({{3000, 3000, 3000, 2500, 2500, 2000, 2000}});

    // Set the collision behavior.
    std::array<double, 7> lower_torque_thresholds_nominal{
        {25.0, 25.0, 22.0, 20.0, 19.0, 17.0, 14.0}};
    std::array<double, 7> upper_torque_thresholds_nominal{
        {35.0, 35.0, 32.0, 30.0, 29.0, 27.0, 24.0}};
    std::array<double, 7> lower_torque_thresholds_acceleration{
        {25.0, 25.0, 22.0, 20.0, 19.0, 17.0, 14.0}};
    std::array<double, 7> upper_torque_thresholds_acceleration{
        {35.0, 35.0, 32.0, 30.0, 29.0, 27.0, 24.0}};
    std::array<double, 6> lower_force_thresholds_nominal{{30.0, 30.0, 30.0, 25.0, 25.0, 25.0}};
    std::array<double, 6> upper_force_thresholds_nominal{{40.0, 40.0, 40.0, 35.0, 35.0, 35.0}};
    std::array<double, 6> lower_force_thresholds_acceleration{{30.0, 30.0, 30.0, 25.0, 25.0, 25.0}};
    std::array<double, 6> upper_force_thresholds_acceleration{{40.0, 40.0, 40.0, 35.0, 35.0, 35.0}};
    robot.setCollisionBehavior(
        lower_torque_thresholds_acceleration, upper_torque_thresholds_acceleration,
        lower_torque_thresholds_nominal, upper_torque_thresholds_nominal,
        lower_force_thresholds_acceleration, upper_force_thresholds_acceleration,
        lower_force_thresholds_nominal, upper_force_thresholds_nominal);

    double time_max = 4.0;
    double v_max = 0.1;
    double angle = M_PI / 4.0;
    double time = 0.0;
    robot.control([=, &time](const franka::RobotState&,
                             franka::Duration period) -> franka::CartesianVelocities {
      time += period.toSec();
      if (time == 0.0) {
        initial_pose = robot_state.O_T_EE_c;
      }

      // Transport X- ans Y-linear velocity and Z- angular velocity
      double v_x = control_commands.x_velocity;
      double v_y = control_commands.y_velocity;
      double w_z = control.commands.rotation; 

      franka::CartesianVelocities output = {{v_x, v_y, 0.0, 0.0, 0.0, w_z}};
      if (time >= 2 * time_max) {
        ROS_INFO("Finished motion, shutting down example");
        return franka::MotionFinished(output);
      }
      return output;
    });
  } catch (const franka::Exception& e) {
    ROS_ERROR("%s", e.what());
    return -1;
  }
  return 0;
}
