#include <cmath>
#include <iostream>
#include <mutex>
#include <franka/exception.h>
#include <franka/robot.h>
#include "examples_common.h"
#include <ros/ros.h>
#include <whisker_customed_msg/FrankaCtrl.h>

#define TOTAL_VEL 0.2 // !DYX! : need to coincide with the setting in master node

struct ControlCommands {
  double x_velocity = 0.0;
  double y_velocity = TOTAL_VEL;
  double rotation = 0.5 & M_PI; // !DYX! : turned into absolute target in radians
  std::mutex mtx;
}; // !DYX! : we could test the initial motion first before officially start transporting ctrl_cmds

ControlCommands control_commands;
std::array<double, 16> initial_pose;

void controlCommandCallback(const whisker_customed_msg::FrankaCtrl::ConstPtr& msg) {
  std::lock_guard<std::mutex> lock(control_commands.mtx);
  control_commands.x_velocity = msg->xvel;
  control_commands.y_velocity = msg->yvel;
  // !DYX! : still need to make sure it's the ee absolute rotation state under world-fixed frame
  control_commands.rotation = msg->orientation;
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "robot_control");
  ros::NodeHandle nh;

  if (argc != 2) {
    ROS_ERROR("Usage: %s <robot-hostname>", argv[0]);
    return -1;
  }

  ros::Subscriber control_command_sub = nh.subscribe("/Franka_Ctrl", 10, controlCommandCallback);

  try {
    franka::Robot robot(argv[1]);
    setDefaultBehavior(robot);

    std::array<double, 7> q_goal = {{0, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, M_PI_4}};
    MotionGenerator motion_generator(0.5, q_goal);
    ROS_WARN("This example will move the robot! Please make sure to have the user stop button at hand!");
    ROS_INFO("Press Enter to continue...");
    std::cin.ignore();

    robot.control(motion_generator);
    ROS_INFO("Finished moving to initial joint configuration.");

    robot.setCollisionBehavior(
        {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}}, {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
        {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}}, {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
        {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}}, {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}},
        {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}}, {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}});

    double time = 0.0;
    robot.control([&time](const franka::RobotState& robot_state,
                          franka::Duration period) -> franka::CartesianPose {
      time += period.toSec();
      if (time == 0.0) {
        initial_pose = robot_state.O_T_EE_c;
      }

      //   <<< Target Cartesian Pose format in this task >>>
      // | cos(target_theta) -sin(target_theta)  0  +delta_x |
      // | sin(target_theta)  cos(target_theta)  0  +delta_y |
      // |       0                   0           1     0     |
      // |       0                   0           0     1     |

      std::array<double, 16> new_pose = robot_state.O_T_EE_c;
      double delta_x, delta_y, target_theta;
      {
        std::lock_guard<std::mutex> lock(control_commands.mtx);
        delta_x = control_commands.x_velocity * period.toSec();
        delta_y = control_commands.y_velocity * period.toSec();
        target_theta = control_commands.rotation;
      }

      // Linear translation on X- and Y-axis in a 2d plane
      new_pose[12] += delta_x;
      new_pose[13] += delta_y;

      // Move to absolute radians target by a pure rotation around Z-axis 
      double cos_theta = std::cos(target_theta);
      double sin_theta = std::sin(target_theta);
      new_pose[0] = cos_theta;   
      new_pose[1] = -sin_theta;  
      new_pose[4] = sin_theta;   
      new_pose[5] = cos_theta;   

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