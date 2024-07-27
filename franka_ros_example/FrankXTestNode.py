#!/usr/bin/env python3

import rospy
import time
import frankx

from argparse import ArgumentParser
from geometry_msgs.msg import Vector3, TwistStamped
from frankx import Affine, JointMotion, Robot, Waypoint, WaypointMotion

if __name__ == '__main__':

    parser = ArgumentParser()
    parser.add_argument('--host', default='192.168.178.12',
                        help='FCI IP of the robot')
    args = parser.parse_args()

    rospy.init_node('franka_whisker', anonymous=True)
    rate = rospy.Rate(5000)

    pub = rospy.Publisher("FrankaEE_state", TwistStamped, queue_size=10) # !DYX! : Temporarily use TwistStamped to represent the 6dim ee-pose

    rospy.loginfo("The Franka Robot has been started.")
    rospy.sleep(1)

    robot = Robot(args.host, repeat_on_error=False)
    gripper = robot.get_gripper()

    robot.set_default_behavior()
    robot.recover_from_errors()
    robot.set_dynamic_rel(0.2)

    # Reset to initial pose
    joint_motion = JointMotion([0, -0.785398163, 0, -2.35619449, 0, 1.57079632679, 0.785398163397])
    robot.move(joint_motion)

    # # Move the gripper if you needed and grasp the whisker sensor device
    # gripper.move(0.05)
    # gripper.clamp()

    # # Get in position for starting a contact trajectory
    # customed_motion = frankx.LinearMotion(Affine(0.466816, -0.009096, 0.183697))
    # robot.move(customed_motion)

    # robot.set_dynamic_rel(0.01)
    # motion = WaypointMotion([
    #     Waypoint(Affine(0.0, 0.10, 0.0), Waypoint.Relative),
    #     Waypoint(Affine(0.0, -0.10, 0.0), Waypoint.Relative),
    #     Waypoint(Affine(0.0, 0.10, 0.0), Waypoint.Relative),
    #     Waypoint(Affine(0.0, -0.10, 0.0), Waypoint.Relative),
    # ])
    # robot_thread = robot.move_async(motion)

    while not rospy.is_shutdown():

        try:
            robot_state = robot.get_state(read_once=True) # !DYX! : Remember change to True if you want to print out
        except frankx.InvalidOperationException:
            robot_state = robot.get_state(read_once=False)

        print("-----------------\n")
        print("Time: ", time.time())
        print("Cartesian EndPoint Pose - 3-dim coordinates and eulers: ", Affine(robot_state.O_T_EE))
        print("Cartesian EndPoint Pose - Transformation Matrix: ", robot_state.O_T_EE)
        print("Cartesian EndPoint Velocity: ", robot_state.O_dP_EE_c)
        
        state_msg = TwistStamped()
        state_msg.header.stamp = rospy.Time.now()
        state_msg.twist.linear.x = Affine(robot_state.O_T_EE).vector()[0]
        state_msg.twist.linear.y = Affine(robot_state.O_T_EE).vector()[1]
        state_msg.twist.angular.z = Affine(robot_state.O_T_EE).vector()[3]
        pub.publish(state_msg)

        rate.sleep()

    robot_thread.join()

    rospy.spin()
