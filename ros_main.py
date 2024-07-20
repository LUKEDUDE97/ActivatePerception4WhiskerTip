#!/usr/bin/env python3

import rospy
from message_filters import Subscriber, ApproximateTimeSynchronizer
from geometry_msgs.msg import PoseStamped
from whisker_customed_msg.msg import MagneticFieldVector, EESensorState

class EESensorStateData:
    def __init__(self):
        self.current_time = rospy.Time()
        self.deflection_moment = 0.0
        self.xpos_ee = 0.0
        self.ypos_ee = 0.0
        self.zrot_ee = 0.0

state = EESensorStateData()

def callback(sensor_msg, frankaEE_msg):
    state.current_time = rospy.Time.now()
    state.deflection_moment = sensor_msg.magnetic_y
    state.xpos_ee = frankaEE_msg.pose.position.x
    state.ypos_ee = frankaEE_msg.pose.position.y
    state.zrot_ee = frankaEE_msg.pose.orientation.z

    rospy.loginfo("Time: %f, Deflection: %f, Position: (%f, %f), Orientation: %f",
                  state.current_time.to_sec(), state.deflection_moment,
                  state.xpos_ee, state.ypos_ee, state.zrot_ee)

    msg = EESensorState()
    msg.header.stamp = rospy.Time.now()
    msg.magnetic_y = sensor_msg.magnetic_y
    msg.pose_ee.position.x = frankaEE_msg.pose.position.x
    msg.pose_ee.position.y = frankaEE_msg.pose.position.y
    msg.pose_ee.orientation.z = frankaEE_msg.pose.orientation.z

    pub.publish(msg)

def main():
    rospy.init_node('Master_node', anonymous=True)

    # Wait for the first message from both topics
    rospy.wait_for_message('/Sensor_state', MagneticFieldVector)
    rospy.wait_for_message('/FrankaEE_state', PoseStamped)

    global pub
    pub = rospy.Publisher('/EE_Sensor_state', EESensorState, queue_size=10)

    # Subscribe to the /Sensor_state and /FrankaEE_state topics
    sensor_sub = Subscriber('/Sensor_state', MagneticFieldVector)
    frankaEE_sub = Subscriber('/FrankaEE_state', PoseStamped)

    # Define the synchronization policy and synchronizer
    ats = ApproximateTimeSynchronizer([sensor_sub, frankaEE_sub], queue_size=10, slop=0.1)

    # Register the callback with the synchronizer
    ats.registerCallback(callback)

    # Spin to process incoming messages
    rospy.spin()

if __name__ == '__main__':
    main()
