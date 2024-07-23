#!/usr/bin/env python3

import rospy
import numpy as np
import scipy.interpolate as interpolate

from message_filters import Subscriber, ApproximateTimeSynchronizer
from geometry_msgs.msg import PoseStamped
from whisker_customed_msg.msg import MagneticFieldVector, EESensorState, FrankaCtrl
from collections import deque
from filterpy.kalman import KalmanFilter

# Filtering parameters
initial_state = [-0.015, 0.098] # !!! DYX !!! : To be defined, the fixed coordinate under our optimal deflection state
initial_variance = 10.0  
filter_window = 10 
scaling_factor_Q = 0.01 
tip_pos_s_deq = deque(maxlen=filter_window) 
tip_pos_w_filtered_que = [] 

# Excuting parameters
collision_threshold = 1e-4 # !!! DYX !!! : To be defined, detect the collision when pass this value
contacted = 0
touch_index = 0

# Rotation decision parameters
keypoint_interval = 5
keypoint_length = 5
keypoints_deq = deque(maxlen=keypoint_length)
n_interior_knots = 5
spline_degree = 3
u_next = 1 + 1 / (keypoint_length - 1)
theta_last_measured = 0.5 * np.pi
theta_next_desired = 0.5 * np.pi

# Translation decision parameters
DEF_TARGET = -2.8e-4 # !!! DYX !!! : To be defined, correspond to the optimal cotact distance XX mm
KP = 500000
KI = 1000
KD = 0
PID_scale_bound = 20  # !!! DYX !!! : To be defined, PID_scale_bound * X_VEL = TOTAL_VEL
X_VEL = 0.01
TOTAL_VEL = 0.2


class EESensorStateData:
    def __init__(self) -> None:
        self.current_time = rospy.Time()
        self.deflection_moment = 0.0
        self.xpos_ee = 0.0
        self.ypos_ee = 0.0
        self.zrot_ee = 0.0


class FrankaRobotCtrl:
    def __init__(self) -> None:
        self.xvel = 0.0
        self.yvel = 0.0
        self.orientation = 0.0


class BayesianFilter:
    def __init__(self) -> None:
        self.f = KalmanFilter(dim_x=2, dim_z=2)
        self.f.x = np.array([initial_state[0], initial_state[1]])
        self.f.F = np.array([[1.0, 0.0], [0.0, 1.0]])
        self.f.H = np.array([[1.0, 0.0], [0.0, 1.0]])
        self.f.P *= initial_variance
        self.f.R = np.eye(self.f.dim_z)
        self.f.Q = np.eye(self.f.dim_x) * scaling_factor_Q


class PIDController:
    def __init__(self, Kp, Ki, Kd, set_point=0) -> None:
        self.Kp = Kp,
        self.Ki = Ki
        self.Kd = Kd
        self.set_point = set_point
        self.previous_error = 0
        self.integral = 0
        self.previous_time = rospy.get_time()

    def update(self, measured_value):
        current_time = rospy.get_time()
        dt = current_time - self.previous_time
        if dt <= 0.0:
            dt = 1e-6  # Avoid division by zero or negative time intervals

        error = self.set_point - measured_value
        self.integral += error * dt
        derivative = (error - self.previous_error) / dt

        output = self.Kp * error + self.Ki * self.integral + self.Kd * derivative
        self.previous_error = error
        self.previous_time = current_time

        return output


# Synchronized state information on end-effector and sensor
state = EESensorStateData()
# Calculated next ctrl command for franka robot
ctrl = FrankaRobotCtrl()
# Bayesian filter initialization
filter = BayesianFilter()
# PIDControll initialization
controller = PIDController(KP, KI, KD, DEF_TARGET)


# Calculate tip position via acquired measuremnt based on a characterized model : Y
def deflection2fy(an):
    fY = (-1.389e19 * an**5 - 1.317e16 * an**4 - 4.362e12 * an**3 -
          6.456e08 * an**2 - 2.075e05 * an - 2.149)
    return -fY / 1000


# Calculate tip position via acquired measuremnt based on a characterized model : X
def deflection2fx(an):
    fX = (-1.41e18 * an**5 - 8.617e14 * an**4 - 9.859e10 * an**3 +
          2.063e08 * an**2 + 5192 * an - 0.1542)
    return (100 - fX) / 1000


# Iteratively update the measurement noise
def update_noise_matrices(measurements):
    # The measurement noise R - covariance matrix was determined and updated empicically by the most recently collected data points
    # sampled length : filter_window = N, including the value on current index
    zset = np.array(measurements)
    filter.f.R = np.cov(zset[:, 0], zset[:, 1])


# Prevent the rotaion from exterme disturbance and refine the orientation
def refineOrientation(r):
    return


def callback(sensor_msg, frankaEE_msg):
    # Give access of the synchronized states (Serial Node & Franka Robot Node) to local object
    state.current_time = rospy.Time.now()
    state.deflection_moment = sensor_msg.magnetic_y
    state.xpos_ee = frankaEE_msg.pose.position.x
    state.ypos_ee = frankaEE_msg.pose.position.y
    state.zrot_ee = frankaEE_msg.pose.orientation.z

    rospy.loginfo("Time: %f, Deflection: %f, Position: (%f, %f), Orientation: %f",
                  state.current_time.to_sec(), state.deflection_moment,
                  state.xpos_ee, state.ypos_ee, state.zrot_ee)

    # Publish the current robot & sensor state
    # msg = EESensorState()
    # msg.header.stamp = rospy.Time.now()
    # msg.magnetic_y = sensor_msg.magnetic_y
    # msg.pose_ee.position.x = frankaEE_msg.pose.position.x
    # msg.pose_ee.position.y = frankaEE_msg.pose.position.y
    # msg.pose_ee.orientation.z = frankaEE_msg.pose.orientation.z
    # pub.publish(msg)

    # Collision detection
    if np.abs(state.deflection_moment) > collision_threshold:
        contacted = 1
        touch_index += 1
        
    # Calculate the contact and next ctrl here
    if contacted:  
        # Produce direct estimate of tip position
        tip_X_s = deflection2fx(state.deflection_moment)
        tip_Y_s = deflection2fy(state.deflection_moment)
        tip_pos_s_deq.append([tip_X_s, tip_Y_s])
        if len(tip_pos_s_deq) == filter_window:
            # Update measurement noise every iteration and filter the result
            update_noise_matrices(tip_pos_s_deq)
            filter.f.predict() # !!! DYX !!! : better to transform state transition (static assumption) into world frame
            filter.f.update(tip_pos_s_deq[-1])
            tip_pos_s_filtered = filter.f.x.copy()
            # Transform the filtered tip point into world-fixed frame
            tip_pos_w_filtered = np.dot(
                np.array([
                    [
                        np.cos(state.zrot_ee), -np.sin(state.zrot_ee),
                        state.xpos_ee
                    ],
                    [
                        np.sin(state.zrot_ee),
                        np.cos(state.zrot_ee), state.ypos_ee
                    ],
                    [0, 0, 1],
                ]),
                np.array([[tip_pos_s_filtered[0]], [tip_pos_s_filtered[1]],
                          [1]]),
            )
            tip_pos_w_filtered_que.append(
                [tip_pos_w_filtered[0], tip_pos_w_filtered[1]])
        else:
            tip_pos_w_filtered = np.dot(
                np.array([
                    [
                        np.cos(state.zrot_ee), -np.sin(state.zrot_ee),
                        state.xpos_ee
                    ],
                    [
                        np.sin(state.zrot_ee),
                        np.cos(state.zrot_ee), state.ypos_ee
                    ],
                    [0, 0, 1],
                ]),
                np.array([[tip_X_s], [tip_Y_s], [1]]),
            )
            tip_pos_w_filtered_que.append(
                [tip_pos_w_filtered[0], tip_pos_w_filtered[1]])

        # Rotary direction decision for next iteration
        if touch_index % keypoint_interval == 0:
            keypoints_deq.append(tip_pos_w_filtered_que[-1])

            # Only if we have collected enough keypoints and it is currently a keypoint, we make a decision
            if len(keypoints_deq) == keypoint_length:
                # Predict next contact position along BSpline curve
                qs = np.linspace(0, 1, n_interior_knots + 2)[1:-1]
                knots = np.quantile(keypoints_deq[:][0], qs)
                tck, u = interpolate.splprep(
                    keypoints_deq[:][0],
                    keypoints_deq[:][1],
                    t=knots,
                    k=spline_degree,
                )
                tip_pos_w_next = interpolate.splev(u_next, tck)
                # Transform a slope to next into angular measurement
                theta_next_measured = np.arctan2(
                    tip_pos_w_next[1] - keypoints_deq[-1][1],
                    tip_pos_w_next[0] - keypoints_deq[-1][0])
                # Refine the measurement as acceptable desired rotary ctrl
                theta_next_desired = refineOrientation(theta_next_measured)
                theta_last_measured = theta_next_measured

        # Linear translation decision for next iteration
        PID_scale = controller.update(state.deflection_moment)
        PID_scale = max(min(PID_scale, PID_scale_bound), -PID_scale_bound)
        xvel_s_next_desired = PID_scale * X_VEL
        yvel_s_next_desired = np.sqrt(TOTAL_VEL**2 - xvel_s_next_desired**2)

        # Transform all the ctrls into world-fixed frame and usable format
        # !!! DYX !!! : adapt to the franka frame first and rotation expression
        theta_w_next_desired = theta_next_desired - 0.5 * np.pi
        xvel_w_next_desired = xvel_s_next_desired * \
            np.cos(theta_w_next_desired) - yvel_s_next_desired * \
            np.sin(theta_w_next_desired)
        yvel_w_next_desired = xvel_s_next_desired * \
            np.sin(theta_w_next_desired) + yvel_s_next_desired * \
            np.cos(theta_w_next_desired)

        # Publish and transport the ctrl commands
        ctrl_msg = FrankaCtrl()
        ctrl_msg.xvel = xvel_w_next_desired
        ctrl_msg.yvel = yvel_w_next_desired
        ctrl_msg.orientation = theta_w_next_desired
        ctrl_pub.publish(ctrl_msg)


def main():
    rospy.init_node('Master_node', anonymous=True)

    # Wait for the first message from both topics
    rospy.wait_for_message('/Sensor_state', MagneticFieldVector)
    rospy.wait_for_message('/FrankaEE_state', PoseStamped)

    global state_pub  # State publisher
    state_pub = rospy.Publisher(
        '/EE_Sensor_state', EESensorState, queue_size=10)
    global ctrl_pub  # Ctrl publisher
    ctrl_pub = rospy.Publisher('/Franka_Ctrl', FrankaCtrl, queue_size=10)

    # Subscribe to the /Sensor_state and /FrankaEE_state topics
    sensor_sub = Subscriber('/Sensor_state', MagneticFieldVector)
    frankaEE_sub = Subscriber('/FrankaEE_state', PoseStamped)

    # Define the synchronization policy and synchronizer
    ats = ApproximateTimeSynchronizer(
        [sensor_sub, frankaEE_sub], queue_size=10, slop=0.1)

    # Register the callback with the synchronizer
    ats.registerCallback(callback)

    # Spin to process incoming messages
    rospy.spin()


if __name__ == '__main__':
    main()