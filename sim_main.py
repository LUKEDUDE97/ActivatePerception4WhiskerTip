import os
import mujoco
import numpy as np
import mediapy as media
import matplotlib.pyplot as plt
import scipy.interpolate as interpolate

from collections import deque
from filterpy.kalman import KalmanFilter


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
def update_noise_matrices(kf, measurements):
    # The measurement noise R - covariance matrix was determined and updated empicically by the most recently collected data points
    # sampled length : filter_window = N, including the value on current index
    zset = np.array(measurements)
    kf.R = np.cov(zset[:, 0], zset[:, 1])


# Refine the direction and limit the rotary speed
def refineOrientation(r):
    return


# Excuting parameters
contacted = 0  # Default contact flag
duration = 180  # Total excutating time duration (seconds)
servoing_ratio = 5  # Calculate and sent out the controls every N iterations

# Filtering parameters
filter_window = 10  # Update measurement noise empirically
scaling_factor_Q = 0.01  # Scaling factor on process noise
initial_state = [-0.015, 0.098] # !!! DYX !!! : To be defined, the fixed coordinate under our optimal deflection state
initial_variance = 10.0  # prior estimate variance P
tip_pos_s_deq = deque(maxlen=filter_window) # Maintain a fifo queue for unfiltered results of whisker base frame
tip_pos_w_filtered_que = []  # Final estimates of tip contacts under world-fixed frame

# Rotation - BSpline orientation parameters
theta_last_measured = 0.5 * np.pi
theta_next_desired = 0.5 * np.pi  # Default rotation ctrl while initializing
keypoint_interval = 5
keypoint_length = 5
u_next = 1 + 1 / (keypoint_length - 1)
keypoints_deq = deque(maxlen=keypoint_length)
n_interior_knots = 5
spline_degree = 3

# Translation - PI Controller parameters
X_VEL = 0.01
TOTAL_VEL = 0.2
PI_scale_bound = 20  # PI_scale_bound * X_VEL = TOTAL_VEL
DEF_TARGET = -2.8e-4  # Correspond to the optimal contact distance 15mm
GAIN_P = 500000
GAIN_I = 1000
def2Target_integral = 0
dt = 1.0  # dynamic time interval between servoing iteration

if __name__ == "__main__":
    # Import .xml model file
    filename = "../mujoco_model/whisker_v4_3_around_circle_r50_fixedBase.xml"
    file = open(filename, mode="r")
    xml = file.read()
    file.close()

    # Load mujoco model
    model = mujoco.MjModel.from_xml_string(xml)
    data = mujoco.MjData(model)
    renderer = mujoco.Renderer(model)
    mujoco.mj_forward(model, data)
    renderer.update_scene(data, camera="whisker_cam")
    media.show_image(renderer.render())

    # Bayesian filter initialize
    kf = KalmanFilter(dim_x=2, dim_z=2)
    kf.x = np.array([initial_state[0], initial_state[1]])
    kf.F = np.array([[1.0, 0.0], [0.0, 1.0]])
    kf.H = np.array([[1.0, 0.0], [0.0, 1.0]])
    kf.P *= initial_variance
    kf.R = np.eye(kf.dim_z)
    kf.Q = np.eye(kf.dim_x) * scaling_factor_Q

    # Set trajectory start point

    # Excuting active perception & tactile servoing
    loop_index = 0
    touch_index = 0
    time_start = data.time
    while data.time < time_start + duration:
        # Serial Node : Obtain deflection state at the moment
        deflection_moment = data.sensor("base2whisker_z").data.copy()

        # Franka Robot Node : Access the robot (ee) state
        ee_state = np.array(
            [0, 0, 0]
        )  # Initial state of end-effector in 2D plane under world frame : [base_X, base_Y, theta]
        ee_state[0] = data.sensor("whisker_joint_x").data.copy()
        ee_state[1] = data.sensor("whisker_joint_y").data.copy()
        ee_state[2] = data.sensor("whisker_joint_z").data.copy()

        # Master node on PC : Active Perception & Tactile Servoing
        if np.abs(deflection_moment) > 1e-4:  # Collision detection
            contacted = 1
            touch_index += 1
        if contacted and loop_index % servoing_ratio == 0:
            # Calculate tip position under whisker base frame
            tip_X_s = deflection2fx(deflection_moment)
            tip_Y_s = deflection2fy(deflection_moment)
            tip_pos_s_deq.append([tip_X_s, tip_Y_s])
            # Filter the tip position estimates
            if len(tip_pos_s_deq) == filter_window:
                update_noise_matrices(kf, tip_pos_s_deq)
                kf.predict(
                )  # !!! DYX !!! : better to transform state transition (static assumption) into world frame
                kf.update(tip_pos_s_deq[-1])
                tip_pos_s_filtered = kf.x.copy()
                # Transform the tip point into world-fixed frame
                tip_pos_w_filtered = np.dot(
                    np.array([
                        [
                            np.cos(ee_state[2]), -np.sin(ee_state[2]),
                            ee_state[0]
                        ],
                        [
                            np.sin(ee_state[2]),
                            np.cos(ee_state[2]), ee_state[1]
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
                            np.cos(ee_state[2]), -np.sin(ee_state[2]),
                            ee_state[0]
                        ],
                        [
                            np.sin(ee_state[2]),
                            np.cos(ee_state[2]), ee_state[1]
                        ],
                        [0, 0, 1],
                    ]),
                    np.array([[tip_X_s], [tip_Y_s], [1]]),
                )
                tip_pos_w_filtered_que.append(
                    [tip_pos_w_filtered[0], tip_pos_w_filtered[1]])

            # Rotary direction measurement in next iteration
            if touch_index % keypoint_interval == 0:
                keypoints_deq.append(tip_pos_w_filtered_que[-1])

                # Only if we have collected enough keypoints and it is a keypoint currently
                if len(keypoints_deq) == keypoint_length:
                    # Predict next contact point on BSpline curve
                    qs = np.linspace(0, 1, n_interior_knots + 2)[1:-1]
                    knots = np.quantile(keypoints_deq[:][0], qs)
                    tck, u = interpolate.splprep(
                        keypoints_deq[:][0],
                        keypoints_deq[:][1],
                        t=knots,
                        k=spline_degree,
                    )
                    tip_pos_w_next = interpolate.splev(u_next, tck)
                    # Transform slope into angular measurement
                    theta_next_measured = np.arctan2(
                        tip_pos_w_next[1] - keypoints_deq[-1][1],
                        tip_pos_w_next[0] - keypoints_deq[-1][0])
                    # Refine the measurement as accpetable desired rotary ctrl
                    theta_next_desired = refineOrientation(theta_next_measured)
                    theta_last_measured = theta_next_measured

            # Linear velocity magnitude in next iteration
            def2Target_error = deflection_moment - DEF_TARGET
            def2Target_integral += def2Target_error * dt
            PI_scale = GAIN_P * def2Target_error + GAIN_I * def2Target_integral
            PI_scale = max(min(PI_scale, PI_scale_bound), -PI_scale_bound)
            xvel_s_next_desired = PI_scale * X_VEL
            yvel_s_next_desired = np.sqrt(TOTAL_VEL**2 -
                                          xvel_s_next_desired**2)

            # Transform into world-fixed frame
            # !!! DYX !!! : transform angular degree into rotary matrix
            theta_w_next_desired = theta_next_desired - 0.5 * np.pi
            xvel_w_next_desired = xvel_s_next_desired * \
                np.cos(theta_w_next_desired) - yvel_s_next_desired * \
                np.sin(theta_w_next_desired)
            yvel_w_next_desired = xvel_s_next_desired * \
                np.sin(theta_w_next_desired) + yvel_s_next_desired * \
                np.cos(theta_w_next_desired)

            # Transmit to effector the franka robot node
            data.ctrl[0] = xvel_w_next_desired
            data.ctrl[1] = yvel_w_next_desired
            data.ctrl[2] = theta_w_next_desired

        mujoco.mj_step(model, data)
        loop_index += 1
