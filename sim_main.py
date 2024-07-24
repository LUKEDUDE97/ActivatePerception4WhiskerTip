import os
import mujoco
import numpy as np
import mediapy as media
import matplotlib.pyplot as plt
import scipy.interpolate as interpolate

from collections import deque
from filterpy.kalman import KalmanFilter


# Excuting parameters
contacted = 0  # Default contact flag
duration = 15  # Total excutating time duration (seconds)
servoing_ratio = 5  # Calculate and sent out the controls every N iterations
stable_distance = 60

# Filtering parameters
filter_window = 20  # Update measurement noise empirically
scaling_factor_Q = 0.01  # Scaling factor on process noise
initial_state = [-0.015, 0.098] # !!! DYX !!! : To be defined, the fixed coordinate under our optimal deflection state
initial_variance = 10.0  # prior estimate variance P
tip_pos_s_deq = deque(maxlen=filter_window) # Maintain a fifo queue for unfiltered results of whisker base frame
tip_pos_w_filtered_que = []  # Final estimates of tip contacts under world-fixed frame

# Rotation - BSpline orientation parameters
theta_last_measured = 0.5 * np.pi
theta_last_desired = 0.5 * np.pi
theta_next_desired = 0.5 * np.pi  # Default rotation ctrl while initializing
keypoint_interval = 3
keypoint_length = 10
u_next = 1 + 1 / (keypoint_length - 1)
keypoints_deq = deque(maxlen=keypoint_length)
n_interior_knots = 5
spline_degree = 3
wrap_count = 0
initial_slope = 0.5 * np.pi

# Translation - PI Controller parameters
X_VEL = 0.01
TOTAL_VEL = 0.2
PI_scale_bound = 20  # PI_scale_bound * X_VEL = TOTAL_VEL
DEF_TARGET = -2.8e-4  # Correspond to the optimal contact distance 15mm
GAIN_P = 500000
GAIN_I = 1000
def2Target_integral = 0
dt = 1.0  # dynamic time interval between servoing iteration


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
    global wrap_count
    #  Use last original measurement (-pi, pi) to detect a full turn
    if r - theta_last_measured > 1.888 * np.pi:
        wrap_count += 1
    elif r - theta_last_measured < -1.888 * np.pi:
        wrap_count -= 1
    r_refined = r - wrap_count * 2 * np.pi

    # rotary speed limitation
    if touch_index >= stable_distance:
        # Multiply a servoing ratio to regulat our limit, lower ratio means larger direction change between every decision
        rot_speed_limit = 0.00025 * servoing_ratio * keypoint_interval
        # Use last desired of accumulated rotation to limit speed
        theta_err = r_refined - theta_last_desired
        if (theta_err > rot_speed_limit and theta_err < 0.5 * np.pi) or \
            (theta_err < - 1.5*np.pi and theta_err > - 2*np.pi + rot_speed_limit) or \
            (theta_err <= -0.5*np.pi and theta_err >= -np.pi) or \
                (theta_err >= np.pi and theta_err <= 1.5*np.pi):
            r_refined = theta_last_desired + rot_speed_limit
        elif (theta_err < -rot_speed_limit and theta_err > -0.5 * np.pi) or \
            (theta_err > 1.5*np.pi and theta_err < 2*np.pi - rot_speed_limit) or \
            (theta_err >= 0.5*np.pi and theta_err <= np.pi) or \
                (theta_err <= - np.pi and theta_err >= - 1.5*np.pi):
            r_refined = theta_last_desired - rot_speed_limit

    # r : (-pi, pi); r_refined : no upper bound accumlated rotation
    return r_refined


# Recordings for test
df_theta_next_measured = []
df_theta_next_desired = []
df_deflection_moment = []


if __name__ == "__main__":
    # Import .xml model file
    filename = "../mujoco_model/whisker_v4_3_1_around_circle_r50_fixedBase.xml"
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

    # Mujoco render settings
    framerate = 30  # (Hz)
    frames = []  # for rendering
    mujoco.mj_resetData(model, data)  # Reset state and time

    # Bayesian filter initialize
    kf = KalmanFilter(dim_x=2, dim_z=2)
    kf.x = np.array([initial_state[0], initial_state[1]])
    kf.F = np.array([[1.0, 0.0], [0.0, 1.0]])
    kf.H = np.array([[1.0, 0.0], [0.0, 1.0]])
    kf.P *= initial_variance
    kf.R = np.eye(kf.dim_z)
    kf.Q = np.eye(kf.dim_x) * scaling_factor_Q

    # Set trajectory start motion
    data.ctrl[1] = TOTAL_VEL

    # Excuting active perception & tactile servoing
    loop_index = 0
    touch_index = 0
    time_start = data.time
    while data.time < time_start + duration:
        # Serial Node : Obtain deflection state at the moment
        [deflection_moment] = data.sensor("base2whisker_z").data.copy()

        # Franka Robot Node : Access the robot (ee) state
        ee_state = np.array([0.0, 0.0, 0.0])         # Initial state of end-effector in 2D plane under world frame : [base_X, base_Y, theta]
        ee_state[0] = data.sensor("whisker_joint_x").data.copy()
        ee_state[1] = data.sensor("whisker_joint_y").data.copy()
        ee_state[2] = data.sensor("whisker_joint_z").data.copy()

        # Master node on PC : Active Perception & Tactile Servoing
        if np.abs(deflection_moment) > 1e-4:  # Collision detection
            contacted = 1

        if contacted and loop_index % servoing_ratio == 0:
            touch_index += 1
            # Calculate tip position under whisker base frame
            tip_X_s = deflection2fx(deflection_moment)
            tip_Y_s = deflection2fy(deflection_moment)
            tip_pos_s_deq.append([tip_X_s, tip_Y_s])
            # Filter the tip position estimates
            if len(tip_pos_s_deq) == filter_window:
                update_noise_matrices(kf, tip_pos_s_deq)
                kf.predict()  # !!! DYX !!! : better to transform state transition (static assumption) into world frame
                kf.update(tip_pos_s_deq[-1])
                tip_pos_s_filtered = kf.x.copy()
                # Transform the tip point into world-fixed frame
                tip_pos_w_filtered = np.dot(
                    np.array([
                        [np.cos(ee_state[2]), -
                         np.sin(ee_state[2]), ee_state[0]],
                        [np.sin(ee_state[2]), np.cos(
                            ee_state[2]), ee_state[1]],
                        [0, 0, 1],
                    ]),
                    np.array([[tip_pos_s_filtered[0]], [
                             tip_pos_s_filtered[1]], [1]]),
                )
                tip_pos_w_filtered_que.append(
                    [tip_pos_w_filtered[0][0], tip_pos_w_filtered[1][0]])
            else:
                tip_pos_w_filtered = np.dot(
                    np.array([
                        [np.cos(ee_state[2]), -
                         np.sin(ee_state[2]), ee_state[0]],
                        [np.sin(ee_state[2]), np.cos(
                            ee_state[2]), ee_state[1]],
                        [0, 0, 1],
                    ]),
                    np.array([[tip_X_s], [tip_Y_s], [1]]),
                )
                tip_pos_w_filtered_que.append(
                    [tip_pos_w_filtered[0][0], tip_pos_w_filtered[1][0]])

            # Rotary direction measurement in next iteration
            if touch_index % keypoint_interval == 0:
                keypoints_deq.append(tip_pos_w_filtered_que[-1])

                # Only if we have collected enough keypoints and it is a keypoint currently
                if len(keypoints_deq) == keypoint_length:
                    # Predict next contact point on BSpline curve
                    qs = np.linspace(0, 1, n_interior_knots + 2)[1:-1]
                    knots = np.quantile(np.array(keypoints_deq)[:, 1], qs)
                    tck, u = interpolate.splprep(
                        [np.array(keypoints_deq)[:, 0],
                         np.array(keypoints_deq)[:, 1]],
                        t=knots,
                        k=spline_degree,
                    )
                    tip_pos_w_next = interpolate.splev(u_next, tck)
                    # Transform slope into angular measurement
                    theta_next_measured = np.arctan2(
                        tip_pos_w_next[1] - np.array(keypoints_deq)[-1][1],
                        tip_pos_w_next[0] - np.array(keypoints_deq)[-1][0])
                    # Refine the measurement as accpetable desired rotary ctrl
                    theta_next_desired = refineOrientation(theta_next_measured)
                    theta_last_desired = theta_next_desired
                    theta_last_measured = theta_next_measured
                else:
                    theta_next_measured = np.pi/2
                    theta_next_desired = theta_next_measured

            # Linear velocity magnitude in next iteration
            def2Target_error = deflection_moment - DEF_TARGET
            def2Target_integral += def2Target_error * dt
            PI_scale = GAIN_P * def2Target_error + GAIN_I * def2Target_integral
            PI_scale = max(min(PI_scale, PI_scale_bound), -PI_scale_bound)
            xvel_s_next_desired = PI_scale * X_VEL
            yvel_s_next_desired = np.sqrt(
                TOTAL_VEL**2 - xvel_s_next_desired**2)

            # Transform into world-fixed frame
            theta_w_next_desired = theta_next_desired - initial_slope
            xvel_w_next_desired = xvel_s_next_desired * \
                np.cos(theta_w_next_desired) - yvel_s_next_desired * \
                np.sin(theta_w_next_desired)
            yvel_w_next_desired = xvel_s_next_desired * \
                np.sin(theta_w_next_desired) + yvel_s_next_desired * \
                np.cos(theta_w_next_desired)

            # Transmit to effector the franka robot node
            if touch_index >= stable_distance:
                data.ctrl[0] = xvel_w_next_desired
                data.ctrl[1] = yvel_w_next_desired
                data.ctrl[2] = theta_w_next_desired

            # Recordings for testing
            df_theta_next_measured.append(theta_next_measured)
            df_theta_next_desired.append(theta_next_desired)
            df_deflection_moment.append(deflection_moment)

        mujoco.mj_step(model, data)
        loop_index += 1

        # if len(frames) < data.time * framerate:
        #     renderer.update_scene(data, camera='whisker_cam')
        #     pixels = renderer.render()
        #     frames.append(pixels)

    # media.show_video(frames, fps=framerate)

    # plt.figure(figsize=(10, 6))
    # plt.grid()
    # plt.plot(np.array(tip_pos_w_filtered_que)[:,1], np.array(tip_pos_w_filtered_que)[:,0] * -1,'o')
    # plt.title('tip_pos')

    # x = np.linspace(1, len(df_theta_next_measured), len(df_theta_next_measured))
    # plt.figure(figsize=(10, 6))
    # plt.grid()
    # plt.plot(x[:], df_theta_next_measured[:],'o')
    # plt.title('df_theta_next_measured')

    # plt.figure(figsize=(10, 6))
    # plt.grid()
    # plt.plot(x[:], df_theta_next_desired[:],'o')
    # plt.title('df_theta_next_desired')

    # x = np.linspace(1, len(df_deflection_moment), len(df_deflection_moment))
    # plt.figure(figsize=(10, 6))
    # plt.grid()
    # plt.plot(x[:], df_deflection_moment[:],'o')
    # plt.title('df_deflection_moment')
