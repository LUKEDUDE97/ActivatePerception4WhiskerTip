import distutils.util
import os
# Other imports and helper functions
import time
import itertools
import numpy as np

import mujoco
# Graphics and plotting.
import mediapy as media
import matplotlib.pyplot as plt

from filterpy.kalman import KalmanFilter
from matplotlib.patches import Ellipse

import scipy.interpolate as interpolate


# Define the angle-tip relation function
def angle2fy(an):
    fx = -1.389e+19 *an**5 - 1.317e+16 *an**4 - 4.362e+12 *an**3 - 6.456e+08 *an**2 - 2.075e+05 *an - 2.149
    # fx = -5.103e+18 *an**5 - 1.188e+15 *an**4 + 6.998e+11 *an**3 + 2.433e+08 *an**2 - 1.447e+05 *an - 0.8301
    return -fx/1000
def angle2fx(an):
    fy = -1.41e+18 *an**5 - 8.617e+14 *an**4 - 9.859e+10 *an**3 + 2.063e+08 *an**2 + 5192 *an - 0.1542
    # fy = -3.739e+18 *an**5 - 2.739e+15 *an**4 - 6.238e+11 *an**3 + 1.393e+08 *an**2 + 2742 *an - 0.08576
    return (100-fy)/1000

def update_noise_matrices(kf, measurements):

    # The measurement noise R - covariance matrix was determined and updated empicically by the most recently collected data points
    # sampled length : window_size = 5, including the value on current index

    zset = np.array(measurements)
    kf.R = np.cov(zset[:, 0], zset[:, 1])

    # The process noise Q

    # kf.Q


## Import xml file
filename = '../mujoco_model/whisker_v4_3_around_circle_r50_fixedBase.xml'
file = open(filename, mode='r') # 打开文件进行读取
xml = file.read() # 读取文件的内容
file.close() # 关闭文件


## Short-cut check
model = mujoco.MjModel.from_xml_string(xml)
data = mujoco.MjData(model)
renderer = mujoco.Renderer(model)
mujoco.mj_forward(model, data)
renderer.update_scene(data, camera='whisker_cam')
media.show_image(renderer.render())


## Simulate and display video.
duration = 10 # (seconds)
framerate = 30  # (Hz)

# Set testing speed 
total_speed = 0.2 # (m/s)
x_speed = 0.01 # (m/s)
z_step = 0.001 # just a factor
set_speed = 0.2 # (m/s)

WHISKER_LENGTH = 100

# Set the range of wanted angle
ANGLE_LOW = -1.6e-4   # -1.6e-4  correspond to Plane Height 5mm
ANGLE_SOLL = -2.8e-4    # -2.8e-4   correspond to Plane Height 15mm
ANGLE_HIGH = -3.8e-4    # -3.8e-4   correspond to Plane Height 25mm

BASE_DIR_BIAS = 0

angle_sub = 0.5e-4 # additional distance when at the end in the Height control


# Flags
contacted = 0  # set the contact flag
is_lowing = 0 # set the x_adjusting flag
is_lifting = 0 # set the x_adjusting flag

# Pre-Defines
theta_err_s = 0
theta_err_pre = 0
base_x_pre = 0.01
base_y_pre = -0.05
tan_count = 0
tan_count_a = 0
tan_count_t = 0
tip_pose_dist_ang_pre = np.pi/2
tip_pose_dist_ang = np.pi/2

angle_diff_sum = 0

pos_pre_arr_sensor = np.zeros(3)

stable_dis = 20  # Can't be too small, need some more data for the stablization

# 
DIR_BIAS = -0   # Define the forwarding direction bias



# PID parameters
A = 15 # 50
B = 0  # 0.1
C = 0  # 0.001

E = 500000   # 500000
F = 1000     # 800

E_c_limit = 20


# Template parameters
i = 0
j = 0
k = 0
ppp = 0



# Model Systems settings
kz_num = 1  # KongZhuan number, stable number, avoid collision impact
sample_ratio = 5
control_ratio = 5 # Can't be too small, need some time for the rotation to be kind of stablized




# Recordings
df = []
df1 = []
df1_1 = []
df2 = []
df3 = []
df4 = []
df5 = []
df6 = []
df7 = []
df8 = []
df9 = []
df_tip_x = []
df_tip_y = []
df_tip_x_f = []
df_tip_y_f = []
df10 = []
df10_1 = []
df11 = []
df11_1 = []

dff1 = []
dff2 = []
dff3 = []
dff4 = []
dff5 = []

t1 = []
t2 = []

######################
## Filters Settings ##
#####################
# Initialize the kalman filter

kf = KalmanFilter(dim_x=2, dim_z=2)
kf.x = np.array([-0.015, 0.098])   # 2d coordinates on X- and Y-axis, z was the position directly measured on two dimensions
kf.F = np.array([[1., 0.], [0., 1.]])
kf.H = np.array([[1., 0.], [0., 1.]])
kf.P *= 10.
kf.R = np.eye(kf.dim_z)
kf.Q = np.eye(kf.dim_x)*0.01

# Executing
window_size = 5

###########################



# Mujoco recordings settings
frames = [] # for rendering
mujoco.mj_resetData(model, data)  # Reset state and time


#####################################
## Move to a start point ##
#####################################

[theta] = data.sensor('whisker_joint_z').data.copy()
mujoco.mj_step(model, data)

# Step back for some time
data.ctrl[1] = -set_speed
data.ctrl[0] = 0

[whisker_joint_y] = data.sensor('whisker_joint_y').data.copy()
while whisker_joint_y > base_y_pre:
    for tt in range(1):
        mujoco.mj_step(model, data)
        # # render
        # if len(frames) < data.time * framerate:
        #     renderer.update_scene(data, camera='whisker_cam')
        #     pixels = renderer.render()
        #     frames.append(pixels) 
    [whisker_joint_y] = data.sensor('whisker_joint_y').data.copy()

# set the height
data.ctrl[0] = set_speed
data.ctrl[1] = 0

[whisker_joint_x] = data.sensor('whisker_joint_x').data.copy()
while whisker_joint_x < base_x_pre:
    for tt in range(1):
        mujoco.mj_step(model, data)
        # # render
        # if len(frames) < data.time * framerate:
        #     renderer.update_scene(data, camera='whisker_cam')
        #     pixels = renderer.render()
        #     frames.append(pixels) 
    [whisker_joint_x] = data.sensor('whisker_joint_x').data.copy()



# Start moving
data.ctrl[1] = total_speed * np.cos(theta)
data.ctrl[0] = - total_speed * np.sin(theta)


#####################################
## Actual Move Start Here ##
#####################################

time_start = data.time


while data.time < time_start + duration:
    [angle_now] = data.sensor('base2whisker_z').data.copy()

    if angle_now < -1e-4:
        contacted = 1

    ## Sample part
    if contacted and i % sample_ratio == 0: 
        ## Sensor part
        # Calculate the point of tip
        [angle_now] = data.sensor('base2whisker_z').data.copy()
        tipy = angle2fy(angle_now)
        tipx = angle2fx(angle_now)
        # [tipy, tipz, tipx] = data.sensor('tip_pos').data.copy()

        
        
        [base_x] = data.sensor('whisker_joint_x').data.copy()
        [base_y] = data.sensor('whisker_joint_y').data.copy()
        
        [theta] = data.sensor('whisker_joint_z').data.copy()
        

        # Change the tip postion to the world_coordinate
        pos_now_arr_sensor = np.dot(np.array([[np.cos(theta), -np.sin(theta), base_x], [np.sin(theta), np.cos(theta), base_y], [0,0,1]]), np.array([[tipx], [tipy], [1]]))

        df_tip_x.append(pos_now_arr_sensor[0])
        df_tip_y.append(pos_now_arr_sensor[1])

    


        # Linearisierung
        if j < stable_dis:
            tip_pose_dist_ang_org = np.array([0.5*np.pi]) 
            tip_pose_dist_ang = tip_pose_dist_ang_org

            x_filtered = tipx
            y_filtered = tipy

            df_tip_x_f.append(pos_now_arr_sensor[0])
            df_tip_y_f.append(pos_now_arr_sensor[1])
        else:
            # 0: Bayesian Filter
            measurements = [[np.array(df_tip_x)[i-window_size][0], np.array(df_tip_y)[i-window_size][0]] for i in range(window_size)]

            update_noise_matrices(kf, measurements)  # DYX: remember to deal with the edge members
            kf.predict()
            kf.update(measurements[-1])
            ux = kf.x.copy()

            x_filtered = ux[0]
            y_filtered = ux[1]

            df_tip_x_f.append(np.array([x_filtered]))
            df_tip_y_f.append(np.array([y_filtered]))
           

            # 1: Simple way
            tip_pose_dist_ang_org = np.arctan2(df_tip_y_f[-1] - df_tip_y_f[-stable_dis], df_tip_x_f[-1] - df_tip_x_f[-stable_dis])
            
            # 2: BSpline
            n_interior_knots = 5
            qs = np.linspace(0, 1, n_interior_knots+2)[1:-1]
            knots = np.quantile(np.array(df_tip_y_f)[-stable_dis:,0], qs)
            tck_q, u = interpolate.splprep([np.array(df_tip_x_f)[-stable_dis:,0], np.array(df_tip_y_f)[-stable_dis:,0]], t=knots, k=3)
            x_new_q, y_new_q = interpolate.splev(np.linspace(0, 1.3, 100), tck_q)
            x_next_q, y_next_q = interpolate.splev(1.05, tck_q)
            
            tip_pose_dist_ang_org = np.arctan2(y_next_q - df_tip_y_f[-1], x_next_q - df_tip_x_f[-1])
            
            ################################################
            # no change
            tip_pose_dist_ang = tip_pose_dist_ang_org

            # Wish a good start
            if j == stable_dis and tip_pose_dist_ang_org < 0:
                tip_pose_dist_ang_org += np.pi


            # Avoid the sudden change of the angle, in the range of (0, pi), value in range of (-pi, 0) should be transformed        
            if tip_pose_dist_ang_org - tip_pose_dist_ang_pre > 1 * np.pi and tip_pose_dist_ang_org - tip_pose_dist_ang_pre < 1.888 * np.pi: 
                tan_count_a = 1
                tip_pose_dist_ang_org = tip_pose_dist_ang_org - tan_count_a * np.pi
            if tip_pose_dist_ang_org - tip_pose_dist_ang_pre < - 1 * np.pi and tip_pose_dist_ang_org - tip_pose_dist_ang_pre > -1.888 * np.pi:
                tan_count_a = -1
                tip_pose_dist_ang_org = tip_pose_dist_ang_org - tan_count_a * np.pi
     
            # For the rotation after one turn - temperate methode       
            # Limit the _ang, avoid sudden change from -pi to pi             
            if tip_pose_dist_ang_org - tip_pose_dist_ang_pre > 1.88888 * np.pi: 
                tan_count += 1
                print("i: ", i)
                print(tan_count)
            if tip_pose_dist_ang_org - tip_pose_dist_ang_pre < - 1.88888 * np.pi: 
                tan_count += -1
                print("i: ", i)
                print(tan_count)

            tip_pose_dist_ang = tip_pose_dist_ang_org - tan_count * 2 * np.pi

            # Avoid the vibration of the angle (caused by the unstable slope estimation)
            if j > 2 * stable_dis:
                adj_speed_limit = 0.00025 * sample_ratio
                if tip_pose_dist_ang - df2[-1] > adj_speed_limit:
                    tip_pose_dist_ang = df2[-1] + adj_speed_limit
                elif tip_pose_dist_ang - df2[-1] < -adj_speed_limit:
                    tip_pose_dist_ang = df2[-1] - adj_speed_limit
        

        # Recording
        # df.append(theta_err)
        df1.append(base_y)
        df1_1.append(-base_x)
        df2.append(tip_pose_dist_ang)
        df3.append(theta)
        # df4.append(theta_err_s)
        # df5.append(theta_err_speed)
        df6.append(angle_now)
        # df7.append(tip_pose_dist_slope)
        df8.append(tip_pose_dist_ang_org)
        df9.append(tan_count_a)
        df10.append(tipx)
        df10_1.append(x_filtered)
        df11.append(tipy)
        df11_1.append(y_filtered)
        


        # Update
        # theta_err_pre = theta_err   
        tip_pose_dist_ang_pre = tip_pose_dist_ang_org
        j += 1



    ## Control part
    if contacted and i % control_ratio == 0:   
        if k >= 2*stable_dis:
            # Rotate
            data.ctrl[2] = (df2[-1] - 0.5 * np.pi)

            angle_now = df6[-1]
            angle_diff = angle_now - ANGLE_SOLL
            angle_diff_sum += angle_diff

            # E_c should be limited
            E_c = E * angle_diff + F * angle_diff_sum
            
            if E_c > E_c_limit:
                E_c = E_c_limit
            elif E_c < -E_c_limit:
                E_c = -E_c_limit
            # print(E_c)

            y_speed = np.sqrt(total_speed**2-(E_c * x_speed)**2)

            data.ctrl[1] = E_c * x_speed * np.sin(data.ctrl[2]) + y_speed * np.cos(data.ctrl[2])
            data.ctrl[0] = E_c * x_speed * np.cos(data.ctrl[2]) - y_speed * np.sin(data.ctrl[2])        
            # print(y_speed)
            dff1.append(angle_diff)
            dff2.append(angle_diff_sum)
            dff3.append(E_c * x_speed )
            dff4.append(y_speed )
            dff5.append(y_speed**2 + (E_c * x_speed )**2 )
        
        k += 1
        
    # Kongzhuan
    for ttt in range(kz_num):
        mujoco.mj_step(model, data)
        # # render
        # if len(frames) < data.time * framerate:
        #     renderer.update_scene(data, camera='whisker_cam')
        #     pixels = renderer.render()
        #     frames.append(pixels) 

    i += 1

    
    

    

    
# media.show_video(frames, fps=framerate)


x = np.linspace(1, len(df1), len(df1))
tx1 = np.linspace(1, len(t1), len(t1))
tx2 = np.linspace(1, len(t2), len(t2))

plt.figure(figsize=(10, 6))
plt.grid()
plt.plot(df_tip_y, np.array(df_tip_x) * -1,'o')
plt.title('tip_pos')

plt.figure(figsize=(10, 6))
plt.grid()
plt.plot(df1[:], df1_1[:],'o')
plt.title('Base_pos')


plt.figure(figsize=(10, 6))
plt.grid()
plt.plot(x[:], df2[:],'o')
plt.title('tip_pose_dist_ang')

plt.figure(figsize=(10, 6))
plt.grid()
plt.plot(x[:], df8[:],'o')
plt.title('angle_org')

# plt.figure(figsize=(10, 6))
# plt.grid()
# plt.plot(x[:], df3[:])
# plt.title('Base direction')

# plt.figure(figsize=(10, 6))
# plt.grid()
# plt.plot(x[:], df[:])
# plt.title('P part - Theta difference')

# plt.figure(figsize=(10, 6))
# plt.grid()
# plt.plot(x[:], df4[:])
# plt.title('I part')

# plt.figure(figsize=(10, 6))
# plt.grid()
# plt.plot(x[:], df5[:])
# plt.title('D part')

plt.figure(figsize=(10, 6))
plt.grid()
plt.plot(x[:], df6[:],'o')
plt.title('Whisker angle')

# plt.figure(figsize=(10, 6))
# plt.grid()
# plt.plot(x[:], df7[:],'o')
# plt.title('estimated slope')



plt.figure(figsize=(10, 6))
plt.grid()
plt.plot(x[:], df9[:])
plt.title('zeig')

# plt.figure(figsize=(10, 6))
# plt.grid()
# plt.plot(x[:], df10[:],'o')
# plt.title('tipx')

# plt.figure(figsize=(10, 6))
# plt.grid()
# plt.plot(x[:], df10_1[:],'o')
# plt.title('tipx_f')

# plt.figure(figsize=(10, 6))
# plt.grid()
# plt.plot(x[:], df11[:],'o')
# plt.title('tipy')

# plt.figure(figsize=(10, 6))
# plt.grid()
# plt.plot(x[:], df11_1[:],'o')
# plt.title('tipy_f')

# x = np.linspace(1, len(dff1), len(dff1))

# plt.figure(figsize=(10, 6))
# plt.grid()
# plt.plot(x[:], dff3[:])
# plt.title('vx')

# plt.figure(figsize=(10, 6))
# plt.grid()
# plt.plot(x[:], dff4[:])
# plt.title('vy')

# plt.figure(figsize=(10, 6))
# plt.grid()
# plt.plot(x[:], dff5[:])
# plt.title('vtotal')