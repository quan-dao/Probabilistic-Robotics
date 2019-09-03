[//]: # (Image References)
[ekf_local]: ./ekf-localization/images/EKF_localization.gif
[just_odometry]: ./ekf-localization/images/just_odometry.gif
[ukf_slam]: ./misc/ukf_slam_example.gif
[dlr_dataset]: ./least-square-slam/images/ls_slam_dlr_dataset.gif

# Intro

This repo contains my implmentation of 3 probabilistic robotic algorithm
* [Extended Kalman Filter (EKF) localization](./ekf-localization)
* [EKF SLAM](./ch10-ekf-slam)
* [Least-square SLAM](./least-square-slam)

The mathematical derivation of each algorithm (except EKF SLAM) is detailed in the corresponding folder's README. 

# EKF Localization

The comparison between EKF Localization and Odometry is shown below. In those figures, the grought truth and the estimated by either EKF or odometry is respectively denoted by the grey robot and the yellow robot. It can be seen that while the Odometry diverse from the ground truth after sometime, EKF Localization still manages to track the true state.

![alt text][just_odometry]

Fig.1 Odometry result 

![alt text][ekf_local]

Fig.2 EKF Localization result

# EKF SLAM

The result of using Unscented Kalman Filter SLAM in a simple simulated environment is shown below.

![alt text][ukf_slam]

Fig.3 UKF SLAM result

# Least-square SLAM

![alt text][dlr_dataset]

Fig.4 Applying LS-SLAM on DLR dataset
