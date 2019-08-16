more off;
clear 
close all;
clc

% add tools directory
addpath('tools');

% load the odometry measurements
load('odom_motions');

% the motions as they are estimated by scan-matching
load('scanmatched_motions');

% create our measurements vector z
z = [scanmatched_motions odom_motions];

% perform the calibration
X = ls_calibrate_odometry(z);
disp('calibration result'); disp(X);

% apply the estimated calibration parameters
calibrated_motions = apply_odometry_correction(X, odom_motions);

% compute the current odometry trajectory, the scanmatch result, and the calibrated odom
odom_trajectory = compute_trajectory(odom_motions);
scanmatch_trajectory = compute_trajectory(scanmatched_motions);
calibrated_trajectory = compute_trajectory(calibrated_motions);

% plot the trajectories
plot(...
  odom_trajectory(:,1), odom_trajectory(:,2), 'b-',...
  scanmatch_trajectory(:,1), scanmatch_trajectory(:,2), 'g-',...
  calibrated_trajectory(:,1), calibrated_trajectory(:,2), 'r-');
legend("Uncalibrated Odometry", "Scan-Matching", "Calibrated Odometry")
