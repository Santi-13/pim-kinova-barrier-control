clear;
clc;
clf;
%% Description
% Run this script to load the necessary variables for the
% rigid_body_dynamics.slx simulation. Only change Kp.

%% Load the Kinova Gen3 6-DOF robot model
kinova1 = importrobot("GEN3-6DOF_NO-VISION_URDF_ARM_V01.urdf", 'DataFormat', 'column');
eeName = "end_effector_link"; % End effector name
kinova1.Gravity = [0, 0, -9.81];

%% Initial positions
q_home = [0 15 -130 0 55 90]' * pi / 180; % Starting joint angles

%% Force control gain [x,y,z]
Kp = [10 10 10]; % (N/m)