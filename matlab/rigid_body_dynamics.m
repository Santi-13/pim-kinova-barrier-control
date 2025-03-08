clear;
clc;
clf;

%% Load the Kinova Gen3 6-DOF robot model
kinova1 = importrobot("GEN3-6DOF_NO-VISION_URDF_ARM_V01.urdf", 'DataFormat', 'column');
eeName = "end_effector_link"; % End effector name
kinova1.Gravity = [0, 0, -9.81];

%% Initial positions
q_home = [0 15 -130 0 55 90]' * pi / 180; % Starting joint angles

%% Force control gain
Kp = 10; % (N/m)