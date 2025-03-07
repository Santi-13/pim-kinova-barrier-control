clear;
clc;
clf;

% Load the Kinova Gen3 6-DOF robot model
kinova1 = importrobot("GEN3-6DOF_NO-VISION_URDF_ARM_V01.urdf", 'DataFormat', 'column');
eeName = "end_effector_link"; % End effector name

%% Initial positions
q_home = [0 15 -130 0 55 90]' * pi / 180; % Starting joint angles
q_init = q_home;
disp('AA');

%% Define Waypoints for the Desired Trajectory
center = [0.5 0 0.4]; % [x y z]
radius = 0.1;
t_final = 10;
dt = 0.1; % Time step for simulation
t = (0:dt:t_final)';
theta = t * (2 * pi / t(end)) - (pi / 2);
points = center + radius * [0 * ones(size(theta)), cos(theta), sin(theta)];

%% Initialize Visualization
figure(1);
ax = show(kinova1, q_init);
hold on;
plot3(points(:,1), points(:,2), points(:,3), '-g', 'LineWidth', 2);
title('Simulated Movement of Kinova Gen3 Robot');
xlabel('x'); ylabel('y'); zlabel('z');
view([60, 10]);
grid minor;
axis auto equal;

%% Simulate sendJointSpeeds Command
kp = 0.5; % Proportional gain for velocity control

for i = 1:size(points, 1)
    % Get current end-effector pose
    T_current = getTransform(kinova1, q_init, eeName);
    pos_current = tform2trvec(T_current)';
    rot_current = tform2eul(T_current)';
    pose_current = [pos_current; rot_current];
    
    % Desired pose (position + orientation)
    pos_desired = points(i, :)';
    rot_desired = tform2eul(getTransform(kinova1, q_init, eeName))'; % Keep orientation fixed
    pose_desired = [pos_desired; rot_desired];
    
    % Task-space error (position only for simplicity)
    error = pose_desired - pose_current;
    error(4:6) = 0; % Ignore orientation error
    
    % Compute desired end-effector velocity
    desired_ee_velocity = kp * error;
    
    % Compute joint velocities using Jacobian pseudo-inverse
    J = geometricJacobian(kinova1, q_init, eeName);
    joint_velocities = transpose(J) * desired_ee_velocity;
    
    % Update joint positions (single integration step)
    q_init = q_init + joint_velocities * dt;
    
    % Visualize the robot
    show(kinova1, q_init, 'PreservePlot', false);
    drawnow;
    
    % Pause to simulate real-time behavior
    pause(dt);
end