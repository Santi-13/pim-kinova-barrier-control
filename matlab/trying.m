clear;
clc;
clf;

% Load the Kinova Gen3 6-DOF robot model
kinova1 = importrobot("GEN3-6DOF_NO-VISION_URDF_ARM_V01.urdf", 'DataFormat', 'column');
eeName = "end_effector_link"; % End effector name
kinova1.Gravity = [0, 0, -9.81];

%% Initial positions
q_home = [0 15 -130 0 55 90]' * pi / 180; % Starting joint angles
q_init = q_home;
q_dot_init = zeros(6, 1); % Initial joint velocities

%% Define trajectory parameters
center = [0.5 0 0.4]; % [x y z]
radius = 0.1;
t_final = 10;
dt = 0.1; % Time step for simulation
t = (0:dt:t_final)';
theta = t * (2 * pi / t(end)) - (pi / 2);
points = center + radius * [0 * ones(size(theta)), cos(theta), sin(theta)];

%% Force control gain
Kp = 0.4; % (N/m)

%% Initialize visualization
figure(1);
ax = show(kinova1, q_init);
hold on;
plot3(points(:,1), points(:,2), points(:,3), '-g', 'LineWidth', 2);
title('Force Control for Kinova Gen3');
xlabel('x'); ylabel('y'); zlabel('z');
view([60, 10]);
grid minor;
axis auto equal;


for i = 1:size(points, 1)
    % Get current end-effector position
    T_current = getTransform(kinova1, q_init, eeName);
    pos_current = tform2trvec(T_current)';
    rot_current = tform2eul(T_current)';

    % Desired position
    pos_desired = points(i, :)';

    % Error calc
    error = [pos_desired - pos_current; zeros(3,1)];
    J = geometricJacobian(kinova1,q_init,eeName);
    
    % Get end-effector velocity
    ee_velocity = J * q_dot_init;

    % Get end-effector applied force (F = hc)
    hc = Kp * error;

    % Get joint torques    
    tau = J' * hc;

    % Compute joint accelerations (forward dynamics)
    M = massMatrix(kinova1, q_init);
    C = velocityProduct(kinova1, q_init, q_dot_init);
    G = gravityTorque(kinova1, q_init);
    
    q_ddot = M \ (tau - transpose(C) * q_dot_init - G);
    q_dot_init = C \ (tau - M * q_ddot - G);
    
    % Visualize
    show(kinova1, q_init, 'PreservePlot', false);
    drawnow;
    
    % Pause for real-time simulation
    pause(dt);
end