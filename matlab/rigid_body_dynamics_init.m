clear;
clc;
clf;
%% Description
% Run this script to load the necessary variables for the
% rigid_body_dynamics.slx simulation. Only change Kp.

%% Pole Placement
% n = 6; % No. of joints
% A = [zeros(n,n) eye(n,n); zeros(n,n), zeros(n,n)];
% B = [zeros(n,n); eye(n,n)];
% Kp = eye(n,n);
% Kd = eye(n,n);
% P = -1 + zeros(2*n,1);
% 
% K = place(A,B,P)

% % Define the number of joints
% n = 6;
% 
% % Example desired poles (modify as needed)
% % Each column represents a joint's pair of poles
% poles = [
%     -1, -1;          % Joint 1 (real poles)
%     -1, -1;    % Joint 2 (complex conjugate)
%     -1, -1;          % Joint 3 (repeated real poles)
%     -1, -1;     % Joint 4 (complex conjugate)
%     -1, -1;           % Joint 5 (distinct real poles)
%     -1, -1;  % Joint 6 (complex conjugate)
%     ];
% 
% % Initialize Kp and Kd vectors
% Kp = zeros(1, n);
% Kd = zeros(1, n);
% 
% % Reshape into a 2x6 matrix (each column is a joint's poles)
% poles = poles.'; % Transpose to get 2 rows and 6 columns
% 
% % Compute Kp and Kd for each joint
% for i = 1:n
%     p1 = poles(1, i);
%     p2 = poles(2, i);
% 
%     % Check if poles are valid (real or complex conjugates)
%     if isreal(p1) && isreal(p2)
%         % Both are real, proceed
%     else
%         % Check if p2 is the conjugate of p1
%         if abs(p1 - conj(p2)) > 1e-6
%             error('Poles for joint %d are invalid. Ensure they are real or complex conjugates.', i);
%         end
%     end
% 
%     % Calculate gains for the current joint
%     Kp(i) = p1 * p2;          % Product of the poles
%     Kd(i) = - (p1 + p2);      % Negative sum of the poles
% end
% % Construct diagonal gain matrices
% K_P = diag(Kp);
% K_D = diag(Kd);
% 
% % Form the closed-loop matrix
% A_cl = [zeros(n), eye(n);
%         -K_P,      -K_D];
% 
% % Compute eigenvalues of the closed-loop matrix
% eigenvalues = eig(A_cl);
% 
% % Display results
% disp('Proportional gain matrix K_P:');
% disp(K_P);
% disp('Derivative gain matrix K_D:');
% disp(K_D);
% disp('Desired poles (column-wise):');
% disp(poles(:));
% disp('Computed eigenvalues:');
% disp(eigenvalues);

%% Initialization
% Load the Kinova Gen3 6-DOF robot model
kinova1 = importrobot("GEN3-6DOF_NO-VISION_URDF_ARM_V01.urdf", 'DataFormat', 'column');
eeName = "end_effector_link"; % End effector name
kinova1.Gravity = [0, 0, -9.81];

% Initial positions
q_home = [0 15 -130 0 55 90]' * pi / 180; % Starting joint angles

% Force control gain [x,y,z]
Kp = [20 20 10]; % (N/m)
Kd = [1 1 .5];

parfor xi = 1 : length(x)
  Jx(xi) = diff(f,x(xi));
end