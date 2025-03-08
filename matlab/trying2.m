clear;
clc;
clf;

% Load the Kinova Gen3 6-DOF robot model
robot = importrobot("GEN3-6DOF_NO-VISION_URDF_ARM_V01.urdf", 'DataFormat', 'column');
numJoints = numel(homeConfiguration(robot));

%% Initial positions
tSpan = 0:0.01:1;
initialState = [homeConfiguration(robot); zeros(6,1)];


%% Define trajectory parameters
targetJointPosition = [0;0;0;1;1;1];

qDesPD  = [targetJointPosition; -1; pi/2; -pi/2; pi/4; 0; 0];

pdMotion = jointSpaceMotionModel("RigidBodyTree",robot,"MotionType","PDControl");
pdMotion.Kp = diag(300*ones(1,6));
pdMotion.Kd = diag(10*ones(1,6));

[tPD,yPD] = ode15s(@(t,y)derivative(pdMotion,y,qDesPD),tSpan,initialState);

% PD with Gravity Compensation
figure
subplot(2,1,1)
plot(tPD,yPD(:,1:numJoints))
hold on
plot(tPD,targetJointPosition*ones(1,length(tPD)),"--")
title("PD Controlled Joint Motion: Position")
xlabel("Time (s)")
ylabel("Position (rad)")
subplot(2,1,2)
plot(tPD,yPD(:,numJoints+1:end))
title("Joint Velocity")
xlabel("Time (s)")
ylabel("Velocity (rad/s)")