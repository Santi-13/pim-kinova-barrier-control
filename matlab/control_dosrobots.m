clear;
clc;
clf;
kinova1 = importrobot("GEN3-6DOF_NO-VISION_URDF_ARM_V01.urdf",'DataFormat','column'); %Create the Kinova1 Model
kinova2 = importrobot("GEN3-6DOF_NO-VISION_URDF_ARM_V01.urdf",'DataFormat','column');

%% Displacing and orienting the 2nd robot base
shoulderLink = getBody(kinova2, 'shoulder_link');
T_translation = trvec2tform([0, 0.3, 0.16]); 
R_invert = axang2tform([9 .55 0 pi]); 
T_base = T_translation * R_invert; 
setFixedTransform(shoulderLink.Joint, T_base);

%% Initial positions
q_home = [0 15 -130 0 55 90]'*pi/180; %Starting Angles for Kinova1
eeName = "end_effector_link"; %End effector
T_home = getTransform(kinova1, q_home, eeName); %End-Effector Transformation

q1_home = [0 15 -130 0 55 90]'*pi/180; % Starting Angles for Kinova2
ee1Name = "end_effector_link"; %End effector
T_home_2 = getTransform(kinova2, q1_home, ee1Name);%End-Effector Transformation
 

%% Create Inverse Kinematics Solver and Set Parameters
% Robot 1
ik = inverseKinematics('RigidBodyTree', kinova1);
ik.SolverParameters.AllowRandomRestart = false;
weights = [1, 1, 1, 1, 1, 1];
q_init = q_home;

% Robot 2
ik2 = inverseKinematics('RigidBodyTree', kinova2);
ik2.SolverParameters.AllowRandomRestart = false;
q2_init = q1_home;

%% Define Waypoints for the Desired Trajectory for kinova 1
center = [0.5 0 0.4]; %[x y z]
radius = 0.1;
dt = 0.25;
t = (0:dt:10)';
theta = t*(2*pi/t(end))-(pi/2);
y_limit=0.01;
points = center + radius*[0*ones(size(theta)) cos(theta) sin(theta)];
for i=1:length(points)
    if points(i,2)>y_limit
        points(i,2)=y_limit;
    end
end

%% Define Waypoints from the Desired Trajectory for kinova 1
center2 = [0.5 0.3 0.4]; %[x y z]
y_limit2=0.3+(-0.01);
points2 = center2 + radius*[0*ones(size(theta)) cos(theta) sin(theta)];
for i=1:length(points2)
    if points2(i,2)<y_limit2
        points2(i,2)=y_limit2;
    end
end
points2 = flip(points2);

%% Generate Trajectories
% Kinova1
numJoints = size(q_home,1);   
numWaypoints = size(points,1);
qs = zeros(numWaypoints,numJoints);
%Solve the Inverse Kinematics for Each Waypoint
for i = 1:numWaypoints
    T_des = T_home;
    T_des(1:3,4) = points(i,:)';
    [q_sol, q_info] = ik(eeName, T_des, weights, q_init);
    % Store the configuration
    qs(i,:) = q_sol(1:numJoints);
    % Start from prior solution
    q_init = q_sol;
end

% Kinova2
numWaypoints2 = size(points2,1);
qs2 = zeros(numWaypoints2,numJoints);
%Solve the Inverse Kinematics for Each Waypoint
for i = 1:numWaypoints2
    T_des2 = T_home_2;
    T_des2(1:3,4) = points2(i,:)';
    [q2_sol, q2_info] = ik2(eeName, T_des2, weights, q2_init);
    % Store the configuration
    qs2(i,:) = q2_sol(1:numJoints);
    % Start from prior solution
    q2_init = q2_sol;
end
    


%Visualize the Animation of the Solution
figure(1); set(gcf,'Visible','on');
ax = show(kinova1,qs(1,:)');
ax.CameraPositionMode='auto';
hold on;
xlabel('x');
ylabel('y');
zlabel('z');
axis auto;
view([60,10]);
grid('minor');
axis equal
show(kinova1, q_home, 'Frames', 'off'); 
show(kinova2, q1_home, 'Frames', 'off'); 
 
% Plot waypoints
plot3(points(:,1),points(:,2),points(:,3),'-g','LineWidth',2);
plot3(points2(:,1),points2(:,2),points2(:,3),'-b','LineWidth',2); % Trayectoria robot 2

axis auto;
view([60,10]);
grid('minor');

hold on;
 
title('Simulated Movement of two Kinova Robots');
% Animate
framesPerSecond = 30;
r = robotics.Rate(framesPerSecond);
for i = 1:numWaypoints
    show(kinova1, qs(i,:)','PreservePlot',false);
    show(kinova2, qs2(i,:)','PreservePlot',false);
    drawnow;
    waitfor(r);
end
hold off

%% Send the Trajectory to the Hardware
prompt = 'Do you want to send same trajectory to the hardware? y/n [n]: ';
str = input(prompt,'s');

if isempty(str)
    str = 'n';
end
 
if str == 'n'
    clear;
    return;
end

%% Command Kinova Gen3 Robot to Track the Pre-Computed Trajectory
%Calculate joint velocity and acceleration at each waypoint using the numerical differentiation
% Robot 1
qs_deg = qs*180/pi;
vel = diff(qs_deg)/dt;
vel(1,:) = 0;
vel(end+1,:) = 0;
acc = diff(vel)/dt;
acc(1,:) = 0;
acc(end+1,:) = 0;
%Interpolate the joint position, velocity and acceleration to ensure the 0.001 seconds time step between two trajectory points
timestamp = 0:0.001:t(end);
qs_deg = interp1(t,qs_deg,timestamp);
vel = interp1(t,vel,timestamp);
acc = interp1(t,acc,timestamp);

% Robot 2
qs_deg2 = qs2*180/pi;
vel2 = diff(qs_deg2)/dt;
vel2(1,:) = 0;
vel2(end+1,:) = 0;
acc2 = diff(vel2)/dt;
acc2(1,:) = 0;
acc2(end+1,:) = 0;
%Interpolate the joint position, velocity and acceleration to ensure the 0.001 seconds time step between two trajectory points
timestamp2 = 0:0.001:t(end);
qs_deg2 = interp1(t,qs_deg2,timestamp2);
vel2 = interp1(t,vel2,timestamp2);
acc2 = interp1(t,acc2,timestamp2);

%% Conect to the robots
%%Create an API Instance to connect the robot 1
Simulink.importExternalCTypes(which('kortex_wrapper_data.h'));
gen3Kinova = kortex();
gen3Kinova.ip_address = '192.168.1.10';
gen3Kinova.user = 'admin';
gen3Kinova.password = 'admin';
gen3Kinova.nbrJointActuators = uint32(6); %Indicate that Kinova1 is 6DOF

%%Connect to the Robot 1
isOk = gen3Kinova.CreateRobotApisWrapper();
if isOk
    disp('You are connected to the Kinova1 !');
else
    error('Failed to establish a valid connection with Kinova1!');
end

%%Create an API Instance to connect the robot 2
Simulink.importExternalCTypes(which('kortex_wrapper_data.h'));
gen3Kinova2 = kortex();
gen3Kinova2.ip_address = '192.168.1.11';
gen3Kinova2.user = 'admin';
gen3Kinova2.password = 'admin';
gen3Kinova2.nbrJointActuators = uint32(6); %Indicate that Kinova1 is 6DOF

%%Connect to the Robot 2
isOk2 = gen3Kinova2.CreateRobotApisWrapper();
if isOk2
    disp('You are connected to the Kinova2 !');
else
    error('Failed to establish a valid connection with Kinova1!');
end

%Visualize the Actual Movement of the Robots
title('Actual Movement of the Robot');
[~,~, actuatorFb, ~] = gen3Kinova.SendRefreshFeedback();
[~,~, actuatorFb2, ~] = gen3Kinova2.SendRefreshFeedback();
hold on
show(kinova1, ((actuatorFb.position(1:6))*pi/180)','PreservePlot',false);
show(kinova2, ((actuatorFb2.position(1:6))*pi/180)','PreservePlot',false);
drawnow;

%% Starting position
%Send Robot 1 to Starting Point of the Trajectory
jointCmd = wrapTo360(qs_deg(1,:));
constraintType = int32(0);
speed = 0.1;
duration = 0;

%jointCmd = [0.232867239893077	38.6659016634925	-127.271338733165	0.240221381022901	75.9373577042250	89.9415522180413];
jointCmd3 = [0.263	68.332	280.572	0.235	74.28	89.92];
jointCmd2 = wrapTo360(qs_deg2(1,:));
isOk = gen3Kinova.SendJointAngles(jointCmd, constraintType, speed, duration);
isOk2 = gen3Kinova2.SendJointAngles(jointCmd2, constraintType, speed, duration);
pause(1)
if isOk && isOk2
    disp('success kinova 1 and kinova 2');
elseif isOk
    disp('success kinova 1, failure kinova 2') 
elseif isOk2
    disp('success kinova 2, failure kinova 2')
else
    disp('SendJointAngles cmd error');
    return;
end
%Check if the robot has reached the starting position
%% 
while 1
    [isOk,~, actuatorFb, ~] = gen3Kinova.SendRefreshFeedback();
    [isOk2,~, actuatorFb2, ~] = gen3Kinova2.SendRefreshFeedback();
    show(kinova1, ((actuatorFb.position(1:6))*pi/180)','PreservePlot',false);
    show(kinova2, ((actuatorFb2.position(1:6))*pi/180)','PreservePlot',false);
    drawnow;
    if isOk && isOk2
        if (max(abs(wrapTo360(qs_deg(1,:))-actuatorFb.position(1:6))) < 0.1)...
                && (max(abs(wrapTo360(qs_deg2(1,:))-actuatorFb2.position(1:6))) < 0.1)
            disp('Starting point reached by Kinova 1 and 2.')
            break;
        end 
    else
        error('SendRefreshFeedback error')
    end
end

%% Send Pre-Computed Trajectory
pause(1)
jointCmd3 = [jointCmd3 0]; 
speed3 = [1.2 20 20 20 20 20];
duration = 0;
%isOk = gen3Kinova.SendJointAngles(jointCmd3, int32(2), speed3, duration);
[result, tempHandle, ~] = kortexApiMexInterface('CreateRobotApisWrapper', '192.168.1.10', 'admin', 'admin', uint32(60000), uint32(20000));
status = kortexApiMexInterface("ReachJointAngles", tempHandle, 2, 20, 0, jointCmd3);
disp(status)
pause(1)
if isOk && isOk2
    disp('success kinova 1 and kinova 2');
elseif isOk
    disp('success kinova 1, failure kinova 2') 
elseif isOk2
    disp('success kinova 2, failure kinova 2')
else
    disp('SendJointAngles cmd error');
    return;
end
%Check if the robot has reached the starting position
%% 
while 1
    [isOk,~, actuatorFb, ~] = gen3Kinova.SendRefreshFeedback();
    [isOk2,~, actuatorFb2, ~] = gen3Kinova2.SendRefreshFeedback();
    show(kinova1, ((actuatorFb.position(1:6))*pi/180)','PreservePlot',false);
    show(kinova2, ((actuatorFb2.position(1:6))*pi/180)','PreservePlot',false);
    drawnow;
    if isOk && isOk2
        if (max(abs(wrapTo360(jointCmd3)-actuatorFb.position(1:6))) < 0.1)...
                && (max(abs(wrapTo360(qs_deg2(1,:))-actuatorFb2.position(1:6))) < 0.1)
            disp('Starting point reached by Kinova 1 and 2.')
            break;
        end 
    else
        error('SendRefreshFeedback error')
    end
end

% isOk1 = gen3Kinova.SendPreComputedTrajectory(qs_deg(1:end,:).', vel(1:end,:).', acc(1:end,:).', timestamp(:,1:end), size(timestamp(:,1:end),2));
% isOk2 = gen3Kinova2.SendPreComputedTrajectory(qs_deg2(1:end,:).', vel2(1:end,:).', acc2(1:end,:).', timestamp2(:,1:end), size(timestamp2(:,1:end),2));
% if isOk1 && isOk2
%     disp('SendPreComputedTrajectory success');
% else
%     disp('SendPreComputedTrajectory command error');
% end
% %Check if the robot has reached the end position
% pause(1)
% while 1
%     [isOk,~, actuatorFb, ~] = gen3Kinova.SendRefreshFeedback();
%     [isOk2,~, actuatorFb2, ~] = gen3Kinova2.SendRefreshFeedback();
%     show(kinova1, ((actuatorFb.position(1:6))*pi/180)','PreservePlot',false);
%     show(kinova2, ((actuatorFb2.position(1:6))*pi/180)','PreservePlot',false);
%     drawnow;
%     if isOk
%         if (max(abs(wrapTo360(qs_deg(end,:))-actuatorFb.position(1:6))) < 0.1) && ...
%                 (max(abs(wrapTo360(qs_deg2(end,:))-actuatorFb2.position(1:6))) < 0.1)
%             disp('End Point reached.')
%             break;
%         end 
%     else
%         error('SendRefreshFeedback error')
%     end
% end
% for i = 1:100:size(qs_deg,1)
%     isOk = gen3Kinova.SendJointAngles(qs_deg(i,:), constraintType, speed, duration);
%     isOk2 = gen3Kinova2.SendJointAngles(qs_deg2(i,:), constraintType, speed, duration);
%     pause(0.1)
%     if isOk && isOk2
%         %%disp('success kinova 1 and kinova 2');
%     elseif isOk
%         disp('success kinova 1, failure kinova 2') 
%     elseif isOk2
%         disp('success kinova 2, failure kinova 2')
%     else
%         disp('SendJointAngles cmd error');
%         return;
%     end
%     %Check if the robot has reached the starting position
%     %% 
%     while 1
%         [isOk,~, actuatorFb, ~] = gen3Kinova.SendRefreshFeedback();
%         [isOk2,~, actuatorFb2, ~] = gen3Kinova2.SendRefreshFeedback();
%         show(kinova1, ((actuatorFb.position(1:6))*pi/180)','PreservePlot',false);
%         show(kinova2, ((actuatorFb2.position(1:6))*pi/180)','PreservePlot',false);
%         hold on;
%         drawnow;
%         if isOk && isOk2
%             if (max(abs(wrapTo360(qs_deg(i,:))-actuatorFb.position(1:6))) < 0.1)...
%                     && (max(abs(wrapTo360(qs_deg2(i,:))-actuatorFb2.position(1:6))) < 0.1)
%                 %%disp('Starting point reached by Kinova 1 and 2.')
%                 break;
%             end 
%         else
%             error('SendRefreshFeedback error')
%         end
%     end
% end
%Disconnect from the Robot
isOk = gen3Kinova.DestroyRobotApisWrapper();
