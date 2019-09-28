clear all
close all

%%%%%%%%%%%%%  Robot Kinematics and Dynamics Model  %%%%%%%%%%%%%

robot = robotics.RigidBodyTree('DataFormat','row','MaxNumBodies',3);
% Define the length of the link
L1 = 0.2;
L2 = 0.2;
% Add link1 with joint1
link1 = robotics.RigidBody('link1');
joint1 = robotics.Joint('joint1', 'revolute');
setFixedTransform(joint1,trvec2tform([0 0 0]));
joint1.JointAxis = [0 0 1];
link1.Joint = joint1;
link1.Mass = 1.0;
link1.CenterOfMass = [0.1 0 0];
link1.Inertia = [0.01 0.01 0.01 0 0 0];
addBody(robot, link1, 'base');
% Add link2 with joint2
link2 = robotics.RigidBody('link2');
joint2 = robotics.Joint('joint2','revolute');
setFixedTransform(joint2, trvec2tform([L1,0,0]));
joint2.JointAxis = [0 0 1];
link2.Joint = joint2;
link2.Mass = 1.0;
link2.CenterOfMass = [0.1 0 0];
link2.Inertia = [0.01 0.01 0.01 0 0 0];
addBody(robot, link2, 'link1');
% Add link3 with joint3
endEffector = robotics.RigidBody('end effector');
joint3 = robotics.Joint('fix1','fixed');
setFixedTransform(joint3, trvec2tform([L2, 0, 0]));
endEffector.Joint = joint3;
endEffector.Mass = 0;
addBody(robot, endEffector, 'link2');

% show the robot
show(robot);
view(2)
ax = gca;
ax.Projection = 'orthographic';
%return


%%%%%%% Inverse Kinematics %%%%%%% 

dt = 0.4;
t = (0:dt:16)'; % Time
count = length(t);
% define the reference trajectory
center = [0.25 0.1 0];
radius = 0.1;
theta = t*(2*pi/t(end));
points_circle = center + radius*[cos(theta) sin(theta) zeros(size(theta))];

q0 = homeConfiguration(robot);
ndof = length(q0);
qs = zeros(count, ndof);

ik = robotics.InverseKinematics('RigidBodyTree', robot);
weights = [0, 0, 0, 1, 1, 0]; % only consider x and y
qInitial = q0; % Use home configuration as the initial guess
for i = 1:count
    % Solve for the configuration satisfying the desired end effector
    % position
    point = points_circle(i,:);
    qSol = ik('end effector', trvec2tform(point), weights, qInitial);
    % Store the configuration
    qs(i,:) = qSol;
    % Start from prior solution
    qInitial = qSol;
end

figure;
plot(points_circle(:,1), points_circle(:,2), 'b.'); % plot the reference trajectory

AnimateRobot(robot, qs)
return

%%%%%%%%%%%%%%%%%%%%% New Reference Trajectory --- Square %%%%%%%%%%%%%%%%%%%%%%%%%
dt = 0.1;
t = (0:dt:16)'; % Time
time_count = length(t)-1;
corner1 = [0.1 0.05 0];
corner2 = [0.1 0.25 0];
corner3 = [0.3 0.25 0];
corner4 = [0.3 0.05 0];
line1 = [linspace(corner1(1),corner2(1),time_count/4+1)' linspace(corner1(2),corner2(2),time_count/4+1)' linspace(corner1(3),corner2(3),time_count/4+1)'];
line2 = [linspace(corner2(1),corner3(1),time_count/4+1)' linspace(corner2(2),corner3(2),time_count/4+1)' linspace(corner2(3),corner3(3),time_count/4+1)'];
line3 = [linspace(corner3(1),corner4(1),time_count/4+1)' linspace(corner3(2),corner4(2),time_count/4+1)' linspace(corner3(3),corner4(3),time_count/4+1)'];
line4 = [linspace(corner4(1),corner1(1),time_count/4+1)' linspace(corner4(2),corner1(2),time_count/4+1)' linspace(corner4(3),corner1(3),time_count/4+1)'];
ref_pos_cartesian = [line1(1:end-1,:); line2(1:end-1,:); line3(1:end-1,:); line4(:,:)];
% plot the square
figure; 
%plot(ref_pos_cartesian(:,1), ref_pos_cartesian(:,2), 'b.');
axis equal
axis([-0.5 0.5 -0.5 0.5])
hold on
%return

% Inverse Kinematics
ik = robotics.InverseKinematics('RigidBodyTree', robot);
weights = [0, 0, 0, 1, 1, 0]; % only consider x and y
qInitial = q0; % Use home configuration as the initial guess
for i = 1:time_count+1
    % Solve for the configuration satisfying the desired end effector
    % position
    point = ref_pos_cartesian(i,:);
    qSol = ik('end effector', trvec2tform(point), weights, qInitial);
    % Store the configuration
    ref_pos(i,:) = qSol;
    % Start from prior solution
    qInitial = qSol;
end

ref_vel = diff(ref_pos)/dt;
ref_vel = [ref_vel;[0 0]];
ref_acc = diff(ref_vel)/dt;
ref_acc = [ref_acc; [0 0]];

% Compute the joint torques required for the robot given specified joint configuration, velocities, and accelerations with no external forces
pos = ref_pos(1,:);
vel = ref_vel(1,:);
acc = ref_acc(1,:);

% Feedback gains
Kp = 0.5;
Kd = 0.1;
prev_pos_error = [0,0];

for i = 1:time_count
    % Feedforward controller
    feedforwardTorque = inverseDynamics(robot, pos(i,:), vel(i,:), ref_acc(i,:));
    
    % Feedback controller
    pos_error = ref_pos(i,:) - pos(i,:);
    pos_error_dot = (pos_error - prev_pos_error)/dt;
    prev_pos_error = pos_error;
    feedbackTorque = Kp*pos_error + Kd*pos_error_dot;
    
    % Combined Feedforward and Feedback controller
    jointTorque = feedforwardTorque + feedbackTorque;
    
    % Add noise to the executed torque
   jointTorque = jointTorque + rand(1,2)*0.002;
    
    % simulate robot motion using forward dynamics
    jointAccel = forwardDynamics(robot, pos(i,:), vel(i,:), jointTorque);
    vel(i+1,:) = vel(i,:)+jointAccel*dt;
    pos(i+1,:) = pos(i,:)+vel(i,:)*dt;
end

% Plot the actual end-effector trajectory
% plot(pos(:,1), pos(:,2), 'ko')
AnimateRobot(robot, pos)


% %%%%%%%%%%%%%%%%%%%%%%%% Function of animating robots in 2D %%%%%%%%%%%%%%%%%%%%%%%% 
% % Take robot model and a sequence of joint positions as input, animate with
% % a frequency of 15 Hz
function AnimateRobot(robot, jointPos)
% figure
% view in 2D
view(2) % 2D
ax = gca;
ax.Projection = 'orthographic';
axis equal
axis([-0.5 0.5 -0.5 0.5])
hold on
% 
framesPerSecond = 15;
r = robotics.Rate(framesPerSecond);
for i = 1:length(jointPos)
    tform = getTransform(robot,jointPos(i,:),'end effector','base');
    plot(tform(1,4), tform(2,4), 'ko')
    show(robot,jointPos(i,:),'PreservePlot',false);
    drawnow

    waitfor(r);
end
end

