%Tutorial Sheet 3 Oli Thompson

%When running please don't close the animation until it has finished playing as it will
%corrupt the other figures.

%------------------------------------Question 1-----------------------------------
clear all;
syms theta1 theta2 theta3 theta4 theta5 theta6 theta7 %set joint angles as symbolic variables

% create a DH table: ai alphai di thetai
DH = [0         0       0.2703  theta1;...  %Joint 1
      0.069     -pi/2   0       theta2;...  %Joint 2
      0         pi/2    0.3644  theta3;...  %Joint 3
      0.069     -pi/2   0       theta4;...  %Joint 4
      0         pi/2    0.3743  theta5;...  %Joint 5
      0.01      -pi/2   0       theta6;...  %Joint 6
      0         pi/2    0.2295  theta7];    %Joint 7

 %DH table is separated into columns.
a = DH(:,1);
alpha = DH(:,2);
d = DH(:,3);
theta = DH(:,4);

TN = size(DH,1);
%Create Transformation matrix for each joint
for i = 1:TN
    T(i).A =  [cos(theta(i)) -sin(theta(i)) 0 a(i);...
               sin(theta(i))*cos(alpha(i)) cos(theta(i))*cos(alpha(i)) -sin(alpha(i)) -sin(alpha(i))*d(i);...
               sin(theta(i))*sin(alpha(i)) cos(theta(i))*sin(alpha(i)) cos(alpha(i)) cos(alpha(i))*d(i);...
               0 0 0 1];
end
%Construct the homogeneous transformation matrix by multiplying each individual transformation matrix together.
transformationmatrix = eye(size(T(1).A)); 
for i = 1:TN
    transformationmatrix = transformationmatrix*T(i).A;
end
disp('transformation Matrix:');%show transformation matrix
disp(transformationmatrix)
%---------------------Question 2---------------------------------------
%Ssubstitute joint angle values to find the tip position andorientation
transformationmatrix = simplify(transformationmatrix); %simplified transformation matrix from tip to base
tipPos = transformationmatrix(1:3,4) %tip position relative to the base
JacobianMatrix = jacobian(tipPos, [theta1, theta2, theta3, theta4, theta5, theta6, theta7]) %symbolic jacobian in terms of thetai

%---------------------Question 3---------------------------------------
% Set lower and upper bound [LB UB}
Boundaries = [-1.7016   1.7016; %Joint 1
              -2.147    1.047;  %Joint 2
              -3.0541   3.0541; %Joint 3
              -0.05     2.618;  %Joint 4
              -3.059    3.059;  %Joint 5
              -1.5707   2.094;  %Joint 6
              -3.059    3.059]; %Joint 7
%find initial Configuration using the given formula
initialangles = zeros(7, 1);
for i = 1:7
    initialangles(i) = Boundaries(i,1) + 0.5*(Boundaries(i,2) - Boundaries(i,1));
end

initialangles2 = [0; 0; -3.0541; 0; 0; 0; 0]; %these are alternative starting angles for question 5
initialangles3 = [0; 0; 0; 2.618; 0; 0; 0];

syms f(t); %declare cartesian position function to be symbolic
f(t) = [(0.05*cos(0.5*pi*t)); (0.05*sin(0.5*pi*t)); 0]; %cartesian position function
df = diff(f,t); %differentiate cartesion position function to get cartesian velocity

syms g(t); % this is a second cartesian position function (its velocity is commented out in the following for loop)
g(t) = [(16*(sin(t)^3)); ((13*cos(t)-(5*cos(2*t))-(2*cos(3*t)-cos(4*t)))); 0]; 
df = diff(g,t); %find the velocirt of the second cartesian function

Timestamp = 201; %Set the amount of time values 
JointangleMatrix = zeros(7, Timestamp); %initialise empty matrix of correct dimensions for the joint angles
newjointangles = initialangles3; %set the starting angles
j=1; %initialise index variable
dt = 0.02; %initialise time step, this value has been changed from 0.002 to improve the running time of the code. Feel free to change it back.

Endeffectorvel = zeros(3, Timestamp); %initialise empty end effector velocity array
pseudoinverseJacobianMatrix = zeros(7, 3, Timestamp); %initialise empty Pseudo inverse Jacobian matrix
Cartesianpositionmatrix = zeros (3, 201);%initialise empty Cartesian Position Matrix
JacobianMatrixMatrix = zeros(3, 7, Timestamp); %initialise empty Jacobian Matrix Matrix
tipPos = vpa(subs(transformationmatrix([1:3],4),[theta1,theta2,theta3,theta4,theta5,theta6,theta7],transpose(newjointangles)));
Cartesianposition = tipPos;%set initial endeffector position
for t = 0:dt:4
    NewJacobianMatrix = vpa(subs(JacobianMatrix,[theta1,theta2,theta3,theta4,theta5,theta6,theta7],transpose(newjointangles)));
    newjointangles = simplify(newjointangles + dt*(pinv(NewJacobianMatrix)*[-(pi*sin((pi*t)/2))/40; (pi*cos((pi*t)/2))/40; 0])); % this is an alternative trajectory: (0.05*[12*cos(t)*sin(t)^2; (5*sin(2*t))/2 + (3*sin(3*t))/2 - sin(4*t) - (13*sin(t))/4; 0])));
    JointangleMatrix(:,j) = newjointangles; %append each set of joint angles to a larger matrix
    Endeffectorvel(:,j) = (NewJacobianMatrix * (pinv(NewJacobianMatrix)*[-(pi*sin((pi*t)/2))/40; (pi*cos((pi*t)/2))/40; 0]));%[-(pi*sin((pi*t)/2))/40 (pi*cos((pi*t)/2))/40 0]; %append the velocity of the end effector to a matrix
    pseudoinverseJacobianMatrix(:,:,j)= pinv(NewJacobianMatrix); %append each pseudo inverted jacobian matrix to a larger matrix
    JacobianMatrixMatrix(:,:,j)= NewJacobianMatrix; %append each jacobian matrix to a larger matrix
    Cartesianposition = Cartesianposition + dt*(NewJacobianMatrix * (pinv(NewJacobianMatrix)*[-(pi*sin((pi*t)/2))/40; (pi*cos((pi*t)/2))/40; 0]));
    Cartesianpositionmatrix(:,j) = Cartesianposition; %append the cartesian positions to a larger matrix
    j=j+1; %increase index variable
end

%--------------------------Question 4--------------------------------------
%plot the changing joint anlges
figure;
T = 0:dt:4;
plot(T, JointangleMatrix);
title('Joint Angles Tracing a Circle Over 4 Seconds')
legend('S0','S1','E0','E1','W0','W1','W2');

%plot the changing end effector velocity values
figure;
plot(T, Endeffectorvel);
title('End Effector Velocities Tracing a Circle Over 4 Seconds')
legend('Vx','Vy','Vz');

%plot the end effector position
figure;
plot(Cartesianpositionmatrix(1,:),Cartesianpositionmatrix(2,:));
legend('Position');
axis square
title('Cartesian Position Calculated from Forward Kinematics');
xlabel('End effector X Coodinate') ;
ylabel('End effector Y Coodinate');

%plot the Changing jacobian matrix
figure;
r = reshape(JacobianMatrixMatrix,21, Timestamp);
plot(T,r);
title('Changing elements of the Jacobian Over 4 seconds')
legend;

%plot the changing pseudo inverse matrix
figure;
q = reshape(pseudoinverseJacobianMatrix,21,Timestamp);
plot(T,q);
title('Changing elements of the Psuedo Inverse of the Jacobian Over 4 seconds')
legend
figure;
%----------------------------Question 5------------------------------
%change the initial angles variable to initialangles2 or initialangles3 to
%see the changing effect in the joint plot and the animation.
%----------------------------Question 6------------------------------
%replace the cartesian trajectory of the cirle with the commented out
%trajectory next to it, this will change the plot into a heart shape.
newrobot = robotics.RigidBodyTree('DataFormat','row','MaxNumBodies',8); %create robot model for animation
% Add link1 with joint1
link1 = robotics.RigidBody('link1');
joint1 = robotics.Joint('joint1', 'revolute');
setFixedTransform(joint1,trvec2tform([0 0 0.273]));
joint1.JointAxis = [0 0 1];
link1.Joint = joint1;
addBody(newrobot, link1, 'base');
% Add link2 with joint2
link2 = robotics.RigidBody('link2');
joint2 = robotics.Joint('joint2','revolute');
setFixedTransform(joint2, trvec2tform([0.069,0,0]));
joint2.JointAxis = [0 1 0];
link2.Joint = joint2;
addBody(newrobot, link2, 'link1');
% Add link3 with joint3
link3 = robotics.RigidBody('link3');
joint3 = robotics.Joint('joint3','revolute');
setFixedTransform(joint3, trvec2tform([0,0,0.3644]));
joint3.JointAxis = [0 0 1];
link3.Joint = joint3;
addBody(newrobot, link3, 'link2');
% Add link4 with joint4
link4 = robotics.RigidBody('link4');
joint4 = robotics.Joint('joint4','revolute');
setFixedTransform(joint4, trvec2tform([0.069,0,0]));
joint4.JointAxis = [0 1 0];
link4.Joint = joint4;
addBody(newrobot, link4, 'link3');
% Add link5 with joint5
link5 = robotics.RigidBody('link5');
joint5 = robotics.Joint('joint5','revolute');
setFixedTransform(joint5, trvec2tform([0,0, 0.3743]));
joint5.JointAxis = [0 0 1];
link5.Joint = joint5;
addBody(newrobot, link5, 'link4');
% Add link6 with joint6
link6 = robotics.RigidBody('link6');
joint6 = robotics.Joint('joint6','revolute');
setFixedTransform(joint6, trvec2tform([0.01,0,0]));
joint6.JointAxis = [0 1 0];
link6.Joint = joint6;
addBody(newrobot, link6, 'link5');
% Add link7 with joint7
link7 = robotics.RigidBody('link7');
joint7 = robotics.Joint('joint7','revolute');
setFixedTransform(joint7, trvec2tform([0,0,0.2295]));
joint7.JointAxis = [0 0 1];
link7.Joint = joint7;
addBody(newrobot, link7, 'link6');
% Add link8 with joint7
endEffector = robotics.RigidBody('end effector');
joint8 = robotics.Joint('fix1','fixed');
setFixedTransform(joint8, trvec2tform([0, 0, 0]));
endEffector.Joint = joint8;
endEffector.Mass = 0;
addBody(newrobot, endEffector, 'link7');

% show the robot
show(newrobot);
view(2)
ax = gca;
ax.Projection = 'orthographic';
AnimateRobot(newrobot, transpose(JointangleMatrix));

function AnimateRobot(newrobot, jointPos)
view(3)
hold on
for i = 1:length(jointPos)
    tform = getTransform(newrobot,jointPos(i,:),'end effector','base');
    plot(tform(1,4), tform(2,4), 'ko')
    show(newrobot,jointPos(i,:),'PreservePlot',false);
    drawnow
end
end
