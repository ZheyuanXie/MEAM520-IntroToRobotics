function [jointPositions,T0e] = calculateFK_sol(q)
% Input: q - 1 x 6 vector of joint inputs [q1,q2,q3,q4,q5,lg]

% Outputs:  jointPositions - 6 x 3 matrix, where each row represents one 
%               joint along the robot. Each row contains the [x,y,z]
%               coordinates of the respective joint's center (mm). For
%               consistency, the first joint should be located at [0,0,0].
%               These values are used to plot the robot.
% 
%           T0e - a 4 x 4 homogeneous transformation matrix, representing 
%               the end effector frame expressed in the base (0) frame

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Your code here

% Lynx ADL5 constants in mm
d1 = 76.2; % base height (table to center of joint 2)
a2 = 146.05; % shoulder to elbow length
a3 = 187.325; %elbow to wrist length
d5 = 76.2; %wrist to base of gripper
lg = 28.575; %length of gripper

%Frame 1 w.r.t Frame 0
T1 = [cos(q(1)) -sin(q(1))*cos(-pi/2)  sin(q(1))*sin(-pi/2)  0;
      sin(q(1))  cos(q(1))*cos(-pi/2) -cos(q(1))*sin(-pi/2)  0;
              0            sin(-pi/2)            cos(-pi/2) d1;
              0                     0                  0     1];
          
%Frame 2 w.r.t Frame 1          
T2 = [cos(q(2)-(pi/2)) -sin(q(2)-(pi/2))  0   a2*cos(q(2)-(pi/2));
      sin(q(2)-(pi/2))  cos(q(2)-(pi/2))  0   a2*sin(q(2)-(pi/2));
              0                        0  1                     0;
              0                        0  0                     1];

%Frame 3 w.r.t Frame 2
T3 = [cos(q(3)+(pi/2)) -sin(q(3)+(pi/2))  0   a3*cos(q(3)+(pi/2));
      sin(q(3)+(pi/2))  cos(q(3)+(pi/2))  0   a3*sin(q(3)+(pi/2));
              0                        0  1                     0;
              0                        0  0                     1];

%Frame 4 w.r.t Frame 3
T4 = [cos(q(4)-(pi/2)) -sin(q(4)-(pi/2))*cos(-pi/2)   sin(q(4)-(pi/2))*sin(-pi/2)   0;
      sin(q(4)-(pi/2))  cos(q(4)-(pi/2))*cos(-pi/2)  -cos(q(4)-(pi/2))*sin(-pi/2)   0;
              0                          sin(-pi/2)                    cos(-pi/2)   0;
              0                                   0                             0   1];
%Frame 5 w.r.t Frame 4 
T5 = [cos(q(5)) -sin(q(5))  0        0;
      sin(q(5))  cos(q(5))  0        0;
              0          0  1       d5;
              0          0  0        1];
          
%Frame 5 w.r.t Frame 4 
T6 = [ 1  0  0   0;
       0  1  0   0;
       0  0  1  lg;
       0  0  0   1];

%Position of First Joint (Base Revolute)
X(1,:) = [0 0 0 1];

%Position of Second Joint (Shoulder Revolute)
X(2,:) = (T1*[0;0;0;1])';

%Position of Third Joint (Elbow Revolute)
X(3,:) = (T1*T2*[0;0;0;1])';

%Position of Fourth Joint (1st Wrist)
X(4,:) = (T1*T2*T3*[0;0;0;1])';

%Position of Fifth Joint (2nd Wrist)
X(5,:) = (T1*T2*T3*T4*[0;0;34;1])';

%Position of Gripper (Base of the Gripper)
X(6,:) = (T1*T2*T3*T4*T5*[0;0;0;1])';

%Outputs the 6x3 of the locations of each joint in the Base Frame
jointPositions = X(:,1:3);

T0e = T1*T2*T3*T4*T5*T6;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
end