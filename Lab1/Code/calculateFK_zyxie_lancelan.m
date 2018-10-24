function [jointPositions,T0e] = calculateFK_zxie(q)
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
%% Dimension of Lynx
INCH2MM = 25.4;
L1 = 2.55 * INCH2MM;    % Joint1 to Joint2
L2 = 5.75 * INCH2MM;    % Joint2 to Joint3
L3 = 7.375 * INCH2MM;   % Joint3 to Joint4
L4 = 3.375 * INCH2MM;   % Joint4 to Joint6

%% DH parameters of Lynx
%       a   alpha   d       theta
dh = [  0   0       0       0           ;   % Link 1
        0   -pi/2   L1      q(1)        ;   % Link 2
        L2  0       0       -pi/2+q(2)  ;   % Link 3
        L3  0       0       pi/2+q(3)   ;   % Link 4
        0   pi/2    0       pi/2+q(4)   ;   % Link 5
        0   0       L4      pi+q(5)     ];  % Link 6

%% Compute Homogenous transformation of the kinematic chain 
Homo = cell(6,1);   % $T_i^{i-1}$
TF = cell(6,1);     % $T_i^0$
for i = 1:6
    a = dh(i,1); d = dh(i,3); alpha = dh(i,2); theta = dh(i,4);
    Homo{i}=[cos(theta) -sin(theta)*cos(alpha) sin(theta)*sin(alpha) a*cos(theta);
             sin(theta) cos(theta)*cos(alpha) -cos(theta)*sin(alpha) a*sin(theta)
             0 sin(alpha) cos(alpha) d;
             0 0 0 1];
    if i == 1
        TF{i} = Homo{1};
    else
        TF{i} = TF{i-1}*Homo{i};
    end
end

%% Assign outputs
T0e = TF{6};
jointPositions(1,:)=[0 0 0];
for i = 2:6
    jointPositions(i,:)=TF{i}(1:3,4)';
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
end