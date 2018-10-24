function [q,is_possible] = IK_lynx_zyxie_lancelan(T0e)
% Input:    T - 4 x 4 homogeneous transformation matrix, representing 
%               the end effector frame expressed in the base (0) frame
%               (position in mm)

% Outputs:  q - a 1 x 5 vector of joint inputs [q1,q2,q3,q4,q5] (rad) which
%               are required for the Lynx robot to reach the given 
%               transformation matrix T
% 
%           is_possible - a boolean set to true if the provided
%               transformation T is achievable by the Lynx robot, ignoring
%               joint limits
% 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Lynx ADL5 constants in mm
d1 = 76.2; % base height (table to center of joint 2)
a2 = 146.05; % shoulder to elbow length
a3 = 187.325; %elbow to wrist length
d5 = 76.2; %wrist to base of gripper
lg = 28.575; %length of gripper

% Target pose
r11=T0e(1,1);r12=T0e(1,2);r13=T0e(1,3);
r21=T0e(2,1);r22=T0e(2,2);r23=T0e(2,3);
r31=T0e(3,1);r32=T0e(3,2);r33=T0e(3,3);
x=T0e(1,4);y=T0e(2,4);z=T0e(3,4);

% Wrist center position
xc=x-(d5+lg)*r13;
yc=y-(d5+lg)*r23;
zc=z-(d5+lg)*r33;

% Check possible
distc=sqrt(xc^2+yc^2+(zc-d1)^2);    % Wrist center position
z_diff = atan2(yc,xc) - atan2(r23,r13);    % Wrist orientation
if (distc > a2+a3) || (distc < a3-a2) || (abs(z_diff) > 0.1)
    is_possible = false;
    q=[0,0,0,0,0];
    return
end

% Inverse position
if xc==0 && yc==0   % Singularity
    q1=0;
else
    q1=atan2(yc,xc);
end
D=(a2^2+a3^2-xc^2-yc^2-(zc-d1)^2)/(2*a2*a3);
q3=atan2(D,sqrt(1-D^2));
q2=pi/2-atan2(zc-d1,sqrt(xc^2+yc^2))-atan2(a3*cos(q3),a2-a3*sin(q3));

% Inverse orientation
c1=cos(q1);s1=sin(q1);
c2=cos(q2);s2=sin(q2);
c3=cos(q3);s3=sin(q3);
c23=c2*c3-s2*s3;
s23=s2*c3+s3*c2;
s4=-c1*s23*r13-s1*s23*r23-c23*r33;
c4=c1*c23*r13+s1*c23*r23-s23*r33;
q4=atan2(s4,c4);
s5=s1*r11-c1*r21;
c5=s1*r12-c1*r22;
q5=atan2(s5,c5);

q=[q1 q2 q3 q4 q5];
is_possible=true;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
end