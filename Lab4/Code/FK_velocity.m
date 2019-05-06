function e_vel = FK_velocity(q, qdot)
% Input: q - 1 x 6 vector of joint inputs [q1,q2,q3,q4,q5,q6]
%        qdot - 1 x 6 vector of joint velocities [q1dot,q2dot,q3dot,q4dot,q5dot,q6dot]

% Outputs:  e_vel - 6 x 1 vector of end effector velocities, where
%                    e_vel(1:3) are the linear velocity
%                    e_vel(4:6) are the angular velocity

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Lynx ADL5 constants in mm
d1 = 76.2; % base height (table to center of joint 2)
a2 = 146.05; % shoulder to elbow length
a3 = 187.325; %elbow to wrist length
d5 = 76.2; %wrist to base of gripper
lg = 28.575; %length of gripper

%% Your code here
s1 = sin(q(1));
c1 = cos(q(1));
s2 = sin(q(2));
c2 = cos(q(2));
c23 = cos(q(2)+q(3));
s23 = sin(q(2)+q(3));
s234 = sin(q(2)+q(3)+q(4));
c234 = cos(q(2)+q(3)+q(4));
x1 = s2*a2+c23*a3+c234*d5;
x2 = c2*a2-s23*a3-s234*d5;
x3 = -s23*a3-s234*d5;
J = [-s1*x1 c1*x2 c1*x3 -c1*s234*d5 0;
      c1*x1 s1*x2 s1*x3 -s1*s234*d5 0;
      0 -s2*a3-c23*a3-c234*d5 -c23*a3-c234*d5 -c234*d5 0;
      0 -s1 -s1 -s1 c1*c234;
      0  c1  c1  c1 s1*c234;
      1  0   0   0    -s234];
e_vel = J*qdot(1:5)';
end