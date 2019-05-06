function [q, is_possible] = IK_lynx_sol(T0e)
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
%% Your code here

% Note that this code finds all four possible solutions.  This is not
% required...

% Assume the pose is reachable and check along the way.
is_possible = 1;
q = [0,0,0,0,0];

% Lynx ADL5 constants in mm
d1 = 76.2; % base height (table to center of joint 2)
a2 = 146.05; % shoulder to elbow length
a3 = 187.325; %elbow to wrist length
d5 = 76.2; %wrist to base of gripper
lg = 28.575; %length of gripper

% Wrist center
wc = T0e*[0;0;-(d5+lg);1];
wc_x = wc(1);
wc_y = wc(2);
wc_z = wc(3);

% Two possible solutions for joint 1 and joint 3 gives a total of 4
% solutions for the first three joints
q1([1,2],1) = atan2(wc_y,wc_x); %Note this is always possible physically and mathematically!
if(q1(1)>0)
    q1([3,4],1) = q1(1) - pi;
else
    q1([3,4],1) = q1(1) + pi;
end
q3(1,1) = -pi/2 - acos( (-a2^2-a3^2+wc_x^2+wc_y^2+(wc_z-d1)^2) / (2*a2*a3) );
% Note that this is only possible geometrically if the wrist center is not
% too close to or too far from the origin of frame 2 (the second joint).
% Mathematically, we see that this is only possible if the portion inside
% acos is between -1 and 1.  This gives us our first check!

if(abs(imag(q3)) > 1e-6)
    is_possible = 0;
    return;
else
    q3 = real(q3);
end

q3(2,1) = -pi-q3(1,1);

q3([3,4],1) = pi - q3([1,2],1); 

% Each pair of q1 and q3 has one valid q2
for(i=1:2)
   % q2 is the amounted needed to rotate (about z1) the position  of the wrist center
   % if q1 and q3 were set but q2 were zero (alpha), to the actual position of
   % the wrist center (beta)
   alpha = atan2(a3*sin(-pi/2-q3(i,1)),a2+a3*cos(-pi/2-q3(i,1)));
   beta = atan2(wc_z-d1,sqrt(wc_x^2+wc_y^2));
   q2(i,1) = pi/2+alpha - beta;
end

q2([3,4],1) = -q2([1,2],1);

for(i = 1:4)

    % Now plug the first three joint angles into FK
    R03 = [ cos(q2(i) + q3(i))*cos(q1(i)), -sin(q2(i) + q3(i))*cos(q1(i)), -sin(q1(i));
            cos(q2(i) + q3(i))*sin(q1(i)), -sin(q2(i) + q3(i))*sin(q1(i)),  cos(q1(i));
                      -sin(q2(i) + q3(i)),            -cos(q2(i) + q3(i)),          0];

    R05 = T0e(1:3,1:3);
    R35 = R03'*R05;

    % We can also solve R35 symbolically using forward kinematics to get:
    %R35 = [   x   x  c4 ;
    %          x   x  s4 ;
    %        -s5 -c5  0];

    % We can combine these two representations to get:
    q4(i,1) = atan2(R35(2,3),R35(1,3));
    q5(i,1) = atan2(-R35(3,1),-R35(3,2));
    
%     Note that it's always mathematically possible to find q4 and q5,
%     since atan2 always returns a value. However, our robot doesn't have
%     6 DOF, so we know that this doesn't always result in the correct
%     transformation matrix. You can easily see this visually, by trying to
%     place the end-effector at the zero pose, then have it point to the
%     left.

%     There are a few ways to do this final check, but one way is to check
%     if the z axis of the desired end-effector frame (or the vector from 
%     the wrist center to the end-effector origin) is in the same plane
%     as the vector from the origin to the wrist center and the z0 axis.

    tol = 1e-3; %Yeah, it's a random choice... 
    if(abs(dot(T0e(1:3,3),cross(wc(1:3),[0;0;1])))>tol)
        is_possible = 0;
        return;
    end

end

q = [q1,q2,q3,q4,q5];

% Only return one q vector (of 4)
%q = q(1,:);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

end