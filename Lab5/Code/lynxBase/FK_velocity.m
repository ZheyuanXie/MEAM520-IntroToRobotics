function e_vel = FK_velocity(q, qdot)
% Input: q - 1 x 6 vector of joint inputs [q1,q2,q3,q4,q5,q6]
%        qdot - 1 x 6 vector of joint velocities [q1dot,q2dot,q3dot,q4dot,q5dot,q6dot]

% Outputs:  e_vel - 6 x 1 vector of end effector velocities, where
%                    e_vel(1:3) are the linear velocity
%                    e_vel(4:6) are the angular velocity

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

J = calcJacobian(q);
e_vel = J * qdot(1:5)';

end
