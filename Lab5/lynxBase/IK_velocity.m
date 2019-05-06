function qdot = IK_velocity(q, e_vel)

J = calcJacobian(q);

% check on e_vel;
idx_skip = (isnan(e_vel));
e_vel(idx_skip) = [];
J(idx_skip,:) = [];

qdot = J\e_vel(:);
qdot(6) = 0;

end
