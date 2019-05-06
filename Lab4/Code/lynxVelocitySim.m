function [quiver_l,quiver_a] = lynxVelocitySim(q,qdot)
lynxServo(q)
[~,T0e] = calculateFK_sol(q);
e_vel = FK_velocity(q, qdot);
quiver_l = quiver3D(T0e(1:3,4)',e_vel(1:3)'*0.5,[1 1 0]);
quiver_a = quiver3D(T0e(1:3,4)',e_vel(4:6)'*100,[0 1 1]);
camlight head;
lighting phong;
end

