lynxStart('Gripper','on','Frame','on');
addpath("quiver3D")
q = [0 0 0 0 0 0];
qdot = [1 0 0 0 0 0];
lynxVelocitySim(q,qdot);
view(54,24);