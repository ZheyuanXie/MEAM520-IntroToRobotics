lynxStart();
addpath("quiver3D")
q = [0 0 0 0 0 0];
qdot = [0.5 0.5 0.5 0.5 0.5 0];
% qdot = [-0.5 -0.5 -0.5 -0.5 -0.5 0];
lynxVelocitySimMove(q,qdot,20,0.1,'g');
plotLynx([0 0 0 0 0 0])