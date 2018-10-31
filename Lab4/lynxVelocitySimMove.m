function [] = lynxVelocitySimMove(q,qdot,T,ts,color)
% Commands the simulated Lynx robot to the angular velocities defined by inputs (in rad/s)
t = 0:ts:T;
qlog = zeros(numel(t),6);
xlog = zeros(numel(t));
ylog = zeros(numel(t));
zlog = zeros(numel(t));

for i = 1:numel(t)
    q(1)=q(1)+qdot(1)*ts;
    q(2)=q(2)+qdot(2)*ts;
    q(3)=q(3)+qdot(3)*ts;
    q(4)=q(4)+qdot(4)*ts;
    q(5)=q(5)+qdot(5)*ts;
    qlog(i,:)=q;
    [~,T0e] = calculateFK_sol(q);
    xlog(i)=T0e(1,4);
    ylog(i)=T0e(2,4);
    zlog(i)=T0e(3,4);
end

plot3(xlog,ylog,zlog,'LineWidth',1.5,'Color',color);

for i = 1:numel(t)
    plotLynx(qlog(i,:));
    pause(ts);
end

end