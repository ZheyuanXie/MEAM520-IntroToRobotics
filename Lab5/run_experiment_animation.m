[a,b]=view();
addpath('lynxBase');
map = loadmap('map/map_4.txt');
% map = loadmap('example_map.txt');
lynxStart();plotmap(map);hold on;
p_start = [292.1,0,222.25];
p_final = [100,-200,300];
% p_start = [292.1,0,222.25];
% p_final = [-150,200,200];
q_start = pose2q(p_start(1),p_start(2),p_start(3),0,0);
q_final = pose2q(p_final(1),p_final(2),p_final(3),0,0);
tic 
[qlog,plog,fatt,frep] = potentialFieldPlanner(q_start,q_final,map,'att_model','conic');
toc
view(a,b)

% animation
quiver_scale = 10;
n_step = size(qlog,1);
hrep1 = quiver3(0,0,0,0,100,0,'LineWidth',3,'Color','r','ShowArrowHead','off');
hrep2 = quiver3(0,0,0,0,100,0,'LineWidth',3,'Color','r','ShowArrowHead','off');
hrep3 = quiver3(0,0,0,0,100,0,'LineWidth',3,'Color','r','ShowArrowHead','off');
hrep4 = quiver3(0,0,0,0,100,0,'LineWidth',3,'Color','r','ShowArrowHead','off');
hatt1 = quiver3(0,0,0,0,100,0,'LineWidth',3,'Color','g','ShowArrowHead','off');
hatt2 = quiver3(0,0,0,0,100,0,'LineWidth',3,'Color','g','ShowArrowHead','off');
hatt3 = quiver3(0,0,0,0,100,0,'LineWidth',3,'Color','g','ShowArrowHead','off');
hatt4 = quiver3(0,0,0,0,100,0,'LineWidth',3,'Color','g','ShowArrowHead','off');
for i=1:20:n_step
    q = qlog(i,:);
    [joint_pos, T0e] = calculateFK_sol(q);
    o1 = joint_pos(2,:);
    o2 = joint_pos(3,:);
    o3 = joint_pos(4,:);
    o4 = T0e(1:3,4)';
    Frep1 = frep(i,1,:);Frep2 = frep(i,2,:);Frep3 = frep(i,3,:);Frep4 = frep(i,4,:);
    Fatt1 = fatt(i,1,:);Fatt2 = fatt(i,2,:);Fatt3 = fatt(i,3,:);Fatt4 = fatt(i,4,:);
    set(hrep1,'xdata',o1(1),'ydata',o1(2),'zdata',o1(3),'udata',Frep1(1) * quiver_scale,'vdata',Frep1(2) * quiver_scale,'wdata',Frep1(3) * quiver_scale)
    set(hrep2,'xdata',o2(1),'ydata',o2(2),'zdata',o2(3),'udata',Frep2(1) * quiver_scale,'vdata',Frep2(2) * quiver_scale,'wdata',Frep2(3) * quiver_scale)
    set(hrep3,'xdata',o3(1),'ydata',o3(2),'zdata',o3(3),'udata',Frep3(1) * quiver_scale,'vdata',Frep3(2) * quiver_scale,'wdata',Frep3(3) * quiver_scale)
    set(hrep4,'xdata',o4(1),'ydata',o4(2),'zdata',o4(3),'udata',Frep4(1) * quiver_scale,'vdata',Frep4(2) * quiver_scale,'wdata',Frep4(3) * quiver_scale)
    set(hatt1,'xdata',o1(1),'ydata',o1(2),'zdata',o1(3),'udata',Fatt1(1) * quiver_scale,'vdata',Fatt1(2) * quiver_scale,'wdata',Fatt1(3) * quiver_scale)
    set(hatt2,'xdata',o2(1),'ydata',o2(2),'zdata',o2(3),'udata',Fatt2(1) * quiver_scale,'vdata',Fatt2(2) * quiver_scale,'wdata',Fatt2(3) * quiver_scale)
    set(hatt3,'xdata',o3(1),'ydata',o3(2),'zdata',o3(3),'udata',Fatt3(1) * quiver_scale,'vdata',Fatt3(2) * quiver_scale,'wdata',Fatt3(3) * quiver_scale)
    set(hatt4,'xdata',o4(1),'ydata',o4(2),'zdata',o4(3),'udata',Fatt4(1) * quiver_scale,'vdata',Fatt4(2) * quiver_scale,'wdata',Fatt4(3) * quiver_scale)
    plotLynx(q)
end
plot3(plog(:,1),plog(:,2),plog(:,3),'r','LineWidth',3);