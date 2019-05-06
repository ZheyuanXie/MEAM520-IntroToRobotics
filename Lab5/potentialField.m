% constant
zeta = 0.1;
eta = 1e6;
stepsize = 0.002;
rho0 = 30;
quiver_scale = 10;
rw_enable = false;

% MATLAB include path
addpath('lynxBase');

filename='44.png';
p_start = [292.1,0,222.25];p_final = [-150,200,350];
p_start = [292.1,0,222.25];p_final = [-150,-200,350];
% % 
p_start = [292.1,0,222.25];p_final = [100,200,300];
p_start = [292.1,0,222.25];p_final = [100,-200,300];
% % 
% p_start = [292.1,0,222.25];p_final = [200,100,300];
% p_start = [292.1,0,222.25];p_final = [0,-250,300];

p_start = [292.1,0,222.25];p_final = [280,-200,222.25];

% lynx init
lynxStart('Frame','off','Gripper','on');
q_start = pose2q(p_start(1),p_start(2),p_start(3),0,0);
q_final = pose2q(p_final(1),p_final(2),p_final(3),0,0);
% q_final = pose2q(200,-200,300,0,0);
plotLynx(q_start);
% view(-60,20)
view(-80,75);view(-50,46)

% load map
map = loadmap('map/map_2.txt');
n_obstacle = size(map.obstacles,1);
plotmap(map); hold on
pause(0.5)

% plot arrows
hrep1 = quiver3(0,0,0,0,100,0,'LineWidth',3,'Color','r','ShowArrowHead','off');
hrep2 = quiver3(0,0,0,0,100,0,'LineWidth',3,'Color','r','ShowArrowHead','off');
hrep3 = quiver3(0,0,0,0,100,0,'LineWidth',3,'Color','r','ShowArrowHead','off');
hrep4 = quiver3(0,0,0,0,100,0,'LineWidth',3,'Color','r','ShowArrowHead','off');
hatt1 = quiver3(0,0,0,0,100,0,'LineWidth',3,'Color','g','ShowArrowHead','off');
hatt2 = quiver3(0,0,0,0,100,0,'LineWidth',3,'Color','g','ShowArrowHead','off');
hatt3 = quiver3(0,0,0,0,100,0,'LineWidth',3,'Color','g','ShowArrowHead','off');
hatt4 = quiver3(0,0,0,0,100,0,'LineWidth',3,'Color','g','ShowArrowHead','off');

[joint_pos, T0e] = calculateFK_sol(q_final);
o1f = joint_pos(2,:);
o2f = joint_pos(3,:);
o3f = joint_pos(4,:);
o4f = T0e(1:3,4)';
q = q_start;
step_count = 1;
[joint_pos, T0e] = calculateFK_sol(q);
p = T0e(1:3,4)';
plog=[];qlog=[];
while norm(p-p_final) > 14
    o1 = joint_pos(2,:);
    o2 = joint_pos(3,:);
    o3 = joint_pos(4,:);
    o4 = T0e(1:3,4)';

    % Calculate attractive force
    Fatt1 = -zeta*(o1-o1f)';
    Fatt2 = -zeta*(o2-o2f)';
    Fatt3 = -zeta*(o3-o3f)';
    Fatt4 = -zeta*(o4-o4f)';
    
    % Calculate repulsive force
    Frep1 = [0,0,0]';
    Frep2 = [0,0,0]';
    Frep3 = [0,0,0]';
    Frep4 = [0,0,0]';
    rhoo1_min = rho0;
    rhoo2_min = rho0;
    rhoo3_min = rho0;
    rhoo4_min = rho0;
    for ob_idx=1:n_obstacle
        box = map.obstacles(ob_idx,:);
        box_center = box(1:3)+box(4:6)/2;
        [rhoo1, point1] = distPointToBox(o1,box);
        [rhoo2, point2] = distPointToBox(o2,box);
        [rhoo3, point3] = distPointToBox(o3,box);
        [rhoo4, point4] = distPointToBox(o4,box);
        if rhoo1 < rhoo1_min
            Frep1 = Frep1+eta*(1/rhoo1 - 1/rho0)*(1/rhoo1^2)*(o1-point1)'/norm(o1-point1);
            rhoo1_min = rhoo1;
        end
        if rhoo2 < rhoo2_min
            Frep2 = Frep2+eta*(1/rhoo2 - 1/rho0)*(1/rhoo2^2)*(o2-point2)'/norm(o2-point2);
            rhoo2_min = rhoo2;
        end
        if rhoo3 < rhoo3_min
            Frep3 = Frep3+eta*(1/rhoo3 - 1/rho0)*(1/rhoo3^2)*(o3-point3)'/norm(o3-point3);
            rhoo3_min = rhoo3;
        end
        if rhoo4 < rhoo4_min
            Frep4 = Frep4+eta*(1/rhoo4 - 1/rho0)*(1/rhoo4^2)*(o4-point4)'/norm(o4-point4);
            rhoo4_min = rhoo4;
        end
    end
    
    % Calculate total force
    F1=Frep1+Fatt1;F2=Frep2+Fatt2;F3=Frep3+Fatt3;F4=Frep4+Fatt4;
    
    % Convert force to joint torque
    Jv1 = calcJv(q,2);
    Jv2 = calcJv(q,3);
    Jv3 = calcJv(q,4);
    Jv4 = calcJv(q,6);
    torque1 = vertcat(Jv1'*F1,[0;0;0;0]);
    torque2 = vertcat(Jv2'*F2,[0;0;0]);
    torque3 = vertcat(Jv3'*F3,[0;0]);
    torque4 = vertcat(Jv4'*F4);
    torque = torque1+torque2+torque3+torque4;
    torque = torque/norm(torque);
    
    
    % update q
    q = q + stepsize*horzcat(torque',0);
    [joint_pos, T0e] = calculateFK_sol(q);
    p = T0e(1:3,4)';
    qlog(end+1,:) = q;
    plog(end+1,:) = p;
    error = norm(p-p_final)
    
    % local minimum detection
    epsilonm = 0.7;
    if (size(plog,1) > 4 && norm(p-p_final) > 30) && rw_enable
        err1 = (norm(p-plog(end-1,:)));
        err2 = (norm(p-plog(end-2,:)));
        err3 = (norm(p-plog(end-3,:)));
        if ( err1 < epsilonm &&...
                err2 < epsilonm &&...
                err3 < epsilonm)
            localmin = q;
            qsign = [-1,1];
            is_collided = true;
            while is_collided
                qs = q + 0.1*[1,1,1,1,0,0].*horzcat(qsign(randi(2,1,5)),0);
                is_collided = checkcollision(qs,map);
                if (~is_collided)
                    q = qs;
                end
            end
        end
    end
    
    % update plot
    if mod(size(qlog,1),50)==0
        plotLynx(q);
        set(hrep1,'xdata',o1(1),'ydata',o1(2),'zdata',o1(3),'udata',Frep1(1) * quiver_scale,'vdata',Frep1(2) * quiver_scale,'wdata',Frep1(3) * quiver_scale)
        set(hrep2,'xdata',o2(1),'ydata',o2(2),'zdata',o2(3),'udata',Frep2(1) * quiver_scale,'vdata',Frep2(2) * quiver_scale,'wdata',Frep2(3) * quiver_scale)
        set(hrep3,'xdata',o3(1),'ydata',o3(2),'zdata',o3(3),'udata',Frep3(1) * quiver_scale,'vdata',Frep3(2) * quiver_scale,'wdata',Frep3(3) * quiver_scale)
        set(hrep4,'xdata',o4(1),'ydata',o4(2),'zdata',o4(3),'udata',Frep4(1) * quiver_scale,'vdata',Frep4(2) * quiver_scale,'wdata',Frep4(3) * quiver_scale)
        set(hatt1,'xdata',o1(1),'ydata',o1(2),'zdata',o1(3),'udata',Fatt1(1) * quiver_scale,'vdata',Fatt1(2) * quiver_scale,'wdata',Fatt1(3) * quiver_scale)
        set(hatt2,'xdata',o2(1),'ydata',o2(2),'zdata',o2(3),'udata',Fatt2(1) * quiver_scale,'vdata',Fatt2(2) * quiver_scale,'wdata',Fatt2(3) * quiver_scale)
        set(hatt3,'xdata',o3(1),'ydata',o3(2),'zdata',o3(3),'udata',Fatt3(1) * quiver_scale,'vdata',Fatt3(2) * quiver_scale,'wdata',Fatt3(3) * quiver_scale)
        set(hatt4,'xdata',o4(1),'ydata',o4(2),'zdata',o4(3),'udata',Fatt4(1) * quiver_scale,'vdata',Fatt4(2) * quiver_scale,'wdata',Fatt4(3) * quiver_scale)
    end
end

plot3(plog(:,1),plog(:,2),plog(:,3),'r','LineWidth',1)
delete([hrep1,hrep2,hrep3,hrep4,hatt1,hatt2,hatt3,hatt4])
axis([-200,400,-400,400,-200,600])
% saveas(gcf,filename)