function [qlog,plog,fattlog,freplog] = potentialFieldPlanner(q_start,q_final,map,varargin)

expectedModels = {'conic','parabolic','combined'};

p = inputParser;
addParameter(p,'zeta',0.1);
addParameter(p,'eta',1e6);
addParameter(p,'stepsize',0.002);
addParameter(p,'rho0',30);
addParameter(p,'att_model','conic',@(x) any(validatestring(x,expectedModels)));
parse(p,varargin{:});

zeta = p.Results.zeta;
eta = p.Results.eta;
stepsize = p.Results.stepsize;
rho0 = p.Results.rho0;
att_model = p.Results.att_model;
rw_enable = true;

% calculate final checkpoints
[joint_pos, T0e] = calculateFK_sol(q_final);
o1f = joint_pos(2,:);
o2f = joint_pos(3,:);
o3f = joint_pos(4,:);
o4f = T0e(1:3,4)';
p_final = o4f;

n_obstacle = size(map.obstacles,1);

q = q_start;
[joint_pos, T0e] = calculateFK_sol(q);
p = T0e(1:3,4)';
plog=p;qlog=q;fattlog=zeros(1,4,3);freplog=zeros(1,4,3);
while norm(p-p_final) > 14
    o1 = joint_pos(2,:);
    o2 = joint_pos(3,:);
    o3 = joint_pos(4,:);
    o4 = T0e(1:3,4)';

    % Calculate attractive force
    if strcmp(att_model,'conic')
        Fatt1 = [0;0;0];
        Fatt2 = -(o2-o2f)'/norm(o2-o2f);
        Fatt3 = -(o3-o3f)'/norm(o3-o3f);
        Fatt4 = -(o4-o4f)'/norm(o4-o4f);
    elseif strcmp(att_model,'combined')
        Fatt1 = -zeta*(o1-o1f)';
        Fatt2 = -zeta*(o2-o2f)';
        Fatt3 = -zeta*(o3-o3f)';
        Fatt4 = -zeta*(o4-o4f)';
    elseif strcmp(att_model,'parabolic')
        Fatt1 = -zeta*(o1-o1f)';
        Fatt2 = -zeta*(o2-o2f)';
        Fatt3 = -zeta*(o3-o3f)';
        Fatt4 = -zeta*(o4-o4f)';
    end
    
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
    % Log force
    fattlog(end+1,:,:)= vertcat(Fatt1',Fatt2',Fatt3',Fatt4');
    freplog(end+1,:,:)=vertcat(Frep1',Frep2',Frep3',Frep4');
    
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
    
    if checkcollision(q,map)
        break
    end
    
    % local minimum detection
    epsilonm = 0.7;
    if (size(plog,1) > 4 && norm(p-p_final) > 30)
        err1 = (norm(p-plog(end-1,:)));
        err2 = (norm(p-plog(end-2,:)));
        err3 = (norm(p-plog(end-3,:)));
        if ( err1 < epsilonm &&...
                err2 < epsilonm &&...
                err3 < epsilonm)
            if ~rw_enable
                break
            end
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
end
end

