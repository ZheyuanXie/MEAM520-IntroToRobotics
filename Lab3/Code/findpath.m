function path = findpath(map, start, goal)

% function for finding an obstacle free path through the provided map from
% the start location to the goal location
% 
% map   : a representation of the environment including obstacles used for
%         planning
%
% start : a 1x3 vector containing the coordinates (x,y,z) of the start
%         location
%
% goal  : a 1x3 vector containing the coordinates (x,y,z) of the goal
%         location
%
% path  : an Nx5 vector of joint variable values which constitute the path found
%         using your planner; Note: N is not known beforehand and will vary
%         depending how the path is constructed

% calculateFK_sol(q) takes in [q1,q2,q3,q4,q5]
% IK_lynx_sol(T0e) takes in gripper transformation matrix

%% Constant
lowerLim = [-1.4 -1.2 -1.8 -1.9 -2 -15];
upperLim = [1.4 1.4 1.7 1.7 1.5 30];
n_sample = 300;  % number of samples in c-space
k = 25;          % number of neighbours to connect

%% Sample in free c-space
f = waitbar(0,'');

disp('Sampling')
N = 0;                  % N = number of nodes
nodes_q = zeros(0,5);   % List of nodes in condiguration space (Nx5)
nodes_w = zeros(0,3);   % List of nodes in workspace (Nx3)
for i = 1:n_sample
    q_sample = random('uniform',lowerLim(1:end-1),upperLim(1:end-1));
    if ~checkcollision(q_sample, map)       % If sample is in free C-space, append to list
        N = N + 1;
        [~,T0e] = calculateFK_sol(q_sample);
        nodes_q(N,:)=q_sample;
        nodes_w(N,:)=T0e(1:3,4)';
    end
    waitbar(i/n_sample,f,'Sampling Free C-Space');
end
hold on
nodes_scatter = scatter3(nodes_w(:,1),nodes_w(:,2),nodes_w(:,3),'.');

%% Connecting Pairs of Configurations
G=graph;                    % Intialize an empty graph
G=addnode(G,N);             % Add N nodes to the graph
for index=1:N               % For each node q
    %dist = sum((nodes_w - nodes_w(index,:)).^2,2); % L2 distance in ws
    dist = max(abs(nodes_q - nodes_q(index,:)),2); % Linf distance in qs
    [~, id] = sort(dist);
    for dist_id=2:1+k       % For each of its k nearest neighbours q'
        index_=id(dist_id);
        q_interp = nodes_q(index,:) + (0:0.1:1)' * (nodes_q(index_,:) - nodes_q(index,:));
        path_collided = false;
        for i_interp=1:10   % For each of the linearly interploated steps
            q = q_interp(i_interp,:);
            collision = checkcollision(q,map);
            path_collided = path_collided || collision;
        end
        if ~path_collided   % Connect q and q' if no collision
%             pts = vertcat(nodes_w(index,:),nodes_w(index_,:));
%             l = line(pts(:,1),pts(:,2),pts(:,3));
            weight = norm(nodes_q(index,:)-nodes_q(index_,:),inf);
            G=addedge(G,index,index_,weight);
        end
    end
    waitbar(index/N,f,'Connecting Nodes');
end
G = simplify(G);            % Simplify and visualize
close(f);
delete(nodes_scatter);

%% Find q_start, q_a, q_b, q_goal
% q_start
p_start = start;
q_start = pose2q(p_start(1),p_start(2),p_start(3),0,0);

% q_goal
p_goal = goal;
q_goal = pose2q(p_goal(1),p_goal(2),p_goal(3),0,0);

% q_a (nearest node to q_start in c-space)
dists_start = max(abs(nodes_q - q_start),[],2);
[dist_start, qa_index] = min(dists_start);
q_a = nodes_q(qa_index,:);
p_a = nodes_w(qa_index,:);

% q_b (nearset node to q_goal in c-space)
dists_goal = max(abs(nodes_q - q_goal),[],2);
[dist_goal, qb_index] = min(dists_goal);
q_b = nodes_q(qb_index,:);
p_b = nodes_w(qb_index,:);


%% Find path from q_start to q_a
path = zeros(0,5);
nstep = round(dist_start * 20);
q_interp = q_start + (0:1/nstep:1)' * (q_a - q_start);
path = vertcat(path,q_interp);

pts = vertcat(start,p_a);
line(pts(:,1),pts(:,2),pts(:,3),'LineWidth',2,'Color','r');     % draw line

%% Find path from q_a to q_b
P = shortestpath(G,qa_index,qb_index,'Method','positive');
for i=1:length(P)-1
    index = P(i); index_ = P(i+1);
    dist = max(abs(nodes_q(index_,:) - nodes_q(index,:)),[],2);
    nstep = round(dist * 20);
    q_interp = nodes_q(index,:) + (0:1/nstep:1)' * (nodes_q(index_,:) - nodes_q(index,:));
    path = vertcat(path,q_interp);
    
    pts = vertcat(nodes_w(index,:),nodes_w(index_,:));
    line(pts(:,1),pts(:,2),pts(:,3),'LineWidth',2,'Color','r'); % draw line
end

%% Find path from q_b to q_goal
nstep = round(dist_goal * 20);
q_interp = q_b + (0:1/nstep:1)' * (q_goal - q_b);
path = vertcat(path,q_interp);

pts = vertcat(p_b,goal);
line(pts(:,1),pts(:,2),pts(:,3),'LineWidth',2,'Color','r');     % draw line

%% scatter end-effector positions
p=zeros(size(path,1),3);
for i=1:length(path)
    q=horzcat(path(i,:),0);
    [~, T0e]=calculateFK_sol(q);
    p(i,:) = T0e(1:3,4)';
end
scatter3(p(:,1),p(:,2),p(:,3),3,'g');
end
