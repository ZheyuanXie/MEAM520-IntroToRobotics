[a,b]=view();
addpath('lynxBase');
map = loadmap('example_map.txt');
lynxStart('Frame','off','Gripper','on');plotmap(map);hold on;
p_start = [292.1,0,222.25];
p_final = [-150,200,200];
q_start = pose2q(p_start(1),p_start(2),p_start(3),0,0);
q_final = pose2q(p_final(1),p_final(2),p_final(3),0,0);

controlled_variable = 'eta';
value_table = [0,10e3,10e6,10e9];
% color_table = ['r','r','b','g','k','k'];
plog = cell(numel(value_table),1);

lineplot = [];
for i=1:numel(value_table)
    [~,plog{i},~,~] = potentialFieldPlanner(q_start,q_final,map,controlled_variable,value_table(i));
    fprintf("Eta:%d Number of Steps:%d\n",value_table(i),size(plog{i},1))
    lineplot(end+1) = plot3(plog{i}(:,1),plog{i}(:,2),plog{i}(:,3),'LineWidth',2);
end

legend(lineplot,{'0','10e3','10e6','10e9'})
plotLynx(q_final);
view(a,b)
axis([-200 400,-200,400,-200,500])
title('Trajectory with differnt \eta')