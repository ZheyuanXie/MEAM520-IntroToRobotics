function [isCollided] = checkcollision(q,map)

n_obstacle = size(map.obstacles,1); % number of obstacles in the map
[jointPositions,T0e] = calculateFK_sol(q);
isCollided = false;

for ob_index = 1:n_obstacle
    for joint = 1:5
    link_collision = detectcollision(jointPositions(joint,:),jointPositions(joint+1,:),map.obstacles(ob_index,:));
    isCollided = isCollided || link_collision;
    end
    end_effector_collision = detectcollision(jointPositions(6,:),T0e(1:3,4)',map.obstacles(ob_index,:));
    isCollided = isCollided || end_effector_collision;
end

end

