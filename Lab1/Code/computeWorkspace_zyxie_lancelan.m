% Place your code for computing the workspace of the Lynx robot here.
n_sample = 10000;
lowerLim = [-1.4 -1.2 -1.8 -1.9 -2 -15]; % Lower joint limits in radians (grip in mm)
upperLim = [1.4 1.4 1.7 1.7 1.5 30]; % Upper joint limits in radians (grip in mm)
sample = zeros(n_sample,3);
% randomly sample n_sample=10000 times in the configuration space
for i=1:n_sample
    q = random('Uniform',lowerLim,upperLim,1,6);
    [jointPositions, T0e] = calculateFK_zyxie_lancelan(q);
    sample(i,:)=jointPositions(6,:);
end
scatter3(sample(:,1),sample(:,2),sample(:,3),1,[0.8,0,0]); % scatter plot the result
axis equal