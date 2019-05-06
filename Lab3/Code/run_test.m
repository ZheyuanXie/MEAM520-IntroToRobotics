% Define some start positions and end positions
sp1 = [39.39 13.54 511.3]; ep1 = [148.3 309.4 335.6];
sp2 = [268.4 -11.71 35.89]; ep2 = [-221.8 -24.13 352.4];
sp3 = [-141.7 -262.2 319.9]; ep3 = [218.9 329 7.483];
sp4 = [299.4 107.3 179.2]; ep4 = [275.4 14.05 335.4];
sp5 = [269.6 -256 275.1]; ep5 = [365 -62.36 305.9];
sp6 = [292.1,0,222.25]; ep6 = [-150,200,200];

% Create an simulation
addpath("lynxBase");
lynxStart('Frame','off');
lynxServoSim([0 0 0 0 0 0]);
map = loadmap('maps/map_7.txt');
plotmap(map);

path = findpath(map,sp6,ep6);

for i=1:length(path)
    q=horzcat(path(i,:),0);
    lynxServoSim(q);
    frame = getframe(gcf);
    pause(0.01)
end
