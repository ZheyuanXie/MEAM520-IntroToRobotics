lynxStart();    % Start in simulation mode
MODE='IK';      % IK - plot IK result, FK - plot FK result
INCH2MM=25.4;

% Target 1
T1_global = [-1 0 0 INCH2MM*-6.5;
             0 0 1 -450;
             0 1 0 INCH2MM*14.5;
             0 0 0 1];
T1_local = [-sind(60) cosd(60) 0 -7.62*INCH2MM*cosd(54.79)*cosd(60);
            cosd(60) sind(60) 0 -7.62*INCH2MM*cosd(54.79)*sind(60);
            0 0 -1 7.62*INCH2MM*sind(54.79);
            0 0 0 1;];
T1_rot = rotm2tform(rotx(-35.21));
T1=T1_global*T1_local*T1_rot;

% Target 2
T2_global = [0 -1 0 INCH2MM*-6.5;
             0 0 -1 450;
             1 0 0 INCH2MM*14.5;
             0 0 0 1];
T2_local = [1 0 0 0;
            0 -1 0 -11.13*INCH2MM*cosd(70.64);
            0 0 -1 11.13*INCH2MM*sind(70.64);
            0 0 0 1;];
T2_rot = rotm2tform(rotx(19.36));
T2=T2_global*T2_local*T2_rot;

% Target 3
T3_global = [0 -1 0 INCH2MM*9.5;
             -1 0 0 0;
             0 0 -1 INCH2MM*22.5;
             0 0 0 1];
T3_local = [0 1 0 0;
            1 0 0 0;
            0 0 -1 INCH2MM*4;
            0 0 0 1;];
T3=T3_global*T3_local;

% Target 4
T4_global = [0 1 0 INCH2MM*5.5;
             0 0 -1 450;
             -1 0 0 0;
             0 0 0 1];
T4_local = [0 1 0 0;
            1 0 0 0;
            0 0 -1 INCH2MM*7.46;
            0 0 0 1;];
T4=T4_global*T4_local;

% T5
T5_global = [0 -1 0 INCH2MM*5.5;
             -1 0 0 0;
             0 0 -1 INCH2MM*22.5;
             0 0 0 1];
T5_local = [0 1 0 0;
            1 0 0 0;
            0 0 -1 INCH2MM*4;
            0 0 0 1;];
T5=T5_global*T5_local;

q=[0,0,0,0,0,0];
[~,T0e]=calculateFK(q);
if strcmp(MODE,'FK')
%     lynxServoSim(q);
elseif strcmp(MODE,'IK')
    [qe,is_possible] = IK_lynx_zyxie_lancelan(T1);
    lynxServoSim([qe,0]);
%     lynxServoPhysical(qe(1),qe(2),qe(3),qe(4),qe(5),30);
end

% Draw Target 1
T1xAxis = T1(1:3,1)';
T1yAxis = T1(1:3,2)';
T1zAxis = T1(1:3,3)';
T1xAxis = 100*T1xAxis;
T1yAxis = 100*T1yAxis;
T1zAxis = 100*T1zAxis;
line([T1(1,4),T1(1,4)+T1xAxis(1)],[T1(2,4),T1(2,4)+T1xAxis(2)],[T1(3,4),T1(3,4)+T1xAxis(3)],'Color','r','LineWidth',1);
line([T1(1,4),T1(1,4)+T1yAxis(1)],[T1(2,4),T1(2,4)+T1yAxis(2)],[T1(3,4),T1(3,4)+T1yAxis(3)],'Color','g','LineWidth',1);
line([T1(1,4),T1(1,4)+T1zAxis(1)],[T1(2,4),T1(2,4)+T1zAxis(2)],[T1(3,4),T1(3,4)+T1zAxis(3)],'Color','b','LineWidth',1);
text(T1(1,4),T1(2,4),T1(3,4),'Target 1')

% Draw Target 2
T2xAxis = T2(1:3,1)';
T2yAxis = T2(1:3,2)';
T2zAxis = T2(1:3,3)';
T2xAxis = 100*T2xAxis;
T2yAxis = 100*T2yAxis;
T2zAxis = 100*T2zAxis;
line([T2(1,4),T2(1,4)+T2xAxis(1)],[T2(2,4),T2(2,4)+T2xAxis(2)],[T2(3,4),T2(3,4)+T2xAxis(3)],'Color','r','LineWidth',1);
line([T2(1,4),T2(1,4)+T2yAxis(1)],[T2(2,4),T2(2,4)+T2yAxis(2)],[T2(3,4),T2(3,4)+T2yAxis(3)],'Color','g','LineWidth',1);
line([T2(1,4),T2(1,4)+T2zAxis(1)],[T2(2,4),T2(2,4)+T2zAxis(2)],[T2(3,4),T2(3,4)+T2zAxis(3)],'Color','b','LineWidth',1);
text(T2(1,4),T2(2,4),T2(3,4),'Target 2')

% Draw Target 3
T3xAxis = T3(1:3,1)';
T3yAxis = T3(1:3,2)';
T3zAxis = T3(1:3,3)';
T3xAxis = 100*T3xAxis;
T3yAxis = 100*T3yAxis;
T3zAxis = 100*T3zAxis;
line([T3(1,4),T3(1,4)+T3xAxis(1)],[T3(2,4),T3(2,4)+T3xAxis(2)],[T3(3,4),T3(3,4)+T3xAxis(3)],'Color','r','LineWidth',1);
line([T3(1,4),T3(1,4)+T3yAxis(1)],[T3(2,4),T3(2,4)+T3yAxis(2)],[T3(3,4),T3(3,4)+T3yAxis(3)],'Color','g','LineWidth',1);
line([T3(1,4),T3(1,4)+T3zAxis(1)],[T3(2,4),T3(2,4)+T3zAxis(2)],[T3(3,4),T3(3,4)+T3zAxis(3)],'Color','b','LineWidth',1);
text(T3(1,4),T3(2,4),T3(3,4),'Target 3')

% Draw Target 4
T4xAxis = T4(1:3,1)';
T4yAxis = T4(1:3,2)';
T4zAxis = T4(1:3,3)';
T4xAxis = 100*T4xAxis;
T4yAxis = 100*T4yAxis;
T4zAxis = 100*T4zAxis;
line([T4(1,4),T4(1,4)+T4xAxis(1)],[T4(2,4),T4(2,4)+T4xAxis(2)],[T4(3,4),T4(3,4)+T4xAxis(3)],'Color','r','LineWidth',1);
line([T4(1,4),T4(1,4)+T4yAxis(1)],[T4(2,4),T4(2,4)+T4yAxis(2)],[T4(3,4),T4(3,4)+T4yAxis(3)],'Color','g','LineWidth',1);
line([T4(1,4),T4(1,4)+T4zAxis(1)],[T4(2,4),T4(2,4)+T4zAxis(2)],[T4(3,4),T4(3,4)+T4zAxis(3)],'Color','b','LineWidth',1);
text(T4(1,4),T4(2,4),T4(3,4),'Target 4')
axis([-450 450 -450 450 -200 600])

% Draw Target 5
T5xAxis = T5(1:3,1)';
T5yAxis = T5(1:3,2)';
T5zAxis = T5(1:3,3)';
T5xAxis = 100*T5xAxis;
T5yAxis = 100*T5yAxis;
T5zAxis = 100*T5zAxis;
line([T5(1,4),T5(1,4)+T5xAxis(1)],[T5(2,4),T5(2,4)+T5xAxis(2)],[T5(3,4),T5(3,4)+T5xAxis(3)],'Color','r','LineWidth',1);
line([T5(1,4),T5(1,4)+T5yAxis(1)],[T5(2,4),T5(2,4)+T5yAxis(2)],[T5(3,4),T5(3,4)+T5yAxis(3)],'Color','g','LineWidth',1);
line([T5(1,4),T5(1,4)+T5zAxis(1)],[T5(2,4),T5(2,4)+T5zAxis(2)],[T5(3,4),T5(3,4)+T5zAxis(3)],'Color','b','LineWidth',1);
text(T5(1,4),T5(2,4),T5(3,4),'Target 3 (NEW)')
axis([-450 450 -450 450 -200 600])