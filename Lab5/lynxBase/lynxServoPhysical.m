function [] = lynxServoPhysical(th1, th2, th3, th4, th5, grip)
% Commands the real Lynx robot to the angles defined by the input (in radians and mm)
% Has the same limits as lynxServo.m.  

% INPUTS:
    % th1...th5 : Joint variables for the six DOF Lynx arm (rad)
    % grip = gripper width (mm)

global ttl robotName
q = [th1 th2 th3 th4 th5 grip]; % Position vector

%% Adjusting for out of range positions
lowerLim = [-1.4 -1.2 -1.8 -1.9 -2 -15]; % Lower joint limits in radians (grip in mm)
upperLim = [1.4 1.4 1.7 1.7 1.5 30]; % Upper joint limits in radians (grip in mm)
maxOmegas = [1 1 1 2 3 20]; %Rad/s and mm/s

% The guide for the controller states that 1000us corresponds 90degrees.
% If so, 636.62 converts from radians to microseconds. (1000/90*180/pi)
maxSpeedCommands = maxOmegas*636.62;

for i=1:length(q)
    if q(i) < lowerLim(i)
        q(i) = lowerLim(i);
        fprintf('Joint %d was sent below lower limit, moved to boundary %0.2f\n',i,lowerLim(i))
    elseif q(i) > upperLim(i)
        q(i) = upperLim(i);
        fprintf('Joint %d was sent above upper limit, moved to boundary %0.2f\n',i,upperLim(i))
    end
end

%% Serial Command Conversion
% Unfortunately, each robot has different zero positions for each joint.
% If your zero pose is off, this is where to make adjustments.  In my
% opinion you should zero the robot while holding it to simulate zero mass,
% but I suppose it's your call.
% Calibrations done by Gedaliah Knizhnik 9/18/18

if strcmpi(robotName, 'Legend')
    % Servo offsets
    servoOffsets = [1450, 1540, 1420, 1450, 1420, 2000];
    servoDirection = [-1,-1,1,-1,1,1];
elseif  strcmpi(robotName, 'Loopy')
    % Servo offsets
    servoOffsets = [1450, 1250, 1470, 1560, 1450, 2000];
    servoDirection = [-1,-1,1,-1,1,1];
elseif  strcmpi(robotName, 'Lucky')
    % Servo offsets
    servoOffsets = [1500, 1600, 1520, 1500, 1460, 2000];
    servoDirection = [-1,-1,1,-1,1,1];
elseif strcmpi(robotName, 'Lyric') 
    % Servo offsets
    servoOffsets = [1370, 1400, 1450, 1470, 1800, 2000];
    servoDirection = [-1,1,1,-1,1,1];
else
    error('Invalid robot name.')
end

% Conversion from joint angles to servo commands.  
P1 = servoDirection(1) * q(1) * (180/pi/0.102) + servoOffsets(1);
P2 = servoDirection(2) * q(2) * (180/pi/0.105) + servoOffsets(2);
P3 = servoDirection(3) * q(3) * (180/pi/0.109) + servoOffsets(3);
P4 = servoDirection(4) * q(4) * (180/pi/0.095) + servoOffsets(4);
P5 = servoDirection(5) * q(5) * (180/pi/0.102) + servoOffsets(5);
P6 = servoDirection(6) * q(6) / -0.028 + servoOffsets(6);

%% Sending commands to lynx
if(nargin == 6)
    fprintf(ttl, '%s\r', ...
        ['#1P' num2str(P1,'%.0f') ' S' num2str(maxSpeedCommands(1),'%.0f')  ...
        '#2P' num2str(P2,'%.0f') ' S' num2str(maxSpeedCommands(2),'%.0f') ...
        '#3P' num2str(P3,'%.0f') ' S' num2str(maxSpeedCommands(3),'%.0f') ...
        '#4P' num2str(P4,'%.0f') ' S' num2str(maxSpeedCommands(4),'%.0f') ...
        '#5P' num2str(P5,'%.0f') ' S' num2str(maxSpeedCommands(5),'%.0f') ...
        '#6P' num2str(P6,'%.0f') ' S' num2str(maxSpeedCommands(6),'%.0f')]);
end

end
