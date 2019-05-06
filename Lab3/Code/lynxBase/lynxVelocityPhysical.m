function [] = lynxVelocityPhysical(om1, om2, om3, om4, om5, gripV)
% Commands the real Lynx robot to the angular velocities defined by inputs (in rad/s and mm/s)
% Has the same limits as lynxServo.m

% NOTE: The Lynx robots are not very good at this.  If left to its own
% devices, each robot would try to continue moving beyond its physical
% limits. To handle this, we actually send each joint to either its max or
% min position at a given velocity.  Once the joint reaches its max or min,
% it stops.

% INPUTS:
%   om1...om6 : Joint variables for the six DOF Lynx arm (th6 = grip)

global ttl robotName
qdot = [om1 om2 om3 om4 om5 gripV]; % Position vector

%% Adjusting for out of range positions
lowerLim = [-1.4 -1.2 -1.8 -1.9 -2 -15]; % Lower joint limits in radians (grip in mm)
upperLim = [1.4 1.4 1.7 1.7 1.5 30]; % Upper joint limits in radians (grip in mm)
maxOmegas = [2 2 2 5 5 100]; 

% Values below this are set to zero
smallestVels = pi/30*ones(1,6);


for i=1:length(qdot)
    if qdot(i) < -maxOmegas(i)
        qdot(i) = -maxOmegas(i);
        fprintf('Joint %d was sent below velocity limit, changed to -%0.2f\n',i,maxOmegas(i))
    elseif qdot(i) > maxOmegas(i)
        qdot(i) = maxOmegas(i);
        fprintf('Joint %d was sent above velocity limit, changed to %0.2f\n',i,maxOmegas(i))
    end
end

%% Serial Command Conversion

% Find which robot is being controled, and adjust offsets accordingly
if strcmpi(robotName, 'Legend')
    % Servo offsets
    servoOffsets = [1380, 1480, 1500, 1450, 1420, 1900];
elseif  strcmpi(robotName, 'Lucky')
    % Servo offsets
    servoOffsets = [1550, 1500, 1470, 1560, 1460, 2000];
elseif strcmpi(robotName, 'Lyric')
    % Servo offsets
    servoOffsets = [1370, 1460, 1500, 1450, 1640, 2000];
else
    error('Invalid robot name.')
end

ratios = [0.102,0.105,0.109,0.095,0.102];

%Calculate max and min positions
for(i = 1:5)
    Pmax(i) = upperLim(i) * (180/pi/ratios(i)) + servoOffsets(i);
    Pmin(i) = lowerLim(i) * (180/pi/ratios(i)) + servoOffsets(i);
end
Pmax(6) = upperLim(6) / -0.028 + servoOffsets(6);
Pmin(6) = lowerLim(6) / -0.028 + servoOffsets(6);

S(1) = qdot(1) * (180/pi/0.102);
S(2) = qdot(2) * (180/pi/0.105);
S(3) = qdot(3) * (180/pi/0.109);
S(4) = qdot(4) * (180/pi/0.095);
S(5) = qdot(5) * (180/pi/0.102);
S(6) = qdot(6) / 0.028;

str{1} = '';
str{2} = '';
str{3} = '';
str{4} = '';
str{5} = '';
str{6} = '';

for(i=1:6)
    if(S(i) > 0) %If velocity is positive
        P(i) = Pmax(i);
    else %Velocity is negative
        P(i) = Pmin(i);
    end
    if( abs(S(i)) > smallestVels(i) )
        str{i} = ['#' num2str(i,'%.0f') 'P' num2str(P(i),'%.0f') ' S' num2str(S(i),'%.0f')];
    else
        str{i} = ['STOP' num2str(i,'%.0f')];
    end
        
end


%% Sending commands to lynx
if(nargin == 6)
    for(i = 1:6)
        fprintf(ttl, '%s\r',[str{i}]);
    end
end

end
