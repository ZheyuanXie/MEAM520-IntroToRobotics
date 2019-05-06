function [] = lynxServo(th1, th2, th3, th4, th5, th6)
% Commands the Lynx to the angles defined by the input (in radians and mm)
%
% INPUTS:
%   th1...th6 : Joint variables for the six DOF Lynx arm (radians and
%   mm)
% OUTPUT:
%   Calls lynxServoSim.m and displays the Lynx in the configuration specified by 
%   the input parameters, or calls lynxServoPhysical.m to move the real
%   robot.

    % Ensure the Lynx has been initialized
    if(~evalin('base','exist(''lynx'',''var'')'))
        error('lynxServo:lynxInitialization','Lynx has not been initialised. Run lynxStart first');
    end
    
    % Import global variables
    global lynx q time_prev time_prev_vel 
    
    if isempty(time_prev)
        time_prev = tic;
    end
    if isempty(time_prev_vel)
        time_prev_vel = tic;
    end
    
    % Check inputs (whether q is 6 entries or single vector)
    if nargin == 6 && length(th1) == 1 
        qNew = [th1 th2 th3 th4 th5 th6];
    elseif nargin == 1 && length(th1) == 6 && numel(th1) == 6
        qNew = th1;
        % Ensure that q is a row vector
        if size(qNew, 1) > 1
            qNew = qNew';
        end
    else
        error('Input must be 6 scalars or a 6 element vector representing desired joint angles followed by the robot name.')
    end
    
    if any(isnan(qNew))
        error('lynxServo:nanInput',['Input contains a value of NaN. Input was: [' num2str(qNew) ']']);
    end
    
    % Check that angular limits are satisfied
    lowerLim = [-1.4 -1.2 -1.8 -1.9 -2 -15]; % Lower joint limits in radians (grip in mm (negative closes more firmly))
    upperLim = [1.4 1.4 1.7 1.7 1.5 30]; % Upper joint limits in radians (grip in mm)
    
    for i=1:length(qNew)
        if qNew(i) < lowerLim(i)
            qNew(i) = lowerLim(i);
            fprintf('Joint %d was sent below lower limit, moved to boundary %0.2f\n',i,lowerLim(i))
        elseif qNew(i) > upperLim(i)
            qNew(i) = upperLim(i);
            fprintf('Joint %d was sent above upper limit, moved to boundary %0.2f\n',i,upperLim(i))
        end
    end

    % Check that angular velocity limits are satisfied
    dt = toc(time_prev_vel); %time since velocity limit was last checked
    time_prev_vel = tic;

    % Admittedly kinda arbitrary. If you have a reason to change this feel
    % free to ask, but in general this should keep the robot safe.
    maxOmegas = [1 1 1 2 3 20];
    
    if dt > 0
        omega = abs((qNew - q)/dt);
        speedind = find(omega>maxOmegas);	
        if ~isempty(speedind)
            jointErrStr = [];
            for i = 1:length(speedind)
                jointErrStr = [jointErrStr sprintf('\nJoint %d has approached an angular velocity of %.3f rad/s. The max angular velocity for joint %d is %0.3f rad/s', speedind(i), omega(speedind(i)),speedind(i),maxOmegas(speedind(i)))];
            end
           error('lynxServo:VelocityLimit','%s',jointErrStr);
        end
    end
    
    q = qNew;

    % Send the angles to the Lynx
    if lynx.hardware_on
        
        lynxServoPhysical(q(1),q(2),q(3),q(4),q(5),q(6));
        
    else
        lynxServoSim(q);
    end
    
end
