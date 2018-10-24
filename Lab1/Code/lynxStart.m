function lynxStart(varargin)
% lynxStart  Loads kinematic data for a Lynx AL5D manipulator and sets
%   variables.
%
%   If using hardware:
%   Starts the Lynx and moves the Lynx to its home position. 
%
%   WARNING: Be aware of the Lynx's position before turning on the 
%   controller. To prevent damage, remove any objects in the path between 
%   the current position and the home position. If the Lynx is likely to 
%   slam into the table surface or other immovable object, manually move 
%   the Lynx towards the home position before using this command.
%
%     Options         Values {Default Value}
%       'Hardware'      'Legend' | 'Loopy' | 'Lucky' | 'Lyric' | {'off'} Sets up the code to use the Lynx
%                       hardware.  If hardware is enabled then there
%                       is no visualization.  Default is simulation.
%                       Be careful to read the above warning before 
%                       enabling hardware.
% 
%       'Port'          string. e.g. 'com3'  -  The com port may change.  
%                       Go to Device manager on a PC to find the port.
%                       To use the Lynx with a MAC, see instructions in
%                       lynxInitializeHardware.m
%
%       'Joints'        {'on'} | 'off' Display the joints of the robot
%
%       'Frame'         {'on'} | 'off' Display the end effector frame
%
%       'Shadow'        {'on'} | 'off' Plot the shadow of the robot on the
%                       ground plane
%
%       'Gripper'       'on' | {'off'} Plot the gripper.  Cool, but is it
%                       really helpful? 
%
%  Updated for MEAM 520, 2018. May still have a few bugs, so please let us
%  know if you find any bugs.  And better yet, suggest how to fix it =)

global lynx robotName q pennkey

%%SET THIS
pennkey = 'zyxie_lancelan';

lynx.firstFrame = true;
lynx.hardware_on = false;
lynx.showFrame = true;
lynx.showJoints = true;
lynx.showShadow = true;
lynx.showGripper = false;

% Home pose
q = [0,0,0,0,0,0];

% Check property inputs is even (each property must be paired with a value)
if mod(size(varargin,2), 2) == 1
    error('Must be a value for each property set')
end

for j = 1:2:size(varargin,2)
    % Use the Lynx hardware or simulation
    if strcmpi(varargin{1,j}, 'Hardware')
        if strcmpi(varargin{1,j+1}, 'off')
            lynx.hardware_on = false;
            
        elseif strcmpi(varargin{1,j+1}, 'Legend') || ...
                strcmpi(varargin{1,j+1}, 'Loopy') || ...
                strcmpi(varargin{1,j+1}, 'Lucky') || ...
                strcmpi(varargin{1,j+1}, 'Lyric')
                robotName = varargin{1,j+1};
				lynx.hardware_on = true;
		else
			error('Invalid value for Hardware property');
        end
        
    elseif strcmpi(varargin{1,j}, 'Port')
        lynx.serialPort = varargin{1,j+1};
        
    elseif strcmpi(varargin{1,j}, 'Joints')
        if strcmpi(varargin{1,j+1}, 'off')
            lynx.showJoints = false;
        elseif strcmpi(varargin{1,j+1}, 'on')
            lynx.showJoints = true;
        else
            error('Invalid value for Joints property');
        end
        
    elseif strcmpi(varargin{1,j}, 'Frame')
        if strcmpi(varargin{1,j+1}, 'off')
            lynx.showFrame = false;
        elseif strcmpi(varargin{1,j+1}, 'on')
            lynx.showFrame = true;
        else
            error('Invalid value for Frame property');
        end
        
    elseif strcmpi(varargin{1,j}, 'Shadow')
        if strcmpi(varargin{1,j+1}, 'off')
            lynx.showShadow = false;
        elseif strcmpi(varargin{1,j+1}, 'on')
            lynx.showShadow = true;
        else
            error('Invalid value for Shadow property');
        end
        
    elseif strcmpi(varargin{1,j}, 'Gripper')
        if strcmpi(varargin{1,j+1}, 'off')
            lynx.showGripper = false;
        elseif strcmpi(varargin{1,j+1}, 'on')
            lynx.showGripper = true;
        else
            error('Invalid value for Gripper property');
        end
       
    end
    
end

% Initialize the Lynx if using hardware
if lynx.hardware_on
    
    % Display warning message
    str = input(['Warning: Be aware of the Lynx''s position before continuing.\n'...
        'Hold the lynx as close to the home position as possible.\n' ...
        'Do you want to continue (y/n)?\n'],'s');
    if isempty(str)
        str = 'n';
    end
    if strcmpi(str, 'y') || strcmpi(str, 'yes')
        lynxInitializeHardware(lynx.serialPort);
    else
        disp('Start cancelled.');
    end
    
% Initialize the plot if using simulation
else
    plotLynx(q);
end

% Set global variables in the base workspace
evalin('base', 'global lynx q')

%Send the robot to a home configuration
lynxServo(q);
