
function lynxInitializeHardware(serialPort)
%Opens serial communication with the lynx

%serialPort is a string. e.g. 'com3'
%The com port may change.  Go to Device manager on a PC to find the port.

%Close any previous communication.
delete(instrfindall);

global ttl

%Alternatively, on a MAC, 
%ttl = serial('/dev/cu.usbserial-AI0484D4'); 
%Note that the portion after the hypen depends on the usb cable.  Type 
%ls /dev/cu.* into the command line to find this name.

ttl = serial(serialPort); 
ttl.BaudRate = 115200;
fopen(ttl);

if(~strcmp(ttl.Status,'open'))
    error('Please connect the Lynx robot via the USB port');
end

end
