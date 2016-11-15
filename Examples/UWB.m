% Project file for UWB

clear all;
close all;

port = 'COM3'; %Where 3 is COMport number
BR = 9600; % BaudRate of port
obj = serial(port, 'BaudRate', BR); % Creating object to read serial
                                    % with baudrate 9600
                                   
fopen(obj);
i = 0;
while(i<100)
    A = fscanf(obj)
    pause(0.1);
    i = i+1;
end

fclose(obj);
delete(obj);
clear obj;
