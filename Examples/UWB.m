% Project file for UWB

%starting with clearing everything
clear all;
close all;

%---------------------------
%@@@@@@@@@@@@@@@@@@@@@@@@@@@
%---------------------------
%          INFO
%
% Mother anchor = A(1)
% Anchor 1 = A(2)
% Anchor 2 = A(3)
%
%---------------------------
%@@@@@@@@@@@@@@@@@@@@@@@@@@@
%---------------------------
%initializing values/ports

time = 100; %For how long will you track?
port = 'COM3'; %Where 3 is COMport number (usually standard)
BR = 9600; % BaudRate of port
obj = serial(port, 'BaudRate', BR); % Creating object to read serial
                                    % with baudrate 9600
fopen(obj); %opens object
i = 1;
result = zeros(3, time); %resulting vector

% Origin of anchors with the acceptable error
x1 = 0;
y1 = 0;

x2 = 0;
y2 = 1;

x3 = 1;
y3 = 0;

e = 0.3;
%---------------------------

%---------------------------

%A1 = sqrt(8);
%A2 = 2;        %Example lengths
%A3 = 2;

%Begins figure and plotting the anchors positioning
figure;
hold on;
plot(x1,y1,'ro',x2,y2,'ro',x3,y3,'ro')

%---------------------------
%Starts if-statement to plot the position in real time
%
while(1)
    
    A = fscanf(obj, ['D1: %d D2: %d D3: %d']) %Just put in the string and 
    %use %d for the values you want
    
    if (A(1,1) <= 0)
        A(1,1) = 10;
    end
    if (A(2,1) <= 0)
        A(2,1) = 10;
    end
    if (A(3,1) <= 0)
        A(3,1) = 10;
    end
    
    A1 = A(1,1)/1000;  %divides it down to [m]
    A2 = A(2,1)/1000;
    A3 = A(3,1)/1000;
  
     
                                              
                                              
    result(1,i) = A1; %puts first value on first line each time
    result(2,i) = A2; %same here
    result(3,i) = A3; %same here
    [xout1, yout1] = circcirc(x1,y1,A1,x2,y2,A2); %checks intersection
    [xout2, yout2] = circcirc(x1,y1,A1,x3,y3,A3); % - || -
    
    xo11 = (xout1(1)); %easier to write
    xo12 = (xout1(2));
    xo21 = (xout2(1));
    xo22 = (xout2(2));
    yo11 = (yout1(1));
    yo12 = (yout1(2));
    yo21 = (yout2(1));
    yo22 = (yout2(2));
    
%     aint = (xout1(1),yout1(1));
%     bint = (xout1(2),yout1(2));
%     cint = (xout2(1),yout2(1));
%     dint = (xout2(2),yout2(2));
    
    
    %checks if the intersections matches and plots the
    %intersection using trilaturation
    
    %if aint = 
    
    if ((xo11 <= xo21+e) && (xo11 >= xo21-e))
        if ((yo11 <= yo21+e) && (yo11 >= yo21-e));
            h = plot(xo11, yo11,'b*');
        else h = plot(xo11,yo12,'b*');
        end
    elseif ((xo12 <= xo21+e && xo12 >= xo21-e ))
        if (yo11 <= yo21+e && yo11 >= yo21-e);
            h = plot(xo12, yo11,'b*');
        else h = plot(xo12,yo12,'b*');
        end
    elseif ((xo11 <= xo22+e && xo11 >= xo22-e ))
        if (yo11 <= yo21+e && yo11 >= yo21-e);
            h = plot(xo11, yo11,'b*');
        else h = plot(xo11,yo21,'b*');
        end
    elseif ((xo12 <= xo22+e && xo12 >= xo22-e ))
        if (yo11 <= yo21+e && yo11 >= yo21-e)
            h = plot(xo12, yo11,'b*');
        else h = plot(xo12,yo12,'b*');
        end
    else h = plot(0,0,'g*');
    end

    hold on
    %pauses, dont need to have continious time
    pause(0.5);
    
    %A1 = A1-0.1;
    %A2 = A2- 0.05;
    %A3 = A3 + 0.075;
    
    %deletes the old value of the plot so that we only see in 
    %real time
    %delete(h);
    
    %if you dont want inf loop, adds i.
    i = i+1;
    
end
%---------------------------


result;


%---------------------------
%closes everything
fclose(obj);
delete(obj);
clear obj;
%---------------------------