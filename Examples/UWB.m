% Project file for UWB

%starting with clearing everything
clear all;
close all;

%initializing values/ports

port = 'COM3'; %Where 3 is COMport number (usually standard)
BR = 9600; % BaudRate of port
%obj = serial(port, 'BaudRate', BR); % Creating object to read serial
                                    % with baudrate 9600
                                   
%fopen(obj); %opens object
i = 1;
result = zeros(3, 10); %resulting vector

%format = '%f';

% Origin of anchors (just an example)
x1 = 0;
y1 = 0;

x2 = 0;
y2 = 2;

x3 = 2;
y3 = 0;

% Overview of how the loop should be if we want to plot after a while.
% Puts the read values into a resulting vector
% that saves every value and plot
A1 = sqrt(8);
A2 = 2;
A3 = 2;
e = 0.0001
figure;
hold on;
plot(x1,y1,'ro',x2,y2,'ro',x3,y3,'ro')
while(i<10)
    
  %  A = fscanf(obj, ['D1: %d D2: %d D3: %d']) %Just put in the string and 
                                              %use %d for the values you want
  %  A1 = A(1,1);
  %  A2 = A(2,1);
  %  A3 = A(3,1);
  
     
                                              
                                              
    result(1,i) = A1; %puts first value on first line each time
    result(2,i) = A2; %same here
    result(3,i) = A3; %same here
    [xout1, yout1] = circcirc(x1,y1,A1,x2,y2,A2);
    [xout2, yout2] = circcirc(x1,y1,A1,x3,y3,A3);
    
    xo11 = (xout1(1));
    xo12 = (xout1(2));
    xo21 = (xout2(1));
    xo22 = (xout2(2));
    yo11 = (xout1(1));
    yo12 = (xout1(2));
    yo21 = (xout2(1));
    yo22 = (xout2(2));
    
    if ((xo11 <= xo21+e && xo11 >= xo21-e ))
        if (yo11 <= yo21+e && yo11 >= yo22-e)
            plot(xo11, yo11,'b*');
        else plot(xo11,yo21,'b*');
        end
    elseif ((xo12 <= xo21+e && xo12 >= xo21-e ))
        if (yo11 <= yo21+e && yo11 >= yo22-e)
            plot(xo12, yo11,'b*');
        else plot(xo12,yo12,'b*');
        end
    elseif ((xo11 <= xo22+e && xo11 >= xo22-e ))
        if (yo11 <= yo21+e && yo11 >= yo22-e)
            plot(xo11, yo11,'b*');
        else plot(xo11,yo21,'b*');
        end
    elseif ((xo12 <= xo22+e && xo12 >= xo22-e ))
        if (yo11 <= yo21+e && yo11 >= yo22-e)
            plot(xo12, yo11,'b*');
        else plot(xo12,yo12,'b*');
        end
    end
    pause(0.1);
    A1 = sqrt(2);
    A2 = sqrt(2);
    A3 = sqrt(2);
    
    i = i+1;
    
end


%intersect(intersect(xq(in2),xq(in3)),xq(in4));
%[xout,yout] = circcirc(x1,y1,r1,x2,y2,r2)

result



%closes everything ----------------------------
fclose(obj);
delete(obj);
clear obj;
