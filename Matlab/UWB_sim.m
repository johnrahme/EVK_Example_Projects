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

% Origin of anchors with the acceptable error
x1 = 0;
y1 = 0;

x2 = 0;
y2 = 0.2;

x3 = 0.2;
y3 = 0;

e = 0.05;
time = 1; %For how long will you track?
result = zeros(3, time); %resulting vector
%---------------------------

%---------------------------

%A1 = sqrt(8);
%A2 = 2;        %Example lengths
%A3 = 2;

%Begins figure and plotting the anchors positioning
figure;
hold on;
plot(x1,y1,'ro',x2,y2,'ro',x3,y3,'ro')

i = 1;
%---------------------------
%Starts if-statement to plot the position in real time
%
simValues = zeros(3,time);
simValues = [100 160 160];  

while(i<=time)
    A1 = simValues(i,1)/1000  %divides it down to [m]
    A2 = simValues(i,2)/1000
    A3 = simValues(i,3)/1000
  
     
                                              
                                              
    result(1,i) = A1; %puts first value on first line each time
    result(2,i) = A2; %same here
    result(3,i) = A3; %same here
    
    [xout1, yout1] = circcirc(x1,y1,A1,x2,y2,A2) %checks intersection
    [xout2, yout2] = circcirc(x1,y1,A1,x3,y3,A3) % - || -
    
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
    
    if (abs(xo11-xo21) < e && abs(yo11-yo21) < e)
        h = plot(xo11, yo11,'b*');
    elseif (abs(xo12-xo21) < e && abs(yo12-yo21) < e)
            h = plot(xo12, yo12,'b*');
    elseif (abs(xo11-xo22) < e && abs(yo11-yo22) < e)
         h = plot(xo11, yo11,'b*');
    elseif (abs(xo11-xo21) < e && abs(yo11-yo21) < e)
           h = plot(xo11, yo21,'b*');
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