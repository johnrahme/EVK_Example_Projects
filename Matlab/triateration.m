clear all;
close all;
format long;
%javaaddpath(fullfile(matlabroot,'work','triateration.jar'))
javaaddpath('Trilateration.jar');
tri = com.lemmingapex.trilateration.TrilaterationTest;

port = 'COM3'; %Where 3 is COMport number (usually standard)
BR = 9600; % BaudRate of port
obj = serial(port, 'BaudRate', BR); % Creating object to read serial
                                    % with baudrate 9600
fopen(obj); %opens object

x1 = 0;
y1 = 0;
x2 = 1.9;
y2 = 0;
x3 = 0;
y3 = 1.8;
%r1 = 2;
%r2 = 2;
%r3= 2;
positions =[ x1,y2; x2, y2 ;  x3, y3 ];

xtot = [x1,x2,x3];
ytot = [y1,y2,y3];

figure;

while(1)
   
    hold on;
    xlim([(min(xtot)-2) (max(xtot)+2)]);
    ylim([(min(ytot)-2) (max(ytot)+2)]);
    plot(x1,y1,'ro',x2,y2,'ro',x3,y3,'ro');
    A = fscanf(obj, ['D1: %d D2: %d D3: %d']) %Just put in the string and 
    %use %d for the values you want
    
    r1 = A(1,1)/1000;  %divides it down to [m]
    r2 = A(2,1)/1000;
    r3 = A(3,1)/1000;
    
    distances = [r1, r2, r3];

    p = javaMethod('trilateration2DInexact1',tri, positions, distances);
    disp(p);
    th = 0:pi/50:2*pi;
    xunit = r1 * cos(th) + x1;
    yunit = r1 * sin(th) + y1;
    h = plot(xunit, yunit);
    
    xunit2 = r2 * cos(th) + x2;
    yunit2 = r2 * sin(th) + y2;
    h = plot(xunit2, yunit2);
   
    xunit3 = r3 * cos(th) + x3;
    yunit3 = r3 * sin(th) + y3;
    h = plot(xunit3, yunit3);
    h = plot(p(1),p(2), 'b*');

    e = 0.1;
    punitx = e*cos(th) + p(1);
    punity = e*sin(th) + p(2);
    plot(punitx,punity, 'g-');
    
    pause(0.1); 
    clf
    %delete(h);
    hold off;
    
end
