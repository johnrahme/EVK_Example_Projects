clear all;
close all;
format long;

%--------------------------
%@@@@@@@@@@@@@@@@@@@@@@@@@@
%     Init Java commands
%--------------------------

%javaaddpath(fullfile(matlabroot,'work','triateration.jar'))
javaaddpath('Trilateration.jar');
tri = com.lemmingapex.trilateration.TrilaterationTest;

%@@@@ End of java init @@@@@

%---------------------------
%@@@@@@@@@@@@@@@@@@@@@@@@@@@
%---------------------------
%          INFO:
%
% Mother anchor = A(1)
% Anchor 1 = A(2)
% Anchor 2 = A(3)
%
%---------------------------
%@@@@@@@@@@@@@@@@@@@@@@@@@@@
%---------------------------
%initializing values/ports


port = 'COM3'; %Where 3 is COMport number (usually standard)
BR = 9600; % BaudRate of port
obj = serial(port, 'BaudRate', BR); % Creating object to read serial
                                    % with baudrate 9600
fopen(obj); %opens object

% Origin of anchors
% (Needs to be hard coded)
x1 = 0;
y1 = 0;

x2 = 1.9;
y2 = 0;

x3 = 0;
y3 = 1.8;

%r1 = 2;
%r2 = 2;
%r3= 2;

%Java input, needs the position of the three anchors
positions =[ x1,y1; x2, y2 ;  x3, y3 ];

%Places all x,y-values to take min/max to lock axis
xtot = [x1,x2,x3];
ytot = [y1,y2,y3];


% Creates window to track movement
figure;

while(1)
   
    hold on;
    %Locks the axis
    xlim([(min(xtot)-2) (max(xtot)+2)]);
    ylim([(min(ytot)-2) (max(ytot)+2)]);
    
    %Plots the anchors (Which are stationary)
    plot(x1,y1,'ro',x2,y2,'ro',x3,y3,'ro');
    
    %Just put in the string and 
    %use %d for the values you want.
    %Reads data from mother node.
    A = fscanf(obj, ['D1: %d D2: %d D3: %d']) 
    
    r1 = A(1,1)/1000;  %divides it down to [m]
    r2 = A(2,1)/1000;
    r3 = A(3,1)/1000;
    
    %Java input, takes the real time distances.
    distances = [r1, r2, r3];

    p = javaMethod('trilateration2DInexact1',tri, positions, distances);
    disp(p);
    
    %Th for plotting of circles.
    %rX for each radius of circle (distances).
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
    
    %Plot the estimated position with an error of e-meters around it.
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


%@@@@@@@@@@@@@@@@@@@@@@@@@@@@
%           End info

% If canceled MATLAB will need a restart due 
% to COMport already being used. Seems to be a problem
% that cant be solved.

%@@@@@@@@@@@@@@@@@@@@@@@@@@@@