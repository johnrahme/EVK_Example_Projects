clear all;
format long;
%javaaddpath(fullfile(matlabroot,'work','triateration.jar'))
javaaddpath('Trilateration.jar');
tri = com.lemmingapex.trilateration.TrilaterationTest;
x1 = 0;
y1 = 0;
x2 = 0.2;
y2 = 0;
x3 = 0;
y3 =0.2;
r1 = 0.13;
r2 = 0.13;
r3= 0.13;
positions =[ x1,y2; x2, y2 ;  x3, y3 ];
distances = [r1,r2,r3];

p = javaMethod('trilateration2DInexact1',tri, positions, distances);
disp(p);
th = 0:pi/50:2*pi;
xunit = r1 * cos(th) + x1;
yunit = r1 * sin(th) + y2;
h = plot(xunit, yunit);
hold on
xunit2 = r2 * cos(th) + x2;
yunit2 = r2 * sin(th) + y2;
h = plot(xunit2, yunit2);
hold on
xunit3 = r3 * cos(th) + x3;
yunit3 = r3 * sin(th) + y3;
h = plot(xunit3, yunit3);
h = plot(p(1),p(2), 'b*');


