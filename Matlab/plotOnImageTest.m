% replace with an image of your choice
img = imread('layout2.jpg');
 
% set the range of the axes
% The image will be stretched to this.
min_x = 0;
max_x = 8;
min_y = 0;
max_y = 6;
 
% make data to plot - just a line.
x = min_x:max_x;
y = (6/8)*x;

%plot(x,y,'b-*','linewidth',1.5);

% NOTE: if your image is RGB, you should use flipdim(img, 1) instead of flipud.
i = 0;

currX = 0;
currY = 0;
%figure;
while(i < 5)
imagesc([0 5], [0 5], flipud(img));
xlabel('Raster Column');
ylabel('Raster Row');
colormap(gray);
set(gca,'ydir','normal');
hold on;
currX = i/2;
currY = i/2;
plot(currX,currY,'b-*','linewidth',1.5);
i = i+1;
pause(1);
hold off;
end