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

figure;
imagesc(img);
xlabel('Raster Column');
ylabel('Raster Row');
colormap(gray);

hold on;
plot(100,300,'b-*','linewidth',1.5);