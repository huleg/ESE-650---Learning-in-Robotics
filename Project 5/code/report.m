clc
clear all
close all

addpath('./matlab');

% for drive model, change to 'car' and for walking use 'walk'
mode = 'car';

colors = ['c.';'y.';'g.';'b.';'m.'];

load(['costs_',mode,'.mat']);

img = imread('aerial_color_resize.jpg');
imshow(img)
for i = 1:5
    [x,y] = ginput(2);
    ctg = dijkstra_matrix(costs,floor(y(2)),floor(x(2)));
    [ip1, jp1] = dijkstra_path(ctg, costs, floor(y(1)), floor(x(1)));
    linearInd2 = sub2ind(size(costs), ip1, jp1);
    hold on
    plot(jp1,ip1,colors(i,:))
    plot(x,y,'r.')
    hold off
    pause;
end