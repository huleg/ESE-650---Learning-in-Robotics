clc
clear all
close all

addpath('matlab');

img = im2double(imread('walk.jpg'));
% a = imresize(img,0.25);
% imwrite(a,'aerial_color_resize.jpg');
no_of_paths = 10;


for j = 1:no_of_paths
    imshow(img);
    pause();
    [x,y] = ginput();
    
    x = ceil(x);
    y = ceil(y);
    pathx = [];
    pathy = [];
    
    map = zeros(size(rgb2gray(img)));
    
    for i = 1:length(x)-1
        [temp1,temp2] = getMapCellsFromRay(x(i),y(i),x(i+1),y(i+1));
        pathx = [pathx;temp1];
        pathy = [pathy;temp2];
        linearInd = sub2ind(size(map), temp2, temp1);
        map(linearInd) = 1;
    end
    Path{j} = [pathx,pathy];
    PathPoints{j} = [x,y];

    imshow(img);
    hold on
    for k = 1:length(Path)
        plot(Path{k}(:,1),Path{k}(:,2),'r.')
        plot(PathPoints{k}(:,1),PathPoints{k}(:,2),'g.')
    end
    hold off
    pause;
end
save('Path_walk','Path','PathPoints');