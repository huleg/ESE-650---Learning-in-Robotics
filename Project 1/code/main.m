clc
clear all
close all

% To perform training, make training = 1
% If training = 0, load model.mat
training = 0;

if training == 1
    model = train_barrel('train/','masks/');
else
    load('model.mat');
end


fig1 = figure;
fig2 = figure;

center = []
distance = []

% Testing
dirstruct = dir('test/*.png');
for i = 1:length(dirstruct),
    % Read one test image
    im = imread(strcat('test/',dirstruct(i).name));
    
    figure(fig1)
    imshow(im)
    %title(dirstruct(i).name)
    % Your computations here!
    [x, y, d, mask] = compute_distance(im,model);
    
    % Display results:
    % (1) Segmented image
    % (2) Barrel center
    % (3) Distance of barrel
    % You may also want to plot and display other diagnostic information
    figure(fig2)
    imshow(mask)
    %title(strcat('center= ',num2str(x(1)),',',num2str(y(1)),' ; distance = ',num2str(d(1))));
    center = [center [x;y]]
    distance = [distance d]
    hold on
    plot(x,y,'r*')
    pause;
end