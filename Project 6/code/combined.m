clc
clear all
close all

% folder = './drill';
folder = './liq_container';

load(strcat(folder,'/model.mat'));
load('./params/calib_vicon_xtion.mat')
load(strcat(folder,'/pose.mat'));
R = eye(3);
T = [0;0;0];

for k =60:20:447
    depth = imread(strcat(folder,'/depth/',sprintf('%06d.png',k)));
    rgb = imread(strcat(folder,'/rgb/',sprintf('%06d.png',k)));
    
    [pcx, pcy, pcz, r, g, b, D_, X, Y,validInd] = depthToCloud_full_RGB(depth, rgb, './params/calib_xtion.mat');
    P_ = s_cam3*[pcx pcy pcz]*[0 0 1;-1 0 0;0 -1 0]'*R_cv3 + repmat(t_cv3,length(pcx),1);
    
    temp = find(P_(:,3)<200);
    rotatedMdata = P_(temp,:)';
    
    temp2 = P_(find(P_(:,3)>200),:)';
    pcshow(temp2','r');
    hold on
    pcshow(rotatedMdata','g');
    hold off


    %% RANSAC
    N = size(rotatedMdata,2);
    bestInNum = 0; 
    threshDist = 10;
    inlierRatio = 0.8;
    for i=1:1000
        idx = randperm(N,3);
        sample = rotatedMdata(:,idx);
        planeNorm = cross(sample(:,3)-sample(:,1),sample(:,2)-sample(:,1));
        D = -planeNorm'*sample(:,1);
        distance = abs(planeNorm'*rotatedMdata+D)./norm(planeNorm,2);
        inlierIdx = find(abs(distance)<=threshDist);
        outlierIdx = find(abs(distance)>threshDist);
        inlierNum = length(inlierIdx);
        if inlierNum>=round(inlierRatio*N) && inlierNum>bestInNum
            bestInNum = inlierNum;
            bestParameters=[planeNorm;D];
            groundPoints = rotatedMdata(:,inlierIdx);
            objectPoints = rotatedMdata(:,outlierIdx);
        end
    end
    
    pcshow(groundPoints','r');
    hold on
    pcshow(objectPoints','g');
    hold off
    
    %% Filtering
    med = median(objectPoints,2);
    dist = pdist([med';objectPoints'],'euclidean');
    dist = dist(1:size(objectPoints,2));
    objectPoints(:,dist>170) = [];
    
    pcshow(groundPoints','r');
    hold on
    pcshow(objectPoints','g');
    hold off
    
    %% Set initial pose
    figure,
    rotatedMdata = objectPoints;
    rotatedMdata = bsxfun(@minus,rotatedMdata,mean(rotatedMdata,2));
    
    %% Perform ICP
    tic;
    [R,T] = icp(Mdata,rotatedMdata,'plane',100);
    toc

end