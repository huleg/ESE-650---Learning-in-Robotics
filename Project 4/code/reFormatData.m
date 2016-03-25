clc
clear all
close all

load('Test\test_lidar.mat','lidar')
load('Test\test_joint.mat','head_angles','ts')

for i=1:numel(lidar)
    ts_lidar(i) = lidar{i}.t;
    pose(i,:) = lidar{i}.pose;
    rpy(i,:) = lidar{i}.rpy;
    scan(i,:) = lidar{i}.scan;
end
%ts_joint = bsxfun(@plus,ts,ts_lidar(1));
ts_joint = ts;
clear i
clear lidar
clear ts
save('test1.mat');