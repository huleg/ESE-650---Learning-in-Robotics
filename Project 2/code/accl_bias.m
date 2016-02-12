function bias = accl_bias(path)

acc_stack = [];
R_stack = [];

for i = [1:5,8:9]
    filename = strcat(path,'imu/imuRaw',num2str(i),'.mat');
    imu = load(filename);
    acc = imu.vals(1:3,:);
    filename = strcat(path,'vicon/viconRot',num2str(i),'.mat');
    vicon = load(filename);
    
    g = [0;0;1];
    R = zeros(3,size(acc,2));
    for j=1:size(acc,2)
        [~, index] = min(abs(vicon.ts-imu.ts(j)));
        R(:,j) = vicon.rots(:,:,index)'*g;
    end
    
    acc_stack = [acc_stack, acc];
    R_stack = [R_stack, R];
    
end

sx = [acc_stack(1,:)',ones(size(acc_stack,2),1)]\R_stack(1,:)';
sy = [acc_stack(2,:)',ones(size(acc_stack,2),1)]\R_stack(2,:)';
sz = [acc_stack(3,:)',ones(size(acc_stack,2),1)]\R_stack(3,:)';

bias = [sx,sy,sz];
