clc
clear all
close all

datapath = '../Data/';
vicon_data = true;
cal_bias = false;
stitch_image = true;

dataset = dir(strcat(datapath,'imu/*.mat'));
for t = 1:size(dataset)
    filename = strcat(datapath,'imu/imuRaw',num2str(t),'.mat');
    imu = load(filename);
    
    acc = imu.vals(1:3,:);
    gyro = imu.vals(4:6,:);
    gyro(4,:) = gyro(1,:);
    gyro(1,:) = [];
    
    gyro_bias = mean(gyro(:,1:200),2);
    gyro_corrected = bsxfun(@minus,gyro,gyro_bias);
    gyro = gyro_corrected * 3300/1023 * pi/180/3.33;
    
    if cal_bias
        acc_bias = accl_bias(datapath);
    else
        load('acc_bias.mat');
    end
    acc = bsxfun(@plus,bsxfun(@times,acc_bias(1,:)',acc),acc_bias(2,:)');
    
    eul_gyro = [];
    rot_gyro = [];
    eul_acc = [];
    eul_com = [];
    orientation = [];
    q = rot2quat(gyro(:,1),1);
    for i = 2:length(imu.vals)
        dt = imu.ts(i)-imu.ts(i-1);
        delta_q = rot2quat(gyro(:,i),dt);
        q = qMul(q,delta_q);
        rot_gyro(:,:,i) = quat2rotm(q');
        eul_gyro(:,i) = rotm2eul(rot_gyro(:,:,i));
        
        eul_acc(1,i) = 0;
        eul_acc(2,i) = atan2(-acc(1,i),sqrt(acc(2,i)^2+acc(3,i)^2));
        eul_acc(3,i) = atan2(acc(2,i),sqrt(acc(1,i)^2+acc(3,i)^2));
        
        eul_com(1,i) = eul_gyro(1,i);
        eul_com(2,i) = 0.7*eul_acc(2,i) + 0.3*eul_gyro(2,i);
        eul_com(3,i) = 0.7*eul_acc(3,i) + 0.3*eul_gyro(3,i);
        orientation(:,:,i) = eul2rotm(eul_com(:,i)'); 
    end
    
    
    
    if vicon_data
        filename = strcat(datapath,'vicon/viconRot',num2str(t),'.mat');
        vicon = load(filename);
        vicon_angle = rotm2eul(vicon.rots);
    end
    
    figure,
    subplot(3,1,1)
    hold on
    %     plot(ts_imu,eul_gyro(1,:));
    %     plot(ts_imu,eul_acc(1,:));
    plot(imu.ts,eul_com(1,:));
    if vicon_data
        plot(vicon.ts,vicon_angle(:,1));
        legend('Comp','Vicon')
    end
    ylabel('Yaw')
    hold off
    
    subplot(3,1,2)
    hold on
    %     plot(ts_imu,eul_gyro(2,:));
    %     plot(ts_imu,eul_acc(2,:));
    plot(imu.ts,eul_com(2,:));
    if vicon_data
        plot(vicon.ts,vicon_angle(:,2));
        legend('Comp','Vicon')
    end
    ylabel('Pitch')
    hold off
    
    subplot(3,1,3)
    hold on
    %     plot(ts_imu,eul_gyro(3,:));
    %     plot(ts_imu,eul_acc(3,:));
    plot(imu.ts,eul_com(3,:));
    if vicon_data
        plot(vicon.ts,vicon_angle(:,3));
        legend('Comp','Vicon')
    end
    ylabel('Roll')
    hold off
    drawnow;
    
    if stitch_image
        stitch(datapath,orientation,t,imu.ts);
    end
end