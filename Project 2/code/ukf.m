clc
clear all
close all

%% Path to the folder containing imu,cam and vicon data in respective folders
datapath = '../Data/';

%% To calculate the accelerometer bias from training set make cal_bias = trues
cal_bias = false;
%% If camera image data is available, make true to stitch image
stitch_image = true;
%% If vicon data is available, make it true to plot for comparison
vicon_data = true;
%% Make rotation_plot true for visual representation of the body in a 3d space
rotation_plot = false;

if rotation_plot
    fig1 = figure;
end

dataset = dir(strcat(datapath,'imu/*.mat'));
for j = 1:size(dataset)
    filename = strcat(datapath,'imu/imuRaw',num2str(j),'.mat');
    imu = load(filename);
    
    if vicon_data
        filename = strcat(datapath,'vicon/viconRot',num2str(j),'.mat');
        vicon = load(filename);
        vicon_angle = rotm2eul(vicon.rots);
    end
    
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
    
    %%% UKF %%%
    q_x = [1 0 0 0]';
    w_x = [0;0;0];
    P = 1e-2*eye(6);
    
    Q = diag([ones(3,1)*78.97;ones(3,1)*6.5]);
    R = diag([ones(3,1)*0.2;ones(3,1)*3]);
    x_q_bar = [1;0;0;0];
    saved_q = [];
    n = length(P);
    
    for t = 1:length(imu.vals)
        disp(strcat('Sample No: ',num2str(t)));
        S = chol(P+Q);
        W = [sqrt(n)*S, -sqrt(n).*S];
        
        %% Sigma points %%
        for i = 1:size(W,2)
            X_q(:,i) = qMul(q_x,rot2quat(W(1:3,i),1));
            X_w(:,i) = w_x + W(4:end,i);
            X_q(:,i) = bsxfun(@rdivide, X_q(:,i),sqrt(sum( X_q(:,i).^2)));
        end
        
        %% Process Model %%
        if t == 1
            delta_t = 0.01;
        else
            delta_t = imu.ts(:,t)-imu.ts(:,t-1);
        end
        for i = 1:size(W,2)
            delta_q(:,i) = rot2quat(X_w(:,i),delta_t);
            Y_q(:,i) = qMul(X_q(:,i),delta_q(:,i));
            Y_w(:,i) = X_w(:,i);
            Y_q(:,i) = bsxfun(@rdivide, Y_q(:,i),sqrt(sum( Y_q(:,i).^2)));
        end
        
        %% Mean %%
        [x_q_bar, error_vector] = igd(Y_q,x_q_bar,1e-5,100);
        x_w_bar = mean(Y_w,2);
        x_q_bar = bsxfun(@rdivide, x_q_bar,sqrt(sum(x_q_bar.^2)));
        
        %% Covariance %%
        Wdash = [error_vector;bsxfun(@minus,Y_w,x_w_bar)];
        P_y = covariance(Wdash,Wdash);
        
        %% Measurement model  %%
        g = [0;0;0;1];
        for i = 1:size(W,2)
            Z_q(:,i) = qMul(qMul(qInv(Y_q(:,i)),g),Y_q(:,i));
            Z_q(:,i) = bsxfun(@rdivide, Z_q(:,i),sqrt(sum( Z_q(:,i).^2)));
            Z_w(:,i) = Y_w(:,i);
        end
        
        %% Mean %%
        for i = 1:2*n
            Z_q_v(:,i) = quat2rot(Z_q(:,i));
        end
        z_q_bar = mean(Z_q_v,2);
        z_w_bar = mean(Z_w,2);
        
        %% Covariance %%
        phi = bsxfun(@minus,[Z_q_v;Z_w],[z_q_bar;z_w_bar]);
        P_zz = covariance(phi,phi);
        
        %% innovation term %%
        v_q = acc(:,t)-z_q_bar;
        v_w = bsxfun(@minus,gyro(:,t),z_w_bar);
        
        P_vv = P_zz + R;
        P_xz = covariance(Wdash,phi);
        
        %% Kalman gain  %%
        K = P_xz/P_vv;
        temp = K * [v_q;v_w];
        temp_q = rot2quat(temp(1:3),1);
        q_x = qMul(x_q_bar,temp_q);
        w_x = x_w_bar + temp(4:end);
        P = P_y - K*P_vv*K';
        saved_q(:,t) = q_x;
        orientation(:,:,t) = quat2rotm(saved_q(:,t)');
        
        if rotation_plot
            figure(fig1),
            
            if vicon_data
                subplot(1,2,1)
                rotplot(orientation(:,:,t));
                title('UKF');
                [~, index] = min(abs(imu.ts(t)-vicon.ts));
                R_vicon = vicon.rots(:,:,index);
                subplot(1,2,2)
                rotplot(R_vicon);
                title('Vicon');
                drawnow;
            else
                rotplot(orientation(:,:,t));
                title('UKF');
            end
            hold off
        end
    end
    
    imu_angles = rotm2eul(quat2rotm(saved_q'));
    
    figure,
    subplot(3,1,1)
    hold on
    if vicon_data
        plot(vicon.ts,vicon_angle(:,1),'.');
    end
    plot(imu.ts,imu_angles(:,1),'.');
    legend('Vicon','UKF')
    ylabel('Yaw')
    hold off
    
    subplot(3,1,2)
    hold on
    
    if vicon_data
        plot(vicon.ts,vicon_angle(:,2),'.');
    end
    plot(imu.ts,imu_angles(:,2),'.');
    legend('Vicon','UKF')
    ylabel('Pitch')
    hold off
    
    subplot(3,1,3)
    hold on
    if vicon_data
        plot(vicon.ts,vicon_angle(:,3),'.');
    end
    plot(imu.ts,imu_angles(:,3),'.');
    legend('Vicon','UKF')
    ylabel('Roll')
    hold off
    drawnow;
    
    if stitch_image
        stitch(datapath,orientation,j,imu.ts);
    end
end