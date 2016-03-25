clc
clear all
close all

texture_map = 1;

addpath('./Test/rgbd');
load('test1.mat');

if texture_map == 1
    load('DEPTH.mat');
    
    for t = 1:numel(DEPTH)
        [c,img_index(t)] = min(abs(ts_lidar-DEPTH{t}.t));
    end
    img_index = round(img_index/10)*10;
end


theta = deg2rad(linspace(-135,135,1081));
h = 1.26;
particles = 100;
motion_noise = [0.05;0.05;deg2rad(3)];
sat_ther = 100;
previous_pose = pose(1,:);
neff_thresh = 40;
fig1 = figure('units','normalized','outerposition',[0 0 1 1]);

fc_rgb = [ 1049.331752604831308 ; 1051.318476285322504 ];
fc_depth = [ 364.457362485643273 ; 364.542810626989194 ];
vid1 = VideoWriter('test1.avi');
open(vid1);
vid2 = VideoWriter('test2.avi');
open(vid2);

%% init LOCAL MAP
LMAP.res   = 0.1; %meters
LMAP.xmin  = -30;  %meters
LMAP.ymin  = -30;
LMAP.xmax  =  30;
LMAP.ymax  =  30;
LMAP.sizex  = ceil((LMAP.xmax - LMAP.xmin) / LMAP.res + 1); %cells
LMAP.sizey  = ceil((LMAP.ymax - LMAP.ymin) / LMAP.res + 1);
LMAP.map = zeros(LMAP.sizex,LMAP.sizey,'int8');

%% init GLOBAL MAP
GMAP.res   = 0.1; %meters
GMAP.xmin  = -50;  %meters
GMAP.ymin  = -50;
GMAP.xmax  =  50;
GMAP.ymax  =  50;
GMAP.sizex  = ceil((GMAP.xmax - GMAP.xmin) / GMAP.res + 1); %cells
GMAP.sizey  = ceil((GMAP.ymax - GMAP.ymin) / GMAP.res + 1);
GMAP.map = zeros(GMAP.sizex,GMAP.sizey,'int8');
GMAP.rgb = zeros(GMAP.sizex,GMAP.sizey,3,'double');

%% Inittialize log odds
occ = 0.9;
free = 0.9;
logodd_occ = 2.5;%log(occ/(1-free));
logodd_free = 2.5/4;%log(free/(1-occ));

%% First scan to initialize maps
%% Time sync
[c index] = min(abs(ts_joint-ts_lidar(1)));
ha(1,:) = head_angles(:,index)';

%% Ground hit removal
z = bsxfun(@times, scan(1,:).*cos(theta),sin(ha(1,2)));
h_corrected(1) = h + 0.15*cos(ha(1,2));
scan(1,:) = scan(1,:).*~(z>=h_corrected(1));

%% Convert to cartesian
indValid = (scan(1,:) < 30) & (scan(1,:) > 0.1);
temp_scan = scan(1,indValid);
temp_theta = theta(indValid);
[scan_x,scan_y] = pol2cart(temp_theta,temp_scan);

%% Skew correction
scan_x = bsxfun(@times,scan_x,cos(ha(1,2)));

%% Convert to cells
xis = double(ceil((scan_x - 0) ./ GMAP.res ));
yis = double(ceil((scan_y - 0) ./ GMAP.res ));

[x,y] = getMapCellsFromRay(zeros(1,length(xis)),zeros(1,length(xis)),xis,yis);

linearInd = sub2ind([GMAP.sizex GMAP.sizey], y+ceil(GMAP.sizey/2), x+ceil(GMAP.sizex/2));
GMAP.map(linearInd) = -logodd_free; %% free

linearInd = sub2ind([GMAP.sizex GMAP.sizey], yis+ceil(GMAP.sizey/2), xis+ceil(GMAP.sizex/2));
GMAP.map(linearInd) = logodd_occ; %% wall

pose_particle = zeros(3,particles)';
w = ones(1,particles) .* (1/particles);
files = 1;
loaded = [];
for i = 10:10:length(scan(:,1))
    %% Load RGB data
    if texture_map
        if mod(files,300)==1
            if(~any(floor((files/300)+1) == loaded))
                rgb_file = sprintf('RGB_%d.mat',int32(files/300)+1);
                disp(strcat('loading ',rgb_file,'....'))
                load(rgb_file)
                loaded = [loaded int32(files/300)+1];
                
            end
        end
    end
    
    
    LMAP.map = zeros(LMAP.sizex,LMAP.sizey,'int8');
    %% Time sync
    [c index] = min(abs(ts_joint-ts_lidar(i)));
    ha(i,:) = head_angles(:,index)';
    
    %% Ground hit removal
    z = bsxfun(@times, scan(i,:).*cos(theta),sin(ha(i,2)));
    h_corrected(i) = h + 0.15*cos(ha(i,2));
    scan(i,:) = scan(i,:).*~(z>=h_corrected(i));
    
    %% Convert to cartesian
    indValid = (scan(i,:) < 30) & (scan(i,:) > 0.1);
    temp_scan = scan(i,indValid);
    temp_theta = theta(indValid);
    [scan_x,scan_y] = pol2cart(temp_theta,temp_scan);
    
    %% Skew correction
    scan_x = bsxfun(@times,scan_x,cos(ha(i,2)));
    
    %% Motion update
    delta_pose = pose(i,:) - previous_pose;
    
    yaw_wrong =  previous_pose(3);
    R = [cos(yaw_wrong) sin(yaw_wrong); -sin(yaw_wrong) cos(yaw_wrong)];
    local_odom = R*delta_pose(1:2)';
    local_odom(3) = delta_pose(3);
    
    previous_pose = pose(i,:);
    
    %% Align local map
    %% yaw correction
    for j = 1:particles
        %% Motion update
        ang = pose_particle(j,3);
        R = [cos(ang) -sin(ang); sin(ang) cos(ang)];
        global_odom(1:2) = R* local_odom(1:2);
        global_odom(3) = local_odom(3);
        m_noise = (rand(3,1)-0.5).*motion_noise;
        pose_particle(j,:) = pose_particle(j,:) + global_odom + m_noise';
        
        ang = pose_particle(j,3) + ha(i,1);
        R = [cos(ang) -sin(ang); sin(ang) cos(ang)];
        scan_new = R*[scan_x;scan_y];
        
        x_im = GMAP.xmin:GMAP.res:GMAP.xmax; %x-positions of each pixel of the map
        y_im = GMAP.ymin:GMAP.res:GMAP.ymax; %y-positions of each pixel of the map
        Y = [scan_new(2,:)+pose_particle(j,2);scan_new(1,:)+pose_particle(j,1);zeros(size(scan_new(1,:)))];
        
        c(j) = map_correlation(GMAP.map,x_im,y_im,double(Y),0,0)+1e-5;
    end
    
    %% Normalize particle weights
    c = c - min(c);
    
    %% Update weights
    w = w .* c;
    w = w/sum(w);
    
    %% Choose best particle
    [~,best_particle] = max(w);
    
    %% Rotate scans
    ang = pose_particle(best_particle,3) + ha(i,1);
    R = [cos(ang) -sin(ang); sin(ang) cos(ang)];
    scan_new = R*[scan_x;scan_y];
    
    %% Translate scans
    
    %% Convert to cells
    xis = double(ceil(scan_new(1,:) ./ LMAP.res ));
    yis = double(ceil(scan_new(2,:) ./ LMAP.res ));
    
    %% create local map coordinates
    [x,y] = getMapCellsFromRay(0,0,xis,yis);
    
    %% Create occupancy grid
    linearInd = sub2ind([LMAP.sizex LMAP.sizey], y+ceil(LMAP.sizey/2), x+ceil(LMAP.sizex/2));
    LMAP.map(linearInd) = -logodd_free; %% free
    
    linearInd = sub2ind([LMAP.sizex LMAP.sizey], yis+ceil(LMAP.sizey/2), xis+ceil(LMAP.sizex/2));
    LMAP.map(linearInd) = logodd_occ; %% wall
    
    %% pose correction
    % center of canvas coordinates
    xindex = ceil(GMAP.sizex/2 - LMAP.sizex/2):ceil(GMAP.sizex/2 + LMAP.sizex/2)-1;
    yindex = ceil(GMAP.sizey/2 - LMAP.sizey/2):ceil(GMAP.sizey/2 + LMAP.sizey/2)-1;
    xindex = xindex + ceil(pose_particle(best_particle,1)/GMAP.res);
    yindex = yindex + ceil(pose_particle(best_particle,2)/GMAP.res);
    
    %% Update Global Map
    GMAP.map(yindex, xindex) = GMAP.map(yindex, xindex)+LMAP.map;
    GMAP.map(GMAP.map>sat_ther) = sat_ther;
    GMAP.map(GMAP.map<-sat_ther) = -sat_ther;
    
    %% Generate image
    if texture_map
        if any(i==img_index)
            img_no = find(img_index==i);
            temp = mod(img_no(1),300);
            if temp == 0
                temp = 300;
            end
            map = rgb_map(GMAP,DEPTH{img_no(1)}.depth,RGB{temp}.image,pose_particle(best_particle,:),fc_rgb,fc_depth,ha(i,:));
            GMAP.rgb = map;
            files = files + 1;
        end
    end
    
    bestpose(i,:) = pose_particle(best_particle,:);
    
    figure(fig1),
    subplot(2,2,[1 3])
    imagesc((-GMAP.map));
    axis square;
    colormap gray;
    title(i);
    hold on
    plot(xis+GMAP.sizex/2+pose_particle(best_particle,1)/GMAP.res,yis+GMAP.sizey/2+pose_particle(best_particle,2)/GMAP.res,'r.','Markersize',3);
    plot(GMAP.sizex/2+bestpose(:,1)/GMAP.res,GMAP.sizey/2+bestpose(:,2)/GMAP.res,'g.','Markersize',0.05);
%     set(hfig, 'Markersize','0.1');
    hold off
    axis xy
    zoom(2);
    frame = getframe;
    writeVideo(vid1,frame);
    subplot(2,2,2)
    imshow(GMAP.rgb);
    axis xy
    axis square;
    zoom(2);
    frame = getframe;
    writeVideo(vid2,frame);
    subplot(2,2,4)
    quiver(pose_particle(:,1),pose_particle(:,2),cos(pose_particle(:,3)).*w',sin(pose_particle(:,3)).*w');
    hold on
    plot(pose_particle(best_particle,1),pose_particle(best_particle,2),'r*')
    hold off

    %% Resample particles
    neff = sum(w)^2/sum(w.^2);
    if neff<neff_thresh
        cumulative_sum = cumsum(w);
        for k = 1 : particles
            pose_particle(k,:) = pose_particle(find(rand <= cumulative_sum,1),:);
        end
        w = ones(1,particles) .* (1/particles);
    end
    drawnow;
end

close(vid1);
close(vid2);

if texture_map
    
    linearInd1 = find(GMAP.map>sat_ther/2);
    linearInd2 = find(GMAP.map<sat_ther/2 & GMAP.map>-sat_ther/2);
    linearInd3 = find(GMAP.map<-sat_ther/2);
    mapr = GMAP.rgb(:,:,1);
    mapr(linearInd1) = 1;
    mapr(linearInd2) = 0.5059;
    map(:,:,1) = mapr;
    
    mapg = GMAP.rgb(:,:,2);
    mapg(linearInd1) = 1;
    mapg(linearInd2) = 0.5059;
    map(:,:,2) = mapg;
    
    mapb = GMAP.rgb(:,:,3);
    mapb(linearInd1) = 1;
    mapb(linearInd2) = 0.5059;
    map(:,:,3) = mapb;
    
    % GMAP.rgb = map;
    
    figure,
    imshow(map)
    axis xy
    colormap gray
    axis square
    imwrite(double(map),'tm.jpeg')
    zoom(2.5);
    pause;
end

figure,
imagesc(-GMAP.map)
axis xy
colormap gray
axis square
zoom(2.5);