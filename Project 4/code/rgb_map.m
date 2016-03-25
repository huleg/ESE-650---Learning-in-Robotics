function map = rgb_map(GMAP,z,img,pose,fc_rgb,fc_depth,ha)

    map = zeros(size(GMAP.rgb));
    u = repmat(1:size(z,2),size(z,1),1);
    v = repmat((1:size(z,1))',1,size(z,2));
    u = u-max(max(u))/2;
    v = v-max(max(v))/2;
%     fig1 = figure;
%     fig2 = figure;
%     fig3 = figure;
    thres = 0.1;
    img = im2double(img);
    R = [0.999968551008760,0.005899814450952,0.005299922913182;-0.005894063933536,0.999982024861347,-0.001099983885351;-0.005306317347155,0.001068711207474,0.999985350318977];
    T = [52.268200000000000;1.519200000000000;-0.605900000000000];
    
    %% Convert to 3d points
    z(z>3000) = Inf;
    z(z<500) = Inf;
    h = 1260;
%     temp = abs((v./fc(1)) - (h./z));
    x = u.*z/fc_depth(1);
    y = v.*z/fc_depth(2);
    xPoints = x(:);
    yPoints = y(:);
    zPoints = z(:);
    
    %% Find ground plane points
    h_corrected = h + 70*cos(ha(2));
    coeff = eul2rotm([0,0,ha(2)])*[0;-1;0];
    val = sum(bsxfun(@times,[coeff;h_corrected],[xPoints';yPoints';zPoints';ones(numel(xPoints),1)']));
%     val = coeff(1)*x + coeff(2)*y + coeff(3)*z + h_corrected;
    GP(1,:) = x(val<thres);
    GP(2,:) = y(val<thres);
    GP(3,:) = z(val<thres);
    
    %% Find corresponding RGB points
    GP_rgb = bsxfun(@plus,R*GP,T);
    u_rgb = ceil(fc_rgb(1).*GP_rgb(1,:)./GP_rgb(3,:) + size(img,2)/2);
    v_rgb = ceil(fc_rgb(2).*GP_rgb(2,:)./GP_rgb(3,:) + size(img,1)/2);
    
    indValid = u_rgb<size(img,2) & u_rgb>0 & v_rgb<size(img,1) & v_rgb>0;
    u_rgb = u_rgb(indValid);
    v_rgb = v_rgb(indValid);
    GP = GP(:,indValid);
    GP_rgb = GP_rgb(:,indValid);
    
    %% Convert to lidar frame
    GP_lidar = [0,0,1;-1,0,0;0,-1,0]*GP;
    
    %% Convery xyz to map scale
    xindex = ceil(GP_lidar(1,:)/(1000*GMAP.res));
    yindex = ceil((GP_lidar(2,:))/(1000*GMAP.res));
    
    %% Rotate it based on pose
    ang = pose(3);
    index_new = [cos(ang), -sin(ang); sin(ang), cos(ang)]*[xindex;yindex];
    xindex = index_new(1,:);
    yindex = index_new(2,:);
    
    %% Translate based on pose
    xindex = ceil(xindex + pose(1)/GMAP.res + GMAP.sizex/2);
    yindex = ceil(yindex + pose(2)/GMAP.res + GMAP.sizey/2);
    
    linearInd1 = sub2ind(size(img(:,:,1)), v_rgb, u_rgb);
    linearInd2 = sub2ind(size(GMAP.rgb(:,:,1)), yindex, xindex);
    mapr = GMAP.rgb(:,:,1);
    imgr = img(:,:,1);
    mapr(ceil(linearInd2)) = imgr(linearInd1);
    map(:,:,1) = mapr;
    mapg = GMAP.rgb(:,:,2);
    imgg = img(:,:,2);
    mapg(ceil(linearInd2)) = imgg(linearInd1);
    map(:,:,2) = mapg;
    mapb = GMAP.rgb(:,:,3);
    imgb = img(:,:,3);
    mapb(ceil(linearInd2)) = imgb(linearInd1);
    map(:,:,3) = mapb;
    
%     size(find(map~=0))
%      figure,
%      imshow(map);
%     hold on
%     plot(u_rgb,v_rgb,'r.');
%     hold off
%     
%     figure(fig2)
%     pcshow([x(:),y(:),z(:)]);
%     hold on
%     pcshow(permute(GP,[3 2 1]),'r');
%     hold off
%     axis square;
%     xlabel('x');
%     ylabel('y');
%     zlabel('z');
%     
%     figure(fig3)
%     imshow(GMAP.rgb);
% %     plot(GP_lidar(1,:),GP_lidar(2,:),'.');
%     drawnow