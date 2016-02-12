function state = stitch(path,rots,file,ts_imu)

filename = strcat(path,'cam/cam',num2str(file),'.mat');
cam = load(filename);

fig1 = figure;

f = 270;
img = im2double(cam.cam(:,:,:,1));
[X,Y] = meshgrid(1:size(img,2),1:size(img,1));
Xc = X(:) - size(X,2)/2;
Yc = Y(:) - size(Y,1)/2;
BP = [Xc(:)';Yc(:)';repmat(f,[1 size(X,1)*size(X,2)])];

canvas = im2double(zeros([1400 1580 3]));

for i = 200:10:size(cam.cam,4)-200
    [c index] = min(abs(ts_imu-cam.ts(i)));
    R(:,:,i) = rots(:,:,index);
    img = im2double(cam.cam(:,:,:,i));
    WP = R(:,:,i)*[0,0,1;-1,0,0;0,-1,0]*BP;
    theta = atan2(WP(2,:),WP(1,:));
    h = WP(3,:)./sqrt(WP(1,:).^2+WP(2,:).^2);
    new_cood = bsxfun(@plus,[-f.*theta;f.*h],[size(canvas,2)/2;size(canvas,1)/2]);
    new_cood(new_cood<0) = 1;
    for k = 1:size(Xc,1)
        if new_cood(2,k)> size(canvas,1) || new_cood(1,k) > size(canvas,2)
            continue;
        else
            canvas(ceil(new_cood(2,k)),ceil(new_cood(1,k)),:) = img(Y(k),X(k),:);
        end
    end
    
    figure(fig1),
    imshow(flipud(canvas));
    state = 1;
end