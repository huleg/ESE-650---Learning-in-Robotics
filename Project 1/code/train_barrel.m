function model = train_barrel(train_set,masks_set)

    dirstruct = dir(strcat(train_set,'*.png'));
    R = [];
    G = [];
    B = [];
    disp('Reading Images');
    for i = 1:length(dirstruct)
        % Read one test image at a time
        original = im2double(imread(strcat(train_set,dirstruct(i).name)));
        
        % Custom color space
        im2 = rgb2hsv(original);
        im3 = rgb2ycbcr(original);
        im = im3(:,:,3);
        im(:,:,2:3) = im2(:,:,2:3);
        
        % Load the handlabbeled masks
        BW = im2double(imread(strcat(masks_set,dirstruct(i).name)));
        
        % Perform some morphological operations in the mask and extract
        % dimention features of the barrel
        se = strel('disk',1);
        new_mask = imerode(BW,se);
        new_mask = imerode(new_mask,se);
        se = strel('disk',10);
        new_mask = imdilate(new_mask,se);
        se = strel('disk',7);
        new_mask = imerode(new_mask,se);
        s = regionprops(new_mask,'all');
        %centroids(i) = cat(1, s.Centroid);
        height(i) = cat(1, s.MajorAxisLength);
        width(i) = cat(1, s.MinorAxisLength);
        %areabb(i) = cat(1, s.Area);
        str = dirstruct(i).name;
        str(strfind(str, '.'):end) = [];
        distance(i) = str2num(str);
        
        % Extract RGB values of barrel pixels from all training images
        maskedRgbImage = bsxfun(@times,im,cast(BW,class(im)));
        r = maskedRgbImage(:,:,1);
        g = maskedRgbImage(:,:,2);
        b = maskedRgbImage(:,:,3);
        R = [R;r(BW>0)];
        G = [G;g(BW>0)];
        B = [B;b(BW>0)];
    end
    
    % Number of clusters for GMM
    clusters = 7;

    data = [R G B];
    channels = 3;
    
    % Initialize mu, sigma and pi for EM
    mu = rand(1,channels,clusters);
    oldmu = zeros(1,channels,clusters);
    a = rand(3,3);
    sigma = a'*a;
    A=arrayfun(@(i) (inv(sigma(:,:,i))) ,1:size(sigma,3),'UniformOutput',false);
    inv_sigma = cat(3, A{:});
    prior = repmat([1/clusters],[1 1 clusters]);
    repdata = repmat(data,[1 1 clusters]);

    % Perform EM for determing ellipsodal GMM
    disp('Performing EM');
    for t = 1:200
        disp(strcat('Iteration No: ',num2str(t)));
        
        % E step - compute alpha
        meancentered = bsxfun(@minus,repdata,mu);
        val = meancentered(:,1,:).*sum(bsxfun(@times, meancentered, permute(inv_sigma(:,1,:), [2 1 3])), 2)+...
            meancentered(:,2,:).*sum(bsxfun(@times, meancentered, permute(inv_sigma(:,2,:), [2 1 3])), 2)+...
            meancentered(:,3,:).*sum(bsxfun(@times, meancentered, permute(inv_sigma(:,3,:), [2 1 3])), 2);
        det_sigma = reshape(arrayfun(@(w) det(sigma(:, :, w)), 1 : size(sigma, 3)),[1 1 size(sigma,3)]);
        temp2 = bsxfun(@times,1/sqrt(det_sigma*(2*pi)^3),exp(-0.5*val));
        temp3 = bsxfun(@times,prior,temp2);
        alpha = bsxfun(@rdivide,temp3,sum(temp3,3));

        % M step - recompute mu, sigma and pi with new alpha
        repalpha = repmat(alpha,[1,channels,1]);
        % Compute mu
        mu = sum(repalpha.*repdata,1)./sum(repalpha,1);
        % Compute Pi
        prior = sum(alpha,1)/size(alpha,1);
        meancentered = bsxfun(@minus,repdata,mu);
        % Compute Sigma
        for i = 1:clusters
            temp = reshape(meancentered(:,:,i)',[1 channels size(meancentered,1)]);
            temp2 = permute(temp,[2 1 3]);
            temp3 = bsxfun(@times,temp2,temp);
            reshapealpha = reshape(alpha(:,:,i),[1 1 size(alpha,1)]);
            sigma(:,:,i) = sum(bsxfun(@times,reshapealpha,temp3),3)/sum(reshapealpha);
            sigma(sigma<eps)=eps;
        end
        % Compute inverse of sigma
        A=arrayfun(@(i) (inv(sigma(:,:,i))) ,1:size(sigma,3),'UniformOutput',false);
        inv_sigma = cat(3, A{:});
        norm(squeeze(oldmu-mu))
        if norm(squeeze(oldmu-mu))<1e-04
            break;
        end
        oldmu = mu;
    end
    
    % generate barrel model
    aspectRatio = height./width;
    model.meanAR = mean(aspectRatio);
    model.stdAR = std(aspectRatio);
    model.distCoeff = regress((1./distance)',[ones(size(width))' width']);
    model.mu = mu;
    model.sigma = sigma;
    model.inv_sigma = inv_sigma;
    save('model.mat','model');