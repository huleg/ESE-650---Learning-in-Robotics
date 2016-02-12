function [x y d new_mask] = compute_distance(im,model)

    % Load model
    mu = model.mu;
    sigma = model.sigma;
    inv_sigma = model.inv_sigma;
    meanAR = model.meanAR;
    stdAR = model.stdAR;
    distCoeff = model.distCoeff;
    clusters = size(mu,3);
    x = 0; y = 0; d = 0;
    % Convert colorspace
    original = im2double(im);
    im2 = rgb2hsv(original);
    im3 = rgb2ycbcr(original);
    im = im3(:,:,3);
    im(:,:,2:3) = im2(:,:,2:3);
    new_im = reshape(im,[size(original,1)*size(original,2) 3]);
    repdata = repmat(new_im,[1 1 clusters]);

    % Find the probility of the pixels in the GMM
    meancentered = bsxfun(@minus,mu,repdata);
    val = meancentered(:,1,:).*sum(bsxfun(@times, meancentered, permute(inv_sigma(:,1,:), [2 1 3])), 2)+...
        meancentered(:,2,:).*sum(bsxfun(@times, meancentered, permute(inv_sigma(:,2,:), [2 1 3])), 2)+...
        meancentered(:,3,:).*sum(bsxfun(@times, meancentered, permute(inv_sigma(:,3,:), [2 1 3])), 2);
    det_sigma = reshape(arrayfun(@(w) det(sigma(:, :, w)), 1 : size(sigma, 3)),[1 1 size(sigma,3)]);
    new_val = bsxfun(@times,1/sqrt(det_sigma*(2*pi)^3),exp(-0.5*val));

    P = sum(new_val,3)/clusters;

    % Find the likelihood
    Pt = P>=0.01*max(max(P));
    Pt_reshape = reshape(Pt,[size(original,1) size(original,2)]);
    mask = Pt_reshape;

    % Perform morphological operations to remove noise
    se = strel('disk',1);
    new_mask = imerode(Pt_reshape,se);
    new_mask = imerode(new_mask,se);
    se = strel('disk',10);
    new_mask = imdilate(new_mask,se);
    se = strel('disk',7);
    new_mask = imerode(new_mask,se);

    % Calculate detected barrel properties
    s = regionprops(new_mask,'all');
    centroids = cat(1, s.Centroid);
    bb = cat(1, s.BoundingBox);
    height = cat(1, s.MajorAxisLength);
    width = cat(1, s.MinorAxisLength);
    aspect_ratio = height./width;
    area = cat(1, s.Area);
    %orientation = cat(1, s.Orientation);
    solidity = cat(1, s.Solidity);
    f = 0;
    if length(s)>1
        for j = 1:length(s),
            if aspect_ratio(j) > meanAR-3*stdAR && aspect_ratio(j) < meanAR+3*stdAR && area(j) > 1400
                f = f+1;
                if solidity(j)<0.8
                    x(f) = bb(j,1)+bb(j,3)/2;
                    y(f) = bb(j,2)+bb(j,4)/2;
                else
                    x(f) = centroids(j,1);
                    y(f) = centroids(j,2);
                end
                bounding_box(f,:) = [bb(j,1),bb(j,2),bb(j,3),bb(j,4)];
                d(f) = 1/(distCoeff(2)*width(j) + distCoeff(1));
            end
        end
    else
        if solidity(1)<0.8
            x = bb(1)+bb(3)/2;
            y = bb(2)+bb(4)/2;
        else
            x = centroids(1);
            y = centroids(2);
        end
        bounding_box = [bb(1),bb(2),bb(3),bb(4)];
        d = 1/(distCoeff(2)*width + distCoeff(1));
    end

    hold on
    if f > 1
        for i = 1:f
            rectangle('Position',bounding_box(i,:),'EdgeColor','g','LineWidth',1)
            plot(x(i),y(i),'g*')
        end
    else
        if x==0 && y == 0 && d == 0
            disp('No barrel found');
        else
            rectangle('Position',bounding_box,'EdgeColor','g','LineWidth',1)
            plot(x,y,'g*')
        end
        hold off
    end