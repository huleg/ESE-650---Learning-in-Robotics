%% Run required parts
addpath('./features')
img = imread('aerial_color_resize.jpg');

%% For all files in feature folder
[BW,~] = pathmap(img);

BW = bwmorph(BW,'clean');
BW = bwmorph(BW,'clean');
BW = bwmorph(BW,'spur');
BW = bwmorph(BW,'spur');

se = strel('disk',1);
a = imerode(BW,se);
se = strel('disk',1);
a = imdilate(a,se);

CC = bwconncomp(a);
stats = regionprops(CC,'all');

f = zeros(size(BW));

for i = 1:length(CC.PixelIdxList)
    if stats(i).Area > 10
        if stats(i).Eccentricity<0.965 %0.87
            f(CC.PixelIdxList{i}) = 1;
        
        end
    end
end
imshow(f);
save('./features/pathmap.mat','f')

%% K means features
nrows = size(img,1);
ncols = size(img,2);
ab = reshape(img,nrows*ncols,3);
nColors = 16;
[cluster_idx, cluster_center] = kmeans(ab,nColors,'distance','sqEuclidean', 'Replicates',1);
pixel_labels = reshape(cluster_idx,nrows,ncols);

for i = 1:nColors
    feature = pixel_labels==i;
    feature = imgaussfilt(im2double(feature));
    f(:,:,i) = feature;
end

%% To combine all geatures into a single mat file
d = dir('./features/*.mat');
for i = 1:length(d)
    load(['./features/',d(i).name]);
    disp(d(i).name);
    f = imgaussfilt(im2double(f));
    feat(:,:,i) = f;
%     figure,
    imagesc(feat(:,:,i))
    pause;
end
save('feature.mat','f');
