clc
clear all
close all

dirstruct = dir('train/*.png');
masks = dir('masks_paril/*.png');
nnzpixels = 0;
R = [];
G = [];
B = [];
for i = 1:length(dirstruct)
	% Read one test image 
	im = im2double(imread(strcat('train/',dirstruct(i).name)));
    %ycbcrmap = rgb2ycbcr(maskedRgbImage);
    BW = im2double(imread(strcat('masks/',dirstruct(i).name)));
    
    maskedRgbImage = bsxfun(@times,im,cast(BW,class(im)));
    nnzpixels = nnzpixels + nnz(BW);
    totalpixels = totalpixels + numel(BW)
    r = maskedRgbImage(:,:,1);
    g = maskedRgbImage(:,:,2);
    b = maskedRgbImage(:,:,3);
    R = [R;r(BW>0)];
    G = [G;g(BW>0)];
    B = [B;b(BW>0)];
    %ycbcrmap = rgb2ycbcr(maskedRgbImage);
    % Your computations here!
end
% meanr = sumr/nnzpixels
% meang = sumg/nnzpixels
% meanb = sumb/nnzpixels
meanr = mean(R);
meang = mean(G);
meanb = mean(B);
C = cov([R G B]);
A = inv(C)
mu = [meanr meang meanb]

dirstruct = dir('test/*.png');

for i = length(dirstruct):-1:1,
	im = im2double(imread(strcat('test/',dirstruct(i).name)));
    new_im = reshape(im,[900*1200 3]);
    temp = bsxfun(@minus,new_im,mu);
    val = temp(:,1).*(temp*A(:,1))+temp(:,2).*(temp*A(:,2))+temp(:,3).*(temp*A(:,3));
    P = sqrt(det(A)/(2*pi)^3).*exp(-0.5*val);
    new_mask = P>5;
    new_mask = reshape(P,[900 1200]);
    imagesc(new_mask)
    pause;
end