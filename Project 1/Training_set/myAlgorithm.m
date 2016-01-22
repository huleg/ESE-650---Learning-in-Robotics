clc
clear all
close all

dirstruct = dir('train/*.png');
masks = dir('masks_paril/*.png');
% sumr = 0;
% sumg = 0;
% sumb = 0;
nnzpixels = 0;
totalpixels = 0;
R = [];
G = [];
B = [];
Rnb = [];
Gnb = [];
Bnb = [];
for i = 1:length(dirstruct)
	% Read one test image 
	im = im2double(imread(strcat('train/',dirstruct(i).name)));
    %ycbcrmap = rgb2ycbcr(maskedRgbImage);
    BW = im2double(imread(strcat('masks/',dirstruct(i).name)));
    
    maskedRgbImage = bsxfun(@times,im,cast(BW,class(im)));
    nnzpixels = nnzpixels + nnz(BW);
    totalpixels = totalpixels + numel(BW);
    r = maskedRgbImage(:,:,1);
    g = maskedRgbImage(:,:,2);
    b = maskedRgbImage(:,:,3);
    R = [R;r(BW>0)];
    G = [G;g(BW>0)];
    B = [B;b(BW>0)];
    maskedRgbImage = bsxfun(@times,im,cast(~BW,class(im)));
    r = maskedRgbImage(:,:,1);
    g = maskedRgbImage(:,:,2);
    b = maskedRgbImage(:,:,3);
    Rnb = [Rnb;r(BW==0)];
    Gnb = [Gnb;g(BW==0)];
    Bnb = [Bnb;b(BW==0)];
    %ycbcrmap = rgb2ycbcr(maskedRgbImage);
    % Your computations here!
end
% meanr = sumr/nnzpixels
% meang = sumg/nnzpixels
% meanb = sumb/nnzpixels
C = cov([R G B]);
A = inv(C)
mu = [mean(R) mean(G) mean(B)]
Pbarrel = nnzpixels/totalpixels;

Cnb = cov([Rnb Gnb Bnb]);
Anb = inv(Cnb)
munb = [mean(Rnb) mean(Gnb) mean(Bnb)]

dirstruct = dir('test/*.png');

for i = length(dirstruct):-1:1,
	im = im2double(imread(strcat('test/',dirstruct(i).name)));
    new_im = reshape(im,[900*1200 3]);
    %%%%%%%%% Barrel color probability P(X|Color)  %%%%%%%%%%%
    temp = bsxfun(@minus,new_im,mu);
    val = temp(:,1).*(temp*A(:,1))+temp(:,2).*(temp*A(:,2))+temp(:,3).*(temp*A(:,3));
    P = sqrt(det(A)/(2*pi)^3).*exp(-0.5*val);
    
    %%%%%%%%% Non Barrel color probability P(X|Others)  %%%%%%%%%%%
    temp = bsxfun(@minus,new_im,munb);
    val = temp(:,1).*(temp*Anb(:,1))+temp(:,2).*(temp*Anb(:,2))+temp(:,3).*(temp*Anb(:,3));
    Pnb = sqrt(det(Anb)/(2*pi)^3).*exp(-0.5*val);
    
    Out = P.*Pbarrel./(P.*Pbarrel+Pnb.*(1-Pbarrel));
    Outdash = Pnb.*(1-Pbarrel)./(P.*Pbarrel+Pnb.*(1-Pbarrel));
    new_mask = reshape((Out>Outdash),[900 1200]);
    imagesc(new_mask)
    pause;
end