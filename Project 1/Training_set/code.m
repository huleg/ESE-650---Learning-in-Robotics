clc
clear all
close all

dirstruct = dir('*.png');
for i = length(dirstruct):-1:1,
	% Read one test image 
	im = imread(dirstruct(i).name);
    BW = roipoly(im);
    subplot(2,1,1)
    imshow(BW)
    subplot(2,1,2)
    imshow(im)
    v = strcat('masks/',dirstruct(i).name);
    imwrite(BW,v,'png');
    % Your computations here!
	%[x, y, d] = myAlgorithm(im);

	% Display results:
	% (1) Segmented image
	% (2) Barrel center 
	% (3) Distance of barrel 
	% You may also want to plot and display other diagnostic information 

    pause;
    close all
end