clc
clear all
close all

addpath('./matlab');

% for drive model, change to 'car' and for walking use 'walk'
mode = 'walk';

map_thresh = 500;
norm_cost_prev = Inf;

load(['Path_',mode,'.mat']);
img = im2double(imread('aerial_color_resize.jpg'));
load(['features_',mode,'.mat']);

d = size(f,3);
% w = ones(1,d);
w =[-0.0265, 0.7566, 0.0434,-0.0098,-0.0042, 0.0849, 0.1641, 0.3110, 0.1626,-0.2960,0.0695, 0.1923,-0.0449,-0.0907,-0.0725, 0.7939,-0.0803,-0.1503,-0.0476,-0.1328,-0.1068, 0.5737,-0.1326,-0.5736]
eta = 1e-4;

costs = exp( sum( bsxfun( @times, f, reshape(w,[1 1 length(w)]) ), 3 ) );
for ite = 1:100
    disp(['Iteration: ',num2str(ite)]);
    imshow(img);
    for j = 1:length(Path)

        disp(['path',num2str(j)]);
        linearInd1 = sub2ind(size(costs), Path{j}(:,2), Path{j}(:,1));
        goal = floor(PathPoints{j}(end,:));
        start = floor(PathPoints{j}(1,:));
        cropy = [max(min(Path{j}(:,2))-map_thresh,1),min(max(Path{j}(:,2))+map_thresh,size(img,1))];
        cropx = [max(min(Path{j}(:,1))-map_thresh,1),min(max(Path{j}(:,1))+map_thresh,size(img,2))];
        temp_map = costs(cropy(1):cropy(2),cropx(1):cropx(2));
        ctg = dijkstra_matrix(temp_map,floor(goal(2)-cropy(1)),floor(goal(1)-cropx(1)));
        [ip1, jp1] = dijkstra_path(ctg, temp_map, floor(start(2)-cropy(1)), floor(start(1)-cropx(1)));
        ip1 = ip1 + cropy(1);
        ip1(ip1>size(costs,1)) = size(costs,1);
        jp1 = jp1 + cropx(1);
        jp1(jp1>size(costs,2)) = size(costs,2);
        linearInd2 = sub2ind(size(costs), ip1, jp1);
        for i = 1:d
            temp = f(:,:,i);
            gradient(j,i) = sum(temp(linearInd1).*costs(linearInd1)) - sum(temp(linearInd2).*costs(linearInd2));
        end
        hold on
        plot(Path{j}(:,1),Path{j}(:,2),'r.')
        plot(jp1,ip1,'g.')
        hold off
        drawnow;
    end
    
    norm_cost = norm(sum(gradient,1));
    if abs(norm_cost-norm_cost_prev) < 0.1
        disp('converged');
        break;
    else
        w = w - eta.*sum(gradient,1)
        costs = exp( sum( bsxfun( @times, f, reshape(w,[1 1 length(w)]) ), 3 ) );
        norm_cost_prev = norm_cost;
    end
end