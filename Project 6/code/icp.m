function [ R,T ] = icp( Mdata, rotatedMdata, type, iterations )

N = size(Mdata,2);
MCloud = pointCloud(Mdata');
allnormals = pcnormals(MCloud);

T = [0;0;0];
R = eye(3);
Xprev = inf;
Ttempprev = inf;
for i = 1:iterations
    if type == 'point'
        
        pDashIdx = knnsearch(Mdata',rotatedMdata');
        reducedp = Mdata(:,pDashIdx);
        p = sum(reducedp,2)/N;
        pdash = sum(rotatedMdata,2)/N;
        
        q = bsxfun(@minus,reducedp,p);
        qDash = bsxfun(@minus,rotatedMdata,pdash);
        
        H = q * qDash';
        [U,S,V] = svd(H);
        
        X = V*U';
        
        Ttemp = pdash - X*p;
        rotatedMdata = bsxfun(@minus,rotatedMdata,Ttemp);
        rotatedMdata = X'*rotatedMdata;
        R = X*R;
        T = R*(T + Ttemp);
    elseif type == 'plane'
        
        % Mdl = KDTreeSearcher(Mdata');
        % pDashIdx = knnsearch(Mdl,rotatedMdata');
        pDashIdx = knnsearch(Mdata',rotatedMdata');
        reducedp = Mdata(:,pDashIdx);
        normals = allnormals(pDashIdx,:);
        
        b = sum(normals.*rotatedMdata',2)-sum(normals.*reducedp',2);
        A = [normals(:,3).*reducedp(2,:)' - normals(:,2).*reducedp(3,:)', ...
            normals(:,1).*reducedp(3,:)' - normals(:,3).*reducedp(1,:)', ...
            normals(:,2).*reducedp(1,:)' - normals(:,1).*reducedp(2,:)', ...
            normals(:,1), normals(:,2), normals(:,3)];
        Adash = pinv(A);
        x = Adash*b;
        Ttemp = [x(4);x(5);x(6)];
        X = eul2rotm([x(3),x(2),x(1)]);
        
        rotatedMdata = bsxfun(@minus,rotatedMdata,Ttemp);
        rotatedMdata = X'*rotatedMdata;
%         H = [X;T]
        R = X*R;
        T = R*(T + Ttemp);
    end
%     subplot(1,2,1)
    pcshow(Mdata','r')
    hold on
    pcshow(rotatedMdata','g')
    hold off
    title(i)
%     xlabel('x'); ylabel('y'); zlabel('z')
%     direction = [0 0 1];
%     rotate(abc,direction,180)
%     saveas(gcf,strcat(num2str(i),'.jpg'));
%     subplot(1,2,2)
%     pcshow(rotatedMdata')
    drawnow;
%     abs(sum(sum(X)) - sum(sum(Xprev)))
%     abs(sum(Ttemp) - sum(Ttempprev))
    if((abs(sum(sum(X)) - sum(sum(Xprev))) < 1e-4) & (abs(sum(Ttemp) - sum(Ttempprev)))<1e-4)
        break;
    else
        Xprev = X;
        Ttempprev = Ttemp;
    end
    
end


