function P = covariance(W,Wdash)
    temp = zeros([size(W,1),size(Wdash,1),size(W,2)]);
    for i = 1:size(W,2)
        temp(:,:,i) = W(:,i)*Wdash(:,i)';
    end
    P = mean(temp,3);
        