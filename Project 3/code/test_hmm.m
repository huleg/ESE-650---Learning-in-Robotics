function [predictions,confidence] = test_hmm(path,gestures,model)

testDir = dir(strcat(path,'*.txt'));
predictions = zeros(length(testDir),1);
confidence = zeros(length(testDir),1);
for test = 1:length(testDir)
    data = load(strcat(path,testDir(test).name));
    data(:,1) = [];
    [~,gestureData] = min(pdist2(data,model.C),[],2);
    
    for class = 1:size(model.A,3)
        A = model.A(:,:,class);
        B = model.B(:,:,class);
        Pi = model.Pi(:,class);
        alpha = [];
        norm_alpha = [];
        alpha = Pi.*B(:,gestureData(1));
        norm_alpha(:,1) = sum(alpha(:,1));
        alpha = alpha/norm_alpha(:,1);
        
        for t = 1:length(gestureData)-1
            alpha(:,t+1) = sum(bsxfun(@times,alpha(:,t)',A),2).*B(:,gestureData(t+1));
            norm_alpha(:,t+1) = sum(alpha(:,t+1));
            alpha(:,t+1) = alpha(:,t+1)/norm_alpha(:,t+1);
        end
        P(:,class) = -1./sum(log(norm_alpha));
    end
    P = P./max(P);
    temp = sort(P,'descend');
    confidence(test) = temp(1) - temp(2);
    [~,d] = max(P);
    figure,
    bar(P)
    set(gca,'XTickLabel',['beat3 ';'beat4 ';'circle';'eight ';'inf   ';'wave  '])
    title(['File: ',char(testDir(test).name),', Result:',char(gestures(d)),', Confidence: ',num2str(confidence(test))]);
    disp(['Testing file:',testDir(test).name,', Result:',char(gestures(d)),', Confidence: ',num2str(confidence(test))])
    predictions(test) = d;
end