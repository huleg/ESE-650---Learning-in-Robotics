function model = train_hmm(path,gestures,N,M)

    %% Load the data %%
    disp('Loading the data')
    data = [];
    for p = 1:length(gestures)
        trainDir = dir(strcat(path,char(gestures(p)),'*.txt'));
        for h = 1:length(trainDir)
            gesturesize((p-1)*length(trainDir)+h) = length(data);
            data = [data;load(strcat(path,trainDir(h).name))];
        end
    end
    gesturesize(end+1) = length(data);
    data(:,1) = [];
    
    %% Cluster all the data
    disp('Performing K-means')
    [idx,C] = kmeans(data,M,'MaxIter',500);

    %% Train the model for each gesture
    disp('Training the models')
    for p = 1:length(gestures)
        trainDir = dir(strcat(path,char(gestures(p)),'*.txt'));
        tempA = [];
        tempB = [];
        tempPi = [];

        for h = 1:length(trainDir)
            disp(strcat('Trainng Dataset: ',num2str(h),' of gesture: ',num2str(p)));
            index = (p-1)*length(trainDir)+h;
            gestureData = idx(gesturesize(index)+1:gesturesize(index+1));

            Pi = rand(N,1);
            Pi = Pi./sum(Pi);
            A = rand(N,N);
            A = bsxfun(@rdivide,A,sum(A,1));
            B = rand(N,M);
            B = bsxfun(@rdivide,B,sum(B,2));

            alpha = [];
            norm_alpha = [];
            norm_beta = [];
            beta = [];
            gamma = [];
            Xi = [];

            for k = 1:10
                alpha = Pi.*B(:,gestureData(1));
                norm_alpha(:,1) = sum(alpha(:,1));
                alpha = alpha/norm_alpha(:,1);

                beta = ones(N,length(gestureData));
                beta = beta/size(beta,1);
                
                %% Calculate alpha from 1 to T-1
                for t = 1:length(gestureData)-1
                    alpha(:,t+1) = sum(bsxfun(@times,alpha(:,t)',A),2).*B(:,gestureData(t+1));
                    norm_alpha(:,t+1) = sum(alpha(:,t+1));
                    alpha(:,t+1) = alpha(:,t+1)/norm_alpha(:,t+1);

                end
                
                %% Calculate beta from T-1 to 1
                for t = length(gestureData)-1:-1:1
                    beta(:,t) = sum(bsxfun(@times,bsxfun(@times,B(:,gestureData(t+1)),A),beta(:,t+1)))';
                    norm_beta(:,t) = sum(beta(:,t));
                    beta(:,t) = beta(:,t)/norm_beta(:,t);
                end
                
                %% Calculate gamma
                gamma = bsxfun(@rdivide,alpha.*beta,sum(alpha.*beta));
                
                %% Calculate Xi
                Xi = zeros(N,N,size(gamma,2));
                for t = 1:length(gestureData)-1
                    Xi(:,:,t) = bsxfun(@times,bsxfun(@times,alpha(:,t)',A),(B(:,gestureData(t+1)).*beta(:,t+1)));
                    Xi(:,:,t) = Xi(:,:,t)/sum(sum(Xi(:,:,t)));
                end
                
                %% Maximization step
                Pi = gamma(:,1);
                A = bsxfun(@rdivide,sum(Xi,3),sum(gamma(:,1:end-1),2));
                obs = unique(idx);
                B = zeros(size(B));
                for i = 1:M
                    B(:,obs(i)) = sum(gamma(:,gestureData==i),2)./sum(gamma,2);
                end
                B(B<1e-10) = 1e-10;
            end
            %% Store model for each dataset of each gesture
            tempA(:,:,h) = A;
            tempB(:,:,h) = B;
            tempPi(:,h) = Pi;
        end
        %% Store model for each gesture
        A_class(:,:,p) = mean(tempA,3);
        B_class(:,:,p) = mean(tempB,3);
        Pi_class(:,p) = mean(tempPi,2);
    end

    %% Return the model
    model.A = A_class;
    model.B = B_class;
    model.Pi = Pi_class;
    model.C = C