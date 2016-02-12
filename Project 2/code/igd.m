function [qbar e_vec] = igd(Y_q,qbar,thres,iter)
    for i = 1:iter
        for j = 1:size(Y_q,2)
            e(:,j) = qMul(Y_q(:,j),qInv(qbar));
            e_vec(:,j) = quat2rot(e(:,j));
        end
        mean_e = mean(e_vec,2);
        if norm(mean_e) < thres
            break;
        end
        qbar = qMul(rot2quat(mean_e,1),qbar);
    end
%     disp(strcat('Number of igd iterations: ',num2str(i)));