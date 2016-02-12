function vec = quat2rot(q)
    alpha = atan2(norm(q(2:end)),q(1));
    if alpha == 0
        vec = [0;0;0];
    else
        vec = 2*alpha .* q(2:end)/norm(q(2:end));
    end
    