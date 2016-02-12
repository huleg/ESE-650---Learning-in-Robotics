function q = rot2quat(r,delta_t)
    alpha = norm(r)*delta_t;
    if alpha == 0
        e = [0;0;0];
    else
        e = r./norm(r);
    end
    q = [cos(alpha/2); e*sin(alpha/2)];