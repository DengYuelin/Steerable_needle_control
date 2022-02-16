function u = getControl(lambda1, lambda2, sigma, epsilon, traj)
    u = zeros(2, 1);

    if (nargin < 4)
        u(1) = lambda1;
        u(2) = lambda2 * sign(sigma);
        return
    elseif (nargin < 5)
        u(1) = lambda1;

        if abs(sigma) > epsilon
            u(2) = lambda2 * sign(sigma);
        else
            u(2) = lambda2 * sigma / epsilon;
        end

        return
    elseif (nargin < 6)
        u(1) = traj.pdot + traj.kp * traj.ez;

        if abs(sigma) > epsilon
            u(2) = lambda2 / lambda1 * sign(sigma) * norm(u(1));
        else
            u(2) = lambda2 / lambda1 * sigma / epsilon * norm(u(1));
        end

        return
    end

end
