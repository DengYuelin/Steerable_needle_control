function [Rnext, pnext] = dynamics_dt(R, p, u, k, dt)
    % Get one step discrete dynamics
    %       [Rnext, pnext] = dynamics_dt(R, p, u, k, dt)
    %   where:
    %   R is 3x3 rotation matrix at time t
    %   p is 3x1 position vector at time t in m
    %   u is 2x1 control input
    %   k is the curvature constant

    % Get dynamics
    v = [0; 0; u(1)];
    W = [0, -u(2), 0;
        u(2), 0, -k * u(1);
        0, k * u(1), 0];
    % Current homogeneous matrix in SE(3)
    g = [R, p;
        zeros(1, 3), 1];

    % Change rate in se(3)
    Vhat = [W, v;
        zeros(1, 3), 0];

    gnext = g * expm(Vhat * dt);
    Rnext = gnext(1:3, 1:3);
    pnext = gnext(1:3, 4);
end
