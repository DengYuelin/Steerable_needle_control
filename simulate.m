function final_error = simulate(setup, const)
    % Constants
    dt = const.dt;
    MAXITER = const.MAXITER;

    % Random Setup
    lambda1 = 0.01; %m/s, equal to 1 cm/s
    lambda_ratio = setup.lambda_ratio; % in range [1 100]
    k = setup.k; % curvature value in range [1 20] m^-1
    goal = setup.goal; % random goal
    lambda2 = k * lambda1 * lambda_ratio;

    % initial state
    p0 = [0; 0; 0];
    eul0 = [0, 0, 0];
    R0 = eul2rotm(eul0);
    %% Check if goal satisfy initial condition
    % Verify goal position satisfy torus condition
    if ~checkReachability(R0, p0, goal, k)
        %         disp("Torus condition NOT satisfied!")
        final_error = NaN;
        return
    end

    et = getErrorVec(R0, p0, goal);

    if ~(et(1) == 0 && (et(2) >= -2 / k && et(2) <= 0) && (et(3) >= 0 && et(3) <= 2 / k))
        disp("Initial goal condition NOT satisfied!")
        return
    end

    %% start control simulation
    pt = p0;
    Rt = R0;

    for i = 1:MAXITER
        % Termination condition
        if et(3) <= 0
            break
        end

        % check error and manifold
        et = getErrorVec(Rt, pt, goal);
        [sigma, sigma_sign] = getManifold(et);
        % get control signal
        ut = getControl(lambda1, lambda2, sigma);
        % Simulate one step dynamics
        [Rnext, pnext] = dynamics_dt(Rt, pt, ut, k, dt);

        Rt = Rnext;
        pt = pnext;
    end

    final_error = norm(et);
end
