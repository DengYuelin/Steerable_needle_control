clc, clear, close('all')
%%
% Constants
dt = 0.01;
MAXITER = 50000;
lambda1 = 0.01; %m/s, equal to 1 cm/s
k = 1;
lambda_ratio = 100;
lambda2 = k * lambda1 * lambda_ratio;

% initial state
p0 = [0; 0; 0];
eul0 = [0, 0, 0];
R0 = eul2rotm(eul0);

% goal position
goal = [-1; -2; 3];

% Time constant
const_q = 2 * pi / sqrt(k^2 * lambda1^2 + lambda2^2); % ignore sign because of square
qbar = sqrt(1 + lambda_ratio^2);

%% Verify goal position satisfy torus condition
if checkReachability(R0, p0, goal, k)
    disp("Torus condition satisfied!")
else
    disp("Torus condition NOT satisfied!")
    return
end

%% start control simulation
pt = p0;
Rt = R0;

Hist.position = [];
Hist.position_pos = []; % position with positive manifold
Hist.position_neg = []; % position with negative manifold
Hist.error = []; % error vector

current_time = 0; % keep track of current time
reached_manifold = false;
reached_tau = false;

% check initial sign
et = getErrorVec(Rt, pt, goal);
[~, initial_sign] = getManifold(et);

while true
    % Termination condition
    if et(3) <= 0
        disp("Condition ez<=0 satisfied at time t = ")
        disp(current_time)
        break
    end

    if current_time >= (dt * MAXITER)
        break
    end

    % check error and manifold
    et = getErrorVec(Rt, pt, goal);
    [~, sigma_sign] = getManifold(et);
    % get control signal
    ut = getControl(lambda1, lambda2, sigma_sign);

    if (~reached_tau) && getTimeConstant(et, ut, const_q, k, current_time)
        reached_tau = true;
        tau = current_time;
        disp('Enter oscillation phase')
        fprintf("tau = %f s\n", tau);
    end

    if (sigma_sign ~= initial_sign) && (~reached_manifold)
        reached_manifold = true;
        fprintf("reached manifold at time: %f s\n", current_time);

        if current_time <= (const_q + tau)
            fprintf("before %f s, satisfy time condition\n", (const_q + tau));
        else
            fprintf("after %f s, did not satisfied time condition\n", (const_q + tau));
        end

    end

    % Simulate one step dynamics
    [Rnext, pnext] = dynamics_dt(Rt, pt, ut, k, dt);

    Rt = Rnext;
    pt = pnext;
    current_time = current_time + dt;

    % Record data
    if sigma_sign <= 0
        Hist.position_neg = [Hist.position_neg pnext];
    else
        Hist.position_pos = [Hist.position_pos pnext];
    end

    Hist.position = [Hist.position pnext];
    Hist.error = [Hist.error et];

end

%% Get upper bound on the Error
upper_bound = lambda_ratio / qbar^3 + (4 * pi * lambda_ratio^3 + 2 * lambda_ratio) / (qbar^3 * (lambda_ratio^2 - 1));

if norm(et) <= upper_bound
    disp("Final error smaller than upper bound")
else
    disp("Final error greater than upper bound")
end

%% Plot results
figure(1)
plot3(Hist.position_pos(1, :), Hist.position_pos(2, :), Hist.position_pos(3, :), 'magenta.', 'LineWidth', 2, 'DisplayName', '$$sigma \geq 0$$')
hold on
plot3(Hist.position_neg(1, :), Hist.position_neg(2, :), Hist.position_neg(3, :), 'cyan.', 'LineWidth', 2, 'DisplayName', '$$sigma \leq 0$$')
plot3(goal(1), goal(2), goal(3), 'r*', 'DisplayName', 'Goal')
grid on
xlabel('x m/s');
ylabel('y m/s');
zlabel('z m/s');
legend('interpreter', 'latex', 'location', 'best');
grid on
set(gca, 'FontSize', 16)
set(gcf, 'position', [100, 100, 700, 700])

figure(2)
hold on
plot((1:size(Hist.error, 2)) * dt, Hist.error(1, :), 'LineWidth', 2, 'DisplayName', '$$e_x$$')
plot((1:size(Hist.error, 2)) * dt, Hist.error(2, :), 'LineWidth', 2, 'DisplayName', '$$e_y$$')
plot((1:size(Hist.error, 2)) * dt, Hist.error(3, :), 'LineWidth', 2, 'DisplayName', '$$e_z$$')
legend('interpreter', 'latex', 'location', 'best');
xlabel('Time s', 'interpreter', 'latex');
ylabel('Error', 'interpreter', 'latex')
set(gca, 'FontSize', 16)
set(gcf, 'position', [100, 100, 700, 700])
