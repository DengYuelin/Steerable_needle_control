clc, clear, close all
%%
T = 25; % total time 25s
dt = 0.001;
tspan = 0:dt:T;

% Constants
lambda1 = 0.005; % m/s, equal to 5 mm/s
k = 10;
lambda2 = 2 * pi; % rad/s
epsilon = deg2rad(1);
% initial state
p0 = [0; 0; 0];
eul0 = [0, 0, 0];
R0 = eul2rotm(eul0);
%% Generate trajectory
RefTraj.A = 0.3; %mm
RefTraj.B = 0.0004; % mm/s^2
RefTraj.a = 0.012; % mm/s
RefTraj.w = 0.5; %rad/s
Pd = zeros(3, size(tspan, 2));

for i = 1:size(tspan, 2)
    t = tspan(i);
    Pd(1, i) = (RefTraj.A - RefTraj.a * t) * cos(RefTraj.w * t) - RefTraj.A;
    Pd(2, i) = RefTraj.B * t^2;
    Pd(3, i) = (RefTraj.A - RefTraj.a * t) * sin(RefTraj.w * t);
end

%% start control simulation
pt = p0;
Rt = R0;

Hist.position = [];
Hist.error = []; % error vector
Rlimit.W = 0;
Rlimit.theta = 0;

for i = 1:size(tspan, 2) - 1
    t = tspan(i);
    et = getErrorVec(Rt, pt, Pd(:, i + 1));
    [sigma, sigma_sign] = getManifold(et);

    traj.pdot = norm([(-RefTraj.w * (RefTraj.A - RefTraj.a * t) * sin(RefTraj.w * t) - RefTraj.a * cos(RefTraj.w * t));
                2 * RefTraj.B * t;
                (RefTraj.w * (RefTraj.A - RefTraj.a * t) * cos(RefTraj.w * t) - RefTraj.a * sin(RefTraj.w * t))]);
    traj.ez = et(3);
    traj.kp = 0.5;

    ut = getControl(lambda1, lambda2, sigma, epsilon, Rlimit, traj);
    % Simulate one step dynamics
    [Rnext, pnext] = dynamics_dt(Rt, pt, ut, k, dt);

    Rt = Rnext;
    pt = pnext;
    Rlimit.theta = Rlimit.theta + ut(2) * dt;

    if abs(Rlimit.theta) > deg2rad(270)
        Rlimit.W = 1;
    elseif abs(Rlimit.theta) < deg2rad(45)
        Rlimit.W = 0;
    end

    Hist.position = [Hist.position pnext];
    Hist.error = [Hist.error norm(et)];
end

%% Plot the trajectory
figure(1)
plot(Pd(1, :), Pd(3, :), 'ro')
grid on
hold on
plot(Hist.position(1, :), Hist.position(3, :), 'b-', 'LineWidth', 2)
set(gca, 'FontSize', 16)
set(gcf, 'position', [100, 100, 700, 700])

figure(2)
plot(Pd(2, :), Pd(3, :), 'ro')
grid on
hold on
plot(Hist.position(2, :), Hist.position(3, :), 'b-', 'LineWidth', 2)
set(gca, 'FontSize', 16)
set(gcf, 'position', [100, 100, 700, 700])

% figure(3)
% plot3(Pd(1, :), Pd(2, :), Pd(3, :))
% grid on
% xlabel('x');
% ylabel('y');
% zlabel('z');
%%
figure(4)
plot(tspan(1:end - 1), Hist.error)
set(gca, 'YScale', 'log')
