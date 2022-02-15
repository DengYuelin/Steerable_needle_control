clc, clear, close all
%% Constants
experiments = 5000;
const.dt = 0.05;
const.MAXITER = 5e4;

%% Generate random experiments
curvature_list = rand(1, experiments) * 19 + 1;
ratio_list = rand(1, experiments) * 99 + 1;
goal_list = zeros(3, experiments);
goal_list(2, :) = rand(1, experiments) .* (-2 ./ curvature_list);
goal_list(3, :) = rand(1, experiments) .* (2 ./ curvature_list);

error_hist = zeros(experiments, 1);

parfor i = 1:experiments
    setup = [];
    setup.k = curvature_list(i);
    setup.lambda_ratio = ratio_list(i);
    setup.goal = goal_list(:, i);
    error_hist(i) = simulate(setup, const);
end

%% Plot result
upper_bound = zeros(1000, 1);
lambdabar = linspace(1, 100, 1000);

for i = 1:1000
    qbar = sqrt(1 + lambdabar(i)^2);
    upper_bound(i) = lambdabar(i) / qbar^3 + (4 * pi * lambdabar(i)^3 + 2 * lambdabar(i)) / (qbar^3 * (lambdabar(i)^2 - 1));
end

index=isnan(error_hist);
error_hist(index) = [];
ratio_list(index) = [];

figure(1)
loglog(lambdabar, upper_bound, 'DisplayName', 'Theoretical Upper Bound')
hold on
plot(ratio_list, error_hist, 'r.', 'DisplayName', 'Value From Simulation')
grid on
legend('interpreter', 'latex', 'location', 'NorthEast');
xlabel('$$\bar{\lambda}=\frac{\lambda_2}{k\lambda_1}$$', 'interpreter', 'latex');
ylabel('$$\min{(k||e(t)||)}$$', 'interpreter', 'latex')
set(gca, 'FontSize', 16)
set(gcf, 'position', [100, 100, 900, 700])
