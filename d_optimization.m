close all; clc;
%% In this file we computed all the thing necessary for the optimization
% At first we computed all the matrix necessary for the computation of the
% two sensitivity, with the use of the MatlabFunction created in the file b.
% And then we make the integration of the two sensitivity that we used
% to compute the new coefficients of the optimal trajectory at the end of the file.
tic
%% OPTIMIZATION CYCLE
% Hyperparameters, chosen in this way, to make a scaling to the size we are interested in
k1 = 1; k2 = 0.7; epochs = 100; h = 0.0005;

% Initialize loss function
Loss = zeros(1, epochs);

% Generate initial trajectory
[aMatrix, M] = coeff_generation(totalTime, dx, dy);
initial_ax = aMatrix(:, 1);
initial_ay = aMatrix(:, 2);

% Get the number of rows
[grado, ~] = size(aMatrix);

% Initialize the gradient vector and the identity matrix
vx = zeros(grado, 1); vy = zeros(grado, 1); I = eye(grado);

% Initialize a vector of the evolution of the parameters
ax_evolution = zeros(grado, epochs); ax_evolution(:, 1) = initial_ax;
ay_evolution = zeros(grado, epochs); ay_evolution(:, 1) = initial_ay;

% Color definition for the different epochs
colorsOfDifferentTrajectories = linspecer(epochs,'sequential');
% Define a counter to iterate over the colors to be chosen for the various trajectories
counterColorTrajectory = 1;
% Define a vector that contains the legend to be shown for plots
b = zeros(epochs);
fig1 = figure(15); hold on
% Define a vector containing the sensitivity at each epoch
sensitivityArrayEpochs = cell(epochs, 1);
counterForLegend = 1;

%% Optimization cycle
for n = 1:epochs
    disp(n),
    ax_old = ax_evolution(:, n); ay_old = ay_evolution(:, n);
    oldCoeffMatrix = [ax_old, ay_old];
    % Generate NEW trajectory
    [r_d, dr_d, ddr_d, theta_d] = trajectory_generation(oldCoeffMatrix, timeVec, totalTime, ...
        linewidth, colors, false);

    % Running the simulation loop for every new trajectory, ALWAYS NOMINAL CASE
    [q_history, u_history, xhi_history, ~] = simulation_loop(initialPositionVec, initialVelocityVec, ...
        delta, ...
        nominal_params, perturbed_params, false, ...
        r_d, dr_d, ddr_d, theta_d,kv,ki,kp);

    % Sensitivity calculation
    [sens_last, sens_hist] = sensitivity_integration(Nstep, nominal_params, ...
        q_history, xhi_history, u_history, ...
        r_d, dr_d, ddr_d, ...
        kv,ki,kp,delta);
    % Saving the sensivity for the plot
    sensitivityArrayEpochs{n} = sens_hist;

    % Sensitivity_ai calculation, by calling the function gamma_integration
    sens_ai_Array = sensitivity_ai_integration_through_gamma(sens_hist, oldCoeffMatrix, ...
        nominal_params, timeVec, ...
        q_history, xhi_history, u_history, ...
        r_d, dr_d, ddr_d, ...
        kv,ki,kp,delta, Nstep);

    %% Calculate vi for each x and y trajectory's coefficient which is the negative gradient of the cost function
    for i = 1:grado
        sensai_last = reshape(sens_ai_Array{i, 1}(1:6, Nstep), 2, [])';
        vx(i) = -trace(sens_last' * sensai_last);
    end

    for i = 1:grado
        sensai_last = reshape(sens_ai_Array{i, 2}(1:6, Nstep), 2, [])';
        vy(i) = -trace(sens_last' * sensai_last);
    end

    % Calculate the loss function: as norm the trace of the sensitivity.
    Loss(n) = 0.5 * trace(sens_last' * sens_last);

    % Update law of the optimization
    ax_new = ax_old + h * (k1 * pinv(M) * (dx - M * ax_evolution(:, n)) + k2 * (I - pinv(M) * M) * vx);
    ay_new = ay_old + h * (k1 * pinv(M) * (dy - M * ay_evolution(:, n)) + k2 * (I - pinv(M) * M) * vy);

    ax_evolution(:, n + 1) = ax_new; ay_evolution(:, n + 1) = ay_new;

    % Plot the trajectories with different lines and different colors
    if counterForLegend <= epochs && mod(n, 2) == 0
        b(counterForLegend) = plot(r_d(1, :), r_d((2), :), 'Color', colorsOfDifferentTrajectories(counterColorTrajectory, :), 'LineWidth', linewidth, 'LineStyle', '-.', 'DisplayName', sprintf('Trajectory n: %d', counterForLegend));
        legend('show');
        drawnow;
        legend(b(1:counterForLegend))
    end

    if counterForLegend <= epochs && mod(n, 2) ~= 0
        b(counterForLegend) = plot(r_d(1, :), r_d((2), :), 'Color', colorsOfDifferentTrajectories(counterColorTrajectory, :), 'LineWidth', linewidth, 'DisplayName', sprintf('Trajectory n: %d', counterForLegend));
        legend('show');
        drawnow;
        legend(b(1:counterForLegend))
    end
    xlabel("x[m]"), ylabel('y[m]','Rotation',0), grid on
    title('Trajectory Variation in each epoch'),
    counterForLegend = counterForLegend + 1;
    counterColorTrajectory = counterColorTrajectory + 1;
end
fontsize(fig1, scale=1.2)  % 120%
hold off

%% Take the optimized trajectory as the last obtained in the optimization epochs:
% One could also take the one that minimizes the loss (if it's not the last)
ax_star = ax_evolution(:, epochs);
ay_star = ay_evolution(:, epochs);
optimizedCoeffMatrix = [ax_star, ay_star];

% Plot Loss function
fig = figure(5);
plot(1:epochs, Loss,'LineWidth',3),hold on
yline(0,'LineStyle','--','Color','k','LineWidth',1.5)
title('\bf{Loss Function variation in each epoch}')
subtitle('$|\!|$ $\Pi(t_{f})$ $|\!|$ = $\frac{1}{2} Tr(\Pi(t_{f})^T \Pi(t_{f}))$')
xlabel('\bf{epochs}','FontSize',fontSize);
ylabel('$|\!|$ $\Pi(t_{f})$ $|\!|$','Rotation',0,'FontSize',25);
legend('Loss(a)')
grid on
fontsize(fig, scale=1.2)  % 120%



% Plot Sensitivity
fig3 = figure(17); hold on

% Define the colors to be used for each era
% NOTE: change the quantity of colors refered to the number of iteration epochs
multiplier = 5;
%colorsOfDifferentSensitivities = linspecer(epochs*multiplier, "sequential");
colorsOfDifferentSensitivities = linspecer(epochs, "sequential");

string = ["$\partial x/\partial r$"; "$\partial x/\partial b$"; "$\partial y/\partial r$"; "$\partial y/\partial b$"; "$\partial \theta/\partial r$"; "$\partial \theta/\partial b$"];

tlo = tiledlayout(3,2);

for k = 1:size(sens_hist, 1)/2
    ax = nexttile(tlo);
    hold on
    for i = 1:epochs
        plot(ax, timeVec, sensitivityArrayEpochs{i,1}(k,:),'Color',colorsOfDifferentSensitivities(i,:));
        % Define a support vector that contains the legend
        lgd{i} = sprintf('Sens at epoch n:%d',i);
    end
    title(ax,[string(k)])
    xlabel(ax,'time [s]')
    grid on
    hold off
end


title(tlo,'Sensitivity variation in time and each epoch:','FontSize',fontSize);
subtitle(tlo,'\Pi(t)','FontSize',fontSize)
lg = legend(lgd);
lg.Layout.Tile = 'East'; % <-- place legend east of tiles
fontsize(fig3, scale=1.2)  % 120%
%% Save optimized coefficients and new trajectory
save('data/coeff_a_star', "ax_star", "ay_star")

toc
