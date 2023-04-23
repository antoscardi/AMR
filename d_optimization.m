close all; clc;
%% In this file we computed all the thing necessary for the optimization
% At first we computed all the matrix necessary for the computation of the
% two sensitivity, with the use of the MatlabFunction created in the file b.
% And then we make the integration of the two sensitivity that we used
% to compute the new coefficients of the optimal trajectory at the end of the file.
tic
%% OPTIMIZATION CYCLE
% Hyperparameters, chosen in this way, to make a scaling to the size we are interested in
k1 = 0.01; k2 = 0.1; epochs = 5;

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
colorsOfDifferentTrajectories = linspecer(epochs, "qualitative");
% Define a counter to iterate over the colors to be chosen for the various trajectories
counterColorTrajectory = 1;
% Define a vector that contains the legend to be shown for plots
b = zeros(epochs);
figure(15); hold on
% Define a vector containing the sensitivity at each epoch
sensitivityArrayEpochs = cell(epochs, 1);

%% Optimization cycle
for n = 1:epochs
    ax_old = ax_evolution(:, n); ay_old = ay_evolution(:, n);
    oldCoeffMatrix = [ax_old, ay_old];
    % Generate NEW trajectory
    [r_d, dr_d, ddr_d, theta_d] = trajectory_generation(oldCoeffMatrix, timeVec, totalTime, ...
        linewidth, colors, false);

    % Running the simulation loop for every new trajectory, ALWAYS NOMINAL CASE
    [q_history, u_history, xhi_history, ~] = simulation_loop(initialPositionVec, initialVelocityVec, ...
        delta, ...
        nominal_params, perturbed_params, false, ...
        r_d, dr_d, ddr_d, theta_d);

    % Sensitivity calculation
    [sens_last, sens_hist] = sensitivity_integration(Nstep, nominal_params, ...
        q_history, xhi_history, u_history, ...
        r_d, dr_d, ddr_d, ...
        delta);
    % Saving the sensivity for the plot
    sensitivityArrayEpochs{n} = sens_hist;

    % Sensitivity_ai calculation, by calling the function gamma_integration
    sens_ai_Array = sensitivity_ai_integration_through_gamma(sens_hist, oldCoeffMatrix, ...
        nominal_params, timeVec, ...
        q_history, xhi_history, u_history, ...
        r_d, dr_d, ddr_d, ...
        delta, Nstep);

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
    ax_new = ax_old + delta * (k1 * pinv(M) * (dx - M * ax_evolution(:, n)) + k2 * (I - pinv(M) * M) * vx);
    ay_new = ay_old + delta * (k1 * pinv(M) * (dy - M * ay_evolution(:, n)) + k2 * (I - pinv(M) * M) * vy);

    ax_evolution(:, n + 1) = ax_new; ay_evolution(:, n + 1) = ay_new;

    % Plot the trajectories with different lines and different colors
    if counterColorTrajectory <= epochs && mod(n, 2) == 0
        b(counterColorTrajectory) = plot(r_d(1, :), r_d((2), :), 'Color', colorsOfDifferentTrajectories(counterColorTrajectory, :), 'LineWidth', linewidth, 'LineStyle', '-.', 'DisplayName', sprintf('Trajectory n: %d', counterColorTrajectory));
        xlabel("x[m]"), ylabel('y[m]'), grid minor
        title('Trajectory Variation for each epoch'), fontsize(fontSize, "points")
        legend('show');
        drawnow;
        legend(b(1:counterColorTrajectory))
    end

    if counterColorTrajectory <= epochs && mod(n, 2) ~= 0
        b(counterColorTrajectory) = plot(r_d(1, :), r_d((2), :), 'Color', colorsOfDifferentTrajectories(counterColorTrajectory, :), 'LineWidth', linewidth, 'DisplayName', sprintf('Trajectory n: %d', counterColorTrajectory));
        xlabel("x[m]"), ylabel('y[m]'), grid minor
        title('Trajectory Variation for each epoch'), fontsize(fontSize, "points")
        legend('show');
        drawnow;
        legend(b(1:counterColorTrajectory))
    end

    counterColorTrajectory = counterColorTrajectory + 1;
    disp(n)
end

hold off

%% Take the optimized trajectory as the last obtained in the optimization epochs:
% One could also take the one that minimizes the loss (if it's not the last)
ax_star = ax_evolution(:, epochs);
ay_star = ay_evolution(:, epochs);
optimizedCoeffMatrix = [ax_star, ay_star];

%% Visualize optimized trajectory
[opt_traj, opt_vel, opt_acc] = trajectory_generation(optimizedCoeffMatrix, timeVec, totalTime, ...
    linewidth, colors, true);

% Plot Loss function
figure(5); hold on
plot(1:epochs, Loss)
title('Loss Function of a')
xlabel('epochs'); ylabel("Norm of sens at tf"); fontsize(fontSize, "points")
hold off

% Plot Sensitivity
figure(17); hold on
% Define a counter to iterate the different colors of the different sensitivities for each epoch
counterColorSens = 1;
% Define the colors to be used for each era
colorsOfDifferentSensitivities = linspecer(100, "qualitative");
% Define a support vector to get the value of the sensitivity at each epoch
sensAtEpoch = zeros(epochs, Nstep);
% Define a support vector that contains the legend
plot_handles = [];
% Define a counter to print the legend of the various elements of sensibility and sensitivity for each epoch
counterLegend = 1;

for i = 1:epochs
    sensAtEpoch = sensitivityArrayEpochs{i};

    for k = 1:size(sens_hist, 1)
        subplot(4, 3, k);
        plot_handles(counterLegend) = plot(timeVec, sensAtEpoch(k, :), 'Color', colorsOfDifferentSensitivities(counterColorSens, :), 'LineWidth', linewidth, 'DisplayName', ['Sens n:' num2str(k) ', at Epoch' num2str(i)]);
        title(['Sens n:' num2str(k)])
        hold on
        counterLegend = counterLegend + 1;
        legend(plot_handles(1:counterLegend - 1));
        legend('show')
    end

    counterColorSens = counterColorSens + 20;
end

%% Save optimized coefficients and new trajectory
save('data/coeff_a_star', "ax_star", "ay_star")
save('data/optimized_traj', 'opt_traj', 'opt_vel', 'opt_acc')
toc
