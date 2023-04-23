close all; clc;

fileCoeff = 'data/coeff_a_star';
dataCoefficients = load(fileCoeff, 'ax_star', 'ay_star');
ax = dataCoefficients.ax_star;
ay = dataCoefficients.ay_star;

%% GENERATION OF THE COEFFICIENT RELATED TO OPTIMAL TRAJECTORY
optimizMatrix = [ax, ay];

%% GENERATION OF THE COEFFICIENT RELATED TO NON-OPTIMIZED TRAJECTORY
[coeffs, ~] = coeff_generation(totalTime, dx, dy);

%% Desired Trajectory Generation (Spline)
% Generate NON-OPTIMAL trajectory
[posNonOpt, velNonOpt, accNonOpt, thetaNonOpt] = trajectory_generation(coeffs, timeVec, totalTime, ...
    linewidth, colors, false);
%% GENERATE OPTIMIZED TRAJECTORY
[posOpt, velOpt, accOpt, thetaOpt] = trajectory_generation(optimizMatrix, timeVec, totalTime, ...
    linewidth, colors, false);

%% GENERATION OF OPTIMIZED TRAJECTORY WITH PERTURBED PARAMETERS
[q_OPT_PERT, ~, ~, e_OPT_PERT] = simulation_loop(initialPositionVec, initialVelocityVec, ...
    delta, ...
    nominal_params, perturbed_params, true, ...
    posOpt, velOpt, accOpt, thetaOpt);
%% GENERATION OF NOT-OPTIMIZED TRAJECTORY WITH PERTURBED PARAMETERS
[q_NOPT_PERT, ~, ~, e_NOPT_PERT] = simulation_loop(initialPositionVec, initialVelocityVec, ...
    delta, ...
    nominal_params, perturbed_params, true, ...
    posNonOpt, velNonOpt, accNonOpt, thetaNonOpt);
%% GENERATION OF NOT-OPTIMIZED TRAJECTORY WITH NOMINAL PARAMETERS
[q_NOPT_NOM, ~, ~, e_NOPT_NOM] = simulation_loop(initialPositionVec, initialVelocityVec, ...
    delta, ...
    nominal_params, perturbed_params, false, ...
    posNonOpt, velNonOpt, accNonOpt, thetaNonOpt);
%% GENERATION OF OPTIMIZED TRAJECTORY WITH NOMINAL PARAMETERS
[q_OPT_NOM, ~, ~, e_OPT_NOM] = simulation_loop(initialPositionVec, initialVelocityVec, ...
    delta, ...
    nominal_params, perturbed_params, false, ...
    posOpt, velOpt, accOpt, thetaOpt);

%% Plotting the errors
%Plot the errors in the non-optimal case, considering the perturbed parameters
plot_function(e_NOPT_PERT', 'Errors in the NON-OPTIMAL case, considering the PERTURBED parameters', ...
    'e_x [m];e_y [m];e_theta [rad];e_tot [m]','', ...
    timeVec, linewidth, colors, counter)
%Plot the errors in the non-optimal case, considering the nominal parameters
plot_function(e_NOPT_NOM', 'Errors in the NON-OPTIMAL case, considering the NOMINAL parameters', ...
    'e_x [m];e_y [m];e_theta [rad];e_tot [m]','', ...
    timeVec, linewidth, colors, counter)
%Plot the errors in the optimal case, considering the perturbed parameters
plot_function(e_OPT_PERT', 'Errors in the OPTIMAL case, considering the PERTURBED parameters', ...
    'e_x [m];e_y [m];e_theta [rad];e_tot [m]','', ...
    timeVec, linewidth, colors, counter)
%Plot the errors in the optimal case, considering the nominal parameters
plot_function(e_OPT_NOM', 'Errors in the OPTIMAL case, considering the NOMINAL parameters', ...
    'e_x [m];e_y [m];e_theta [rad];e_tot [m]', '', ...
    timeVec, linewidth, colors, counter)

%% Plot the states
% Plot the comparison between the state evolution in the optimal and not optimal case, with the PERTURBED parameters
plot_function([q_NOPT_PERT', q_OPT_PERT'], 'Comparison between NOT OPTIMAL and OPTIMAL state evolution, PERTURBED case', ...
    'x [m] ; y [m]; theta [rad]', ...
    'x NON OPT ;y NON OPT ;theta NON OPT;x OPT;y OPT;theta OPT', ...
    timeVec, linewidth, colors, counter)
% Plot the comparison between the state evolution in the optimal and not optimal case with with the NOMINAL parameters
plot_function([q_NOPT_NOM', q_OPT_NOM'], 'Comparison between NOT OPTIMAL and OPTIMAL state evolution, NOMINAL case', ...
    'x [m] ; y [m]; theta [rad]', ...
    'x NON OPT ;y NON OPT ;theta NON OPT;x OPT;y OPT;theta OPT', ...
    timeVec, linewidth, colors, counter)

%% Calculating the error between the state evolution in the optimal and not optimal case, considering the perturbed or the nominal parameters
% Error between the state vectors with the perturbed and nominal parameters, in the optimal case.
e_OPT_PerturbedVSNominal = zeros(3, Nstep); eTot_OPT_PerturbedVSNominal = zeros(Nstep, 1);

for i = 1:Nstep
    e_OPT_PerturbedVSNominal(:, i) = q_OPT_PERT(:, i) - q_OPT_NOM(:, i);
    eTot_OPT_PerturbedVSNominal(i) = sqrt_of_quadratics(e_OPT_PerturbedVSNominal(:, i));
end

% Error between the state vectors with the perturbed and nominal parameters, in the non-optimal case.
error_NOPT_PerturbedVSNominal = zeros(3, Nstep); eTot_NOPT_PerturbedVSNominal = zeros(Nstep, 1);

for i = 1:Nstep
    error_NOPT_PerturbedVSNominal(:, i) = q_NOPT_PERT(:, i) - q_NOPT_NOM(:, i);
    eTot_NOPT_PerturbedVSNominal(i) = sqrt_of_quadratics(error_NOPT_PerturbedVSNominal(:, i));
end

plot_function([e_OPT_PerturbedVSNominal',eTot_OPT_PerturbedVSNominal,error_NOPT_PerturbedVSNominal',eTot_NOPT_PerturbedVSNominal],...
    'Difference between the state variables, perturbed - nominal, on the OPTIMAL and NON-OPTIMAL trajectory', ...
    'e_x = x_pert - x_nom [m]; e_y = y_pert - y_nom [m];e_theta = theta_pert - theta_nom [m]; e_tot_OPT = q_PERT - q_NOM [m]', ...
    'e_x_OPT;e_y_OPT;e_theta_OPT;e_tot_OPT;e_x_NOPT;e_y_NOPT;e_theta_NOPT;e_tot_NOPT', ...
    timeVec, linewidth, colors, counter)

% Error between the state vectors of the optimal and not-optimal trajectory, in the PERTURBED parameters.
error_Perturbed_NOPTvsOPT = zeros(3, Nstep); eTot_Perturbed_NOPTvsOPT = zeros(Nstep, 1);

for i = 1:Nstep
    error_Perturbed_NOPTvsOPT(:, i) = q_NOPT_PERT(:, i) - q_OPT_PERT(:, i);
    eTot_Perturbed_NOPTvsOPT(i) = sqrt_of_quadratics(error_Perturbed_NOPTvsOPT(:, i));
end

% Error between the state vectors of the optimal and not-optimal trajectory, in the NOMINAL parameters.
error_Nominal_NOPTvsOPT = zeros(3, Nstep); eTot_Nominal_NOPTvsOPT = zeros(Nstep, 1);

for i = 1:Nstep
    error_Nominal_NOPTvsOPT(:, i) = q_NOPT_NOM(:, i) - q_OPT_NOM(:, i);
    eTot_Nominal_NOPTvsOPT(i) = sqrt_of_quadratics(error_Nominal_NOPTvsOPT(:, i));
end

plot_function([error_Perturbed_NOPTvsOPT',eTot_Perturbed_NOPTvsOPT,error_Nominal_NOPTvsOPT', eTot_Nominal_NOPTvsOPT],...
    'Difference between the state variables nonOptimal - optimal, in the NOMINAL and PERTURBED parameters', ...
    'e_x = x_nopt - x_opt [m]; e_y = y_nopt - y_opt [m];e_theta = theta_nopt - theta_opt [m]; e_tot_OPT = q_NOPT - q_OPT [m]', ...
    'e_x_PERT;e_y_PERT;e_theta_PERT;e_tot_PERT;e_x_NOM;e_y_NOM;e_theta_NOM;e_tot_NOM',...
    timeVec, linewidth, colors, counter)

%% Videos
b_n = perturbed_params(2);
%video(q_history,r_d,b_n,timeVec,linewidth,delta, 'Non Optimized Trajectory Following')
%% Create and display video animation and plots for the Optimal trajectory.
%video(q_history,opt_traj,b_n,timeVec,linewidth,delta, 'Optimal Trajectory Following')
