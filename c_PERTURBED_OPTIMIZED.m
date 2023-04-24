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
%Plot the errors comparison with the PERTURBED parameters
plot_function([e_NOPT_PERT,e_OPT_PERT], 'Errors with PERTURBED parameters, OptimalvsNonOptimal trajectory tracking', ...
    'e_x [m];e_y [m];e_tot [m];e_theta [rad]',...
    'ex NOPT;ey NOPT;etot NOPT;e_theta NOPT;ex OPT;ey OPT;etot OPT;e_theta OPT', ...
    timeVec, linewidth, colors, counter)
%Plot the errors comparison with the NOMINAL parameters
plot_function([e_NOPT_NOM,e_OPT_NOM], 'Errors with NOMINAL parameters, OptimalvsNonOptimal trajectory tracking', ...
    'e_x [m];e_y [m];e_tot [m];e_theta [rad]',...
    'ex NOPT;ey NOPT;etot NOPT;e_theta NOPT;ex OPT;ey OPT;etot OPT;e_theta OPT', ...
    timeVec, linewidth, colors, counter)

%% Plot the states
% Plot the comparison between the state evolution in the optimal and not optimal case, with the PERTURBED parameters
%plot_function([q_NOPT_PERT', q_OPT_PERT'],'Comparison between NOT OPTIMAL and OPTIMAL state evolution, PERTURBED case', ...
%    'x [m] ; y [m]; theta [rad]', ...
%   'x NON OPT ;y NON OPT ;theta NON OPT;x OPT;y OPT;theta OPT', ...
%   timeVec, linewidth, colors, counter)
% Plot the comparison between the state evolution in the optimal and not optimal case with with the NOMINAL parameters
%plot_function([q_NOPT_NOM', q_OPT_NOM'], 'Comparison between NOT OPTIMAL and OPTIMAL state evolution, NOMINAL case', ...
%    'x [m] ; y [m]; theta [rad]', ...
%   'x NON OPT ;y NON OPT ;theta NON OPT;x OPT;y OPT;theta OPT', ...
%   timeVec, linewidth, colors, counter)

%% Calculating the error between the state evolution in the optimal and not optimal case, considering the perturbed or the nominal parameters
% Error between the state vectors with the perturbed and nominal parameters, in the optimal case.
e_OPT_PerturbedVSNominal = zeros(3, Nstep); eTot_OPT_PerturbedVSNominal = zeros(Nstep, 1);

for i = 1:Nstep
    e_OPT_PerturbedVSNominal(:, i) = abs(q_OPT_NOM(:, i) - q_OPT_PERT(:, i));
    eTot_OPT_PerturbedVSNominal(i) = sqrt_of_quadratics(e_OPT_PerturbedVSNominal(:, i));
end

% Error between the state vectors with the perturbed and nominal parameters, in the non-optimal case.
e_NOPT_PerturbedVSNominal = zeros(3, Nstep); eTot_NOPT_PerturbedVSNominal = zeros(Nstep, 1);

for i = 1:Nstep
    e_NOPT_PerturbedVSNominal(:, i) = abs(q_NOPT_NOM(:, i) - q_NOPT_PERT(:, i));
    eTot_NOPT_PerturbedVSNominal(i) = sqrt_of_quadratics(e_NOPT_PerturbedVSNominal(:, i));
end

plot_function([e_OPT_PerturbedVSNominal',eTot_OPT_PerturbedVSNominal,e_NOPT_PerturbedVSNominal',eTot_NOPT_PerturbedVSNominal],...
    'Difference between the state variables, perturbed - nominal, on the OPTIMAL and NON-OPTIMAL trajectory', ...
    'e_x = x_nom - x_pert [m]; e_y = y_nom - y_pert [m];e_theta = theta_nom - theta_pert [m]; e_tot_OPT = q_NOM - q_PERT [m]', ...
    'e_x_OPT;e_y_OPT;e_theta_OPT;e_tot_OPT;e_x_NOPT;e_y_NOPT;e_theta_NOPT;e_tot_NOPT', ...
    timeVec, linewidth, colors, counter)

%% Final difference of all components non-optimal/optimal
x_nopt = e_NOPT_PerturbedVSNominal(1,Nstep);
y_nopt = e_NOPT_PerturbedVSNominal(2,Nstep);
theta_nopt = e_NOPT_PerturbedVSNominal(3,Nstep);
x_opt = e_OPT_PerturbedVSNominal(1,Nstep);
y_opt = e_OPT_PerturbedVSNominal(2,Nstep);
theta_opt = e_OPT_PerturbedVSNominal(3,Nstep);
performance = sqrt( x_opt^2 + y_opt^2 + theta_opt^2)/sqrt( x_nopt^2 + y_nopt^2 + theta_nopt^2);
strg = ['The total difference al all the states in the optimal case is ', sprintf('%1.1f',performance),' times smaller than in the non optimal one.'];
disp(strg)

% Error between the state vectors of the optimal and not-optimal trajectory, in the PERTURBED parameters.
e_Perturbed_NOPTvsOPT = zeros(3, Nstep); eTot_Perturbed_NOPTvsOPT = zeros(Nstep, 1);

for i = 1:Nstep
    e_Perturbed_NOPTvsOPT(:, i) = abs(q_NOPT_PERT(:, i) - q_OPT_PERT(:, i));
    eTot_Perturbed_NOPTvsOPT(i) = sqrt_of_quadratics(e_Perturbed_NOPTvsOPT(:, i));
end

% Error between the state vectors of the optimal and not-optimal trajectory, in the NOMINAL parameters.
e_Nominal_NOPTvsOPT = zeros(3, Nstep); eTot_Nominal_NOPTvsOPT = zeros(Nstep, 1);

for i = 1:Nstep
    error_Nominal_NOPTvsOPT(:, i) = abs(q_NOPT_NOM(:, i) - q_OPT_NOM(:, i));
    eTot_Nominal_NOPTvsOPT(i) = sqrt_of_quadratics(e_Nominal_NOPTvsOPT(:, i));
end

plot_function([e_Perturbed_NOPTvsOPT',eTot_Perturbed_NOPTvsOPT,e_Nominal_NOPTvsOPT', eTot_Nominal_NOPTvsOPT],...
    'Difference between the state variables nonOptimal - optimal, in the NOMINAL and PERTURBED parameters', ...
    'e_x = x_nopt - x_opt [m]; e_y = y_nopt - y_opt [m];e_theta = theta_nopt - theta_opt [m]; e_tot_OPT = q_NOPT - q_OPT [m]', ...
    'e_x_PERT;e_y_PERT;e_theta_PERT;e_tot_PERT;e_x_NOM;e_y_NOM;e_theta_NOM;e_tot_NOM',...
    timeVec, linewidth, colors, counter)

%% Videos
b_n = perturbed_params(2);
%video(q_history,r_d,b_n,timeVec,linewidth,delta, 'Non Optimized Trajectory Following')
%% Create and display video animation and plots for the Optimal trajectory.
%video(q_history,opt_traj,b_n,timeVec,linewidth,delta, 'Optimal Trajectory Following')
