close all; clc;

fileCoeff = 'data/coeff_a_star';
dataCoefficients = load(fileCoeff, 'ax_star', 'ay_star');
ax = dataCoefficients.ax_star;
ay = dataCoefficients.ay_star;

% GENERATION OF THE COEFFICIENT RELATED TO OPTIMAL TRAJECTORY
optimizMatrix = [ax, ay];

% GENERATION OF THE COEFFICIENT RELATED TO NON-OPTIMIZED TRAJECTORY
[coeffs, ~] = coeff_generation(totalTime, dx, dy);

% Generate NON-OPTIMAL trajectory
[posNonOpt, velNonOpt, accNonOpt, thetaNonOpt] = trajectory_generation(coeffs, timeVec, totalTime, ...
    linewidth, colors, false);
%% GENERATE OPTIMIZED TRAJECTORY
[posOpt, velOpt, accOpt, thetaOpt] = trajectory_generation(optimizMatrix, timeVec, totalTime, ...
    linewidth, colors, true);

%% GENERATION OF NOT-OPTIMIZED TRAJECTORY WITH NOMINAL PARAMETERS
[q_NOPT_NOM, ~, ~, e_NOPT_NOM] = simulation_loop(initialPositionVec, initialVelocityVec, ...
    delta, ...
    nominal_params, perturbed_params, false, ...
    posNonOpt, velNonOpt, accNonOpt, thetaNonOpt, kv,ki,kp);
%% GENERATION OF OPTIMIZED TRAJECTORY WITH NOMINAL PARAMETERS
[q_OPT_NOM, ~, ~, e_OPT_NOM] = simulation_loop(initialPositionVec, initialVelocityVec, ...
    delta, ...
    nominal_params, perturbed_params, false, ...
    posOpt, velOpt, accOpt, thetaOpt, kv,ki,kp);
%% GENERATION OF OPTIMIZED TRAJECTORY WITH PERTURBED PARAMETERS
[q_OPT_PERT, ~, ~, e_OPT_PERT] = simulation_loop(initialPositionVec, initialVelocityVec, ...
    delta, ...
    nominal_params, perturbed_params, true, ...
    posOpt, velOpt, accOpt, thetaOpt, kv, ki, kp);

%% GENERATION OF NOT-OPTIMIZED TRAJECTORY WITH PERTURBED PARAMETERS
   [q_NOPT_PERT, ~, ~, e_NOPT_PERT] = simulation_loop(initialPositionVec, initialVelocityVec, ...
    delta, ...
    nominal_params, perturbed_params, true, ...
    posNonOpt, velNonOpt, accNonOpt, thetaNonOpt, kv,ki,kp);

%% Plotting the errors
%Plot the errors comparison with the PERTURBED parameters
    %'e_x  [m]; e_y  [m]; e_theta [m]; e_tot [m]',...
plot_function([e_NOPT_PERT,e_OPT_PERT], 'Errors with PERTURBED parameters, OptimalvsNonOptimal trajectory tracking', ...
    '\bf{Error = q_{des} -  q_{robot}}',...
    '          ;        ;         ;         ',...
    'NON-OPT;NON-OPT;NON-OPT;NON-OPT;OPT;OPT;OPT;OPT', ...
    '\boldmath{$e_{tot}$};\boldmath{$e_{\theta}$};$\sqrt{e_{x}^2 + e_{y}^2}$;',...
    timeVec, linewidth, colors,f)
%Plot the errors comparison with the NOMINAL parameters
      %'e_x  [m]; e_y  [m]; e_theta [m]; e_tot [m]', ...
plot_function([e_NOPT_NOM,e_OPT_NOM], 'Errors with NOMINAL parameters, OptimalvsNonOptimal trajectory tracking', ...
    '\bf{Error = q_{des} -  q_{robot}}',...
     '         ;        ;        ;          ',...
     'NON-OPT;NON-OPT;NON-OPT;NON-OPT;OPT;OPT;OPT;OPT',...
    '\boldmath{$e_{tot}$};\boldmath{$e_{\theta}$};$\sqrt{e_{x}^2 + e_{y}^2}$;',...
    timeVec, linewidth, colors,f)

%% Calculating the error between the state evolution in the optimal and not optimal case, considering the perturbed or the nominal parameters
% Error between the state vectors with the perturbed and nominal parameters, in the optimal case.
e_OPT_PerturbedVSNominal = zeros(3, Nstep); eTot_OPT_PerturbedVSNominal = zeros(Nstep, 1);

for i = 1:Nstep
    e_OPT_PerturbedVSNominal(:, i) = abs(q_OPT_NOM(:, i) - q_OPT_PERT(:, i));
    eTot_OPT_PerturbedVSNominal(i) = sqrt(e_OPT_PerturbedVSNominal(1,i)^2 + e_OPT_PerturbedVSNominal(2,i)^2 + e_OPT_PerturbedVSNominal(3,i)^2);
end

% Error between the state vectors with the perturbed and nominal parameters, in the non-optimal case.
e_NOPT_PerturbedVSNominal = zeros(3, Nstep); eTot_NOPT_PerturbedVSNominal = zeros(Nstep, 1);

for j = 1:Nstep
    e_NOPT_PerturbedVSNominal(:, j) = abs(q_NOPT_NOM(:, j) - q_NOPT_PERT(:, j));
    eTot_NOPT_PerturbedVSNominal(j) = sqrt(e_NOPT_PerturbedVSNominal(1,j)^2 + e_NOPT_PerturbedVSNominal(2,j)^2 + e_NOPT_PerturbedVSNominal(3,j)^2);
end

    %'e_x  [m]; e_y  [m]; e_theta [m]; e_tot [m]', ...
plot_function([e_NOPT_PerturbedVSNominal',eTot_NOPT_PerturbedVSNominal,e_OPT_PerturbedVSNominal',eTot_OPT_PerturbedVSNominal],...
    'Difference between the state variables, nominal - perturbed, on the OPTIMAL and NON-OPTIMAL trajectory', ...
    '\bf{Error = q_{nom} -  q_{pert}}',...
    '         ;         ;        ;          ',...
    'NON-OPT;NON-OPT;NON-OPT;NON-OPT;OPT;OPT;OPT;OPT', ...
    '\boldmath{$e_{\theta}$};\boldmath{$e_{tot}$}; ;$\sqrt{e_{x}^2 + e_{y}^2 + e_{\theta}^2}$',...
    timeVec, linewidth, colors,f)

%% Final difference of all components non-optimal/optimal
x_nopt = e_NOPT_PerturbedVSNominal(1,Nstep);
y_nopt = e_NOPT_PerturbedVSNominal(2,Nstep);
theta_nopt = e_NOPT_PerturbedVSNominal(3,Nstep);
x_opt = e_OPT_PerturbedVSNominal(1,Nstep);
y_opt = e_OPT_PerturbedVSNominal(2,Nstep);
theta_opt = e_OPT_PerturbedVSNominal(3,Nstep);
performance = sqrt(x_nopt^2 + y_nopt^2 + theta_nopt^2)/sqrt(x_opt^2 + y_opt^2 + theta_opt^2);
strg = ['The total difference al all the states in the optimal case is ', sprintf('%1.1f',performance),' times smaller than in the non optimal one.'];
disp("PAPER ERRORS:")
disp(strg)

%% Final difference of all components non-optimal/optimal WITH OUR ERRORS
ex_nopt = e_NOPT_PERT(Nstep,1);
ey_nopt = e_NOPT_PERT(Nstep,2);
etheta_nopt = e_NOPT_PERT(Nstep,4);
ex_opt = e_OPT_PERT(Nstep,1);
ey_opt = e_OPT_PERT(Nstep,2);
etheta_opt = e_OPT_PERT(Nstep,4);
eperformance = sqrt(ex_nopt^2 + ey_nopt^2 + etheta_nopt^2)/sqrt(ex_opt^2 + ey_opt^2 + etheta_opt^2);
strg = ['The total difference al all the states in the optimal case is ', sprintf('%1.1f',eperformance),' times smaller than in the non optimal one.'];
disp("OUR ERRORS:")
disp(strg)
% Difference only on x and y withouth theta
eperf_xy = e_NOPT_PERT(Nstep,3)/e_OPT_PERT(Nstep,3);
strg = ['The difference on x and y in the optimal case is ', sprintf('%1.1f',eperf_xy),' times smaller than in the non optimal one.'];
disp(strg)