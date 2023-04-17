close all; clc;

fileCoeff = 'data/coeff_a_star';
dataCoefficients = load(fileCoeff,'ax_star','ay_star');
ax = dataCoefficients.ax_star;
ay = dataCoefficients.ay_star; 

%% GENERATION OF THE COEFFICIENT RELATED TO OPTIMAL TRAJECTORY
optimizMatrix= [ax,ay];

%% GENERATION OF THE COEFFICIENT RELATED TO DESIDERED TRAJECTORY
[coeffs, ~] = coeff_generation(totalTime, dx, dy);

%% Desired Trajectory Generation (Spline)
% Generate NON-OPTIMAL trajectory
[posNonOpt, velNonOpt, accNonOpt] = trajectory_generation(coeffs, timeVec, totalTime, ...
    linewidth, colors, false);
%% GENERATE OPTIMIZED TRAJECTORY 
[opt_traj, opt_vel, opt_acc] = trajectory_generation(optimizMatrix, timeVec, totalTime, ...
    linewidth, colors, false);
%% GENERATION OF OPTIMIZED TRAJECTORY WITH PERTURBED PARAMETERS
[q_OPT_PERT, u_OPT_PERT, xhi_OPT_PERT, e_OPT_PERT] = simulation_loop(initialPositionVec, initialVelocityVec, ...
    delta, ...
    nominal_params, perturbed_params, true, ...
    opt_traj, opt_vel, opt_acc);
%% GENERATION OF NOT-OPTIMIZED TRAJECTORY WITH PERTURBED PARAMETERS
[q_NOPT_PERT, u_NOPT_PERT, xhi_NOPT_PERT, e_NOPT_PERT] = simulation_loop(initialPositionVec, initialVelocityVec, ...
    delta, ...
    nominal_params, perturbed_params, true, ...
    posNonOpt, velNonOpt, accNonOpt);
%% GENERATION OF NOT-OPTIMIZED TRAJECTORY WITH NOMINAL PARAMETERS
[q_NOPT_NOM, u_NOPT_NOM, xhi_NOPT_NOM, e_NOPT_NOM] = simulation_loop(initialPositionVec, initialVelocityVec, ...
    delta, ...
    nominal_params, perturbed_params, false, ...
    posNonOpt, velNonOpt, accNonOpt);
%% GENERATION OF OPTIMIZED TRAJECTORY WITH NOMINAL PARAMETERS
[q_OPT_NOM, u_OPT_NOM, xhi_OPT_NOM, e_OPT_NOM] = simulation_loop(initialPositionVec, initialVelocityVec, ...
    delta, ...
    nominal_params, perturbed_params, false, ...
    opt_traj, opt_vel, opt_acc);

%% Color Definition
%1 -> blu
%2 -> rosso
%3 -> verde
%4 -> arancione
%5 -> viola
blu = colors(1,:);
red = colors(2,:);
green = colors(3,:);
orange = colors(4,:);
violet = colors(5,:);

%% Create and display video animation and plots for the NON-Optimal trajectory.
% Plot state variables (vector q).
plot_function([q_NOPT_PERT',q_OPT_PERT'], 'Comparison between NOT OPTIMAL and OPTIMAL state evolution in the perturbed case', 'X[m];Y[m];THETA[rad]', timeVec, linewidth, colors, counter)
% % Plot input (vector u).
% plot_function(u_NOPT_PERT, 'Input variation, non-optimal case', 'wr [rad/s] ; wl [rad/s]', timeVec, linewidth, colors, counter)
% % % Plot errors.
% plot_function(e_NOPT_PERT', 'Error variation, non-optimal case', 'e_x [m]; e_y [m]; e_tot [m];e_theta [m]', timeVec, linewidth, colors, counter)
% 
% % % The video function just needs the distance between the wheels in order to plot the robot.
% b_n = perturbed_params(2);
% %video(q_history,r_d,b_n,timeVec,linewidth,delta, 'Non Optimized Trajectory Following')
% 
% %% Create and display video animation and plots for the OPTIMAL trajectory.
% % Plot state variables (vector q).
% plot_function(q_OPT_PERT, 'State variation, optimal case', 'x [m] ; y [m] ; theta [rad/s]', timeVec, linewidth, colors, counter)
% % Plot input (vector u).
% plot_function(u_OPT_PERT, 'Input variation, optimal case', 'wr [rad/s] ; wl [rad/s]', timeVec, linewidth, colors, counter)
% % Plot errors.
% plot_function(e_OPT_PERT', 'Error variation, optimal case', 'e_x [m]; e_y [m]; e_tot [m];e_theta [m]', timeVec, linewidth, colors, counter)
% 
% %% The video function just needs the distance between the wheels in order to plot the robot.
% b_n = perturbed_params(2);
% %video(q_history,opt_traj,b_n,timeVec,linewidth,delta, 'Optimal Trajectory Following')
% 
% figure(3),
% plot(posNonOpt(1,:),posNonOpt(2,:),'Color',colors(10,:),'LineWidth',linewidth, 'DisplayName', 'Nominal Trajectory')
% xlabel("x[m]"), ylabel('y[m]'), grid minor
% title('Trajectory'),fontsize(fontSize,"points"), hold on 
% 
% plot(opt_traj(1,:),opt_traj(2,:),'Color',colors(2,:),'LineStyle','-.','LineWidth',linewidth, 'DisplayName', 'Optimal Trajectory')
% legend('Location', 'southeast')
% 
% figure(4),
% plot(velNonOpt(1,:),velNonOpt(2,:),'Color',colors(12,:),'LineWidth',linewidth, 'DisplayName', 'Nominal Velocity')
% xlabel("xdot[m/s]"), ylabel('ydot[m/s]'), grid minor
% title('Velocity of the Trajectory'),fontsize(fontSize,"points"), hold on 
% 
% plot(opt_vel(1,:),opt_vel(2,:),'Color',colors(2,:),'LineStyle','-.','LineWidth',linewidth, 'DisplayName', 'Optimal Velocity')
% legend ('Location', 'southeast')
% 
% figure(5),
% plot(q_NOPT_PERT(1,:),q_NOPT_PERT(2,:),'Color','r','LineWidth',linewidth, 'DisplayName', 'State for the Nominal Trajectory')
% xlabel("x[m]"), ylabel('y[m]'), grid minor
% title('Compare state of the robot'),fontsize(fontSize,"points"), hold on 
% 
% plot(q_OPT_PERT(1,:),q_OPT_PERT(2,:),'Color','b','LineStyle','-.','LineWidth',linewidth, 'DisplayName', 'State for the Optimal Trajectory') 
% legend ('Location', 'southeast')
% 
% 
% %% Error between the state vectors with the perturbed and nominal parameters, in the optimal case.
% eOptimalVSNonOptimal = zeros(3, Nstep); eTotOptimalVSNonOptimal = zeros(Nstep, 1);
% 
% for i = 1:Nstep
%     eOptimalVSNonOptimal(:, i) = q_OPT_PERT(:, i) - q_OPT_NOM(:, i);
%     eTotOptimalVSNonOptimal(i) = sqrt_of_quadratics(eOptimalVSNonOptimal(:, i));
% end
% 
% plot_function([eOptimalVSNonOptimal;eTotOptimalVSNonOptimal'], 'Error between the state vectors with the perturbed and nominal parameters, in the optimal case.', 'error_x[m]; error_y[m]; error_theta[m]; error_total[m]', timeVec, linewidth, colors, counter)
% 
% %% Error between the state vectors with the perturbed and nominal parameters, in the non-optimal case.
% error_NOPT_PerturbedVSNominal = zeros(3, Nstep); eTot_NOPT_PerturbedVSNominal = zeros(Nstep, 1);
% 
% for i = 1:Nstep
%     error_NOPT_PerturbedVSNominal(:, i) = q_NOPT_PERT(:, i) - q_NOPT_NOM(:, i);
%     eTot_NOPT_PerturbedVSNominal(i) = sqrt_of_quadratics(error_NOPT_PerturbedVSNominal(:, i));
% end
% 
% plot_function([error_NOPT_PerturbedVSNominal;eTot_NOPT_PerturbedVSNominal'], 'Error between the perturbed state and nominal state on the non-optimal trajectory', 'error_x [m];error_y [m];error_theta [m]; error_total[m]', timeVec, linewidth, colors, counter)
% 
% 
% %% Error between the state vectors of the optimal and not-optimal trajectory, in the perturbed parameters.
% error_Perturbed_NOPTvsOPT = zeros(3, Nstep); eTot_Perturbed_NOPTvsOPT = zeros(Nstep, 1);
% 
% for i = 1:Nstep
%     error_Perturbed_NOPTvsOPT(:, i) = q_NOPT_PERT(:, i) - q_OPT_PERT(:, i);
%     eTot_Perturbed_NOPTvsOPT(i) = sqrt_of_quadratics(error_Perturbed_NOPTvsOPT(:, i));
% end
% 
% plot_function([error_Perturbed_NOPTvsOPT; eTot_Perturbed_NOPTvsOPT'], 'Error between the state vectors of the optimal and not-optimal trajectory, in the perturbed parameters.', 'error_x [m];error_y [m];error_theta [m]; error_total[m]', timeVec, linewidth, colors, counter)
% 
% %% Error between the state vectors of the optimal and not-optimal trajectory, in the nominal parameters.
% error_Nominal_d_NOPTvsOPT = zeros(3, Nstep); eTot_Nominal_NOPTvsOPT = zeros(Nstep, 1);
% 
% for i = 1:Nstep
%     error_Nominal_d_NOPTvsOPT(:, i) = q_NOPT_NOM(:, i) - q_OPT_NOM(:, i);
%     eTot_Nominal_NOPTvsOPT(i) = sqrt_of_quadratics(error_Nominal_d_NOPTvsOPT(:, i));
% end
% 
% plot_function([error_Nominal_d_NOPTvsOPT; eTot_Nominal_NOPTvsOPT'], 'Error between the state vectors of the optimal and not-optimal trajectory, in the nominal parameters.', 'error_x [m];error_y [m];error_theta [m]; error_total[m]', timeVec, linewidth, colors, counter)
% 
