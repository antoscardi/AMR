close all; clc;

%% Desired Trajectory Generation (Spline)
% Generate coefficients
[coeffMatrix, ~] = coeff_generation(totalTime, dx, dy);
%% Desired Trajectory Generation (Spline)
% Generate trajectory
doPlotsSave = true;
[r_d, dr_d, ddr_d] = trajectory_generation(coeffMatrix, timeVec, totalTime, ...
    linewidth, colors, doPlotsSave);

%% CONTROL with REAL PARAMS of the OPTIMIZED TRAJECTORY
%% Optimized trajectory 
[opt_traj,opt_vel,opt_acc] = trajectory_generation(optimizedCoeffMatrix, timeVec, totalTime,...
                                                   linewidth, colors, true);
doPerturbation = true;
[q_history, u_history, xhi_history, e] = simulation_loop(initialPositionVec, initialVelocityVec, ...
                                                            delta, ...
                                                            nominal_params, perturbed_params, doPerturbation, ...
                                                            opt_traj, opt_vel, opt_acc);
[q_history_NO, u_history_NO, xhi_history_NO, e_NO] = simulation_loop(initialPositionVec, initialVelocityVec, ...
                                                            delta, ...
                                                            nominal_params, perturbed_params, doPerturbation, ...
                                                            r_d, dr_d, ddr_d);
%% Create and display video animation and plots for the NON-Optimal trajectory.
% Plot state variables (vector q).
plot_function(q_history_NO,'State variation, non-optimal case','x [m] ; y [m] ; theta [rad/s]', timeVec, linewidth, colors, counter) 
% % Plot input (vector u).
plot_function(u_history_NO,'Input variation, non-optimal case','wr [rad/s] ; wl [rad/s]', timeVec, linewidth, colors, counter)
% % Plot errors.
plot_function(e_NO','Error variation, non-optimal case','e_x [m]; e_y [m]; e_tot [m];e_theta [m]',timeVec, linewidth, colors, counter)

% % The video function just needs the distance between the wheels in order to plot the robot.
b_n = perturbed_params(2); 
video(q_history,r_d,b_n,timeVec,linewidth,delta, 'Non Optimized Trajectory Following')

%% Create and display video animation and plots for the OPTIMAL trajectory.
% Plot state variables (vector q).
plot_function(q_history,'State variation, optimal case','x [m] ; y [m] ; theta [rad/s]', timeVec, linewidth, colors, counter) 
% Plot input (vector u).
plot_function(u_history,'Input variation, optimal case','wr [rad/s] ; wl [rad/s]', timeVec, linewidth, colors, counter)
% Plot errors.
plot_function(e','Error variation, optimal case','e_x [m]; e_y [m]; e_tot [m];e_theta [m]',timeVec, linewidth, colors, counter)

% The video function just needs the distance between the wheels in order to plot the robot.
b_n = perturbed_params(2); 
video(q_history,opt_traj,b_n,timeVec,linewidth,delta, 'Optimal Trajectory Following')
