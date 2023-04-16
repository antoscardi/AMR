close all; clc;

fileCoeff = 'data/coeff_a_star';
dataCoefficients = load(fileCoeff,'ax_star','ay_star');
ax = dataCoefficients.ax_star;
ay = dataCoefficients.ay_star; 
optimizMatrix= [ax,ay];

%% Desired Trajectory Generation (Spline)
% Generate coefficients
[coeffMatrix, ~] = coeff_generation(totalTime, dx, dy);
%% Desired Trajectory Generation (Spline)
% Generate trajectory

[r_d, dr_d, ddr_d] = trajectory_generation(coeffMatrix, timeVec, totalTime, ...
                                            linewidth, colors, false);

%% CONTROL with REAL PARAMS of the OPTIMIZED TRAJECTORY
%% Optimized trajectory 
[opt_traj,opt_vel,opt_acc] = trajectory_generation(optimizMatrix, timeVec, totalTime,...
                                                   linewidth, colors, false);
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

figure(3),
plot(r_d(1,:),r_d(2,:),'Color',colors(10,:),'LineWidth',linewidth, 'DisplayName', 'Nominal Trajectory')
xlabel("x[m]"), ylabel('y[m]'), grid minor
title('Trajectory'),fontsize(fontSize,"points"), hold on 

plot(opt_traj(1,:),opt_traj(2,:),'Color',colors(2,:),'LineStyle','-.','LineWidth',linewidth, 'DisplayName', 'Optimal Trajectory')
legend('Location', 'southeast')

figure(4),
plot(dr_d(1,:),dr_d(2,:),'Color',colors(12,:),'LineWidth',linewidth, 'DisplayName', 'Nominal Velocity')
xlabel("xdot[m/s]"), ylabel('ydot[m/s]'), grid minor
title('Velocity of the Trajectory'),fontsize(fontSize,"points"), hold on 

plot(opt_vel(1,:),opt_vel(2,:),'Color',colors(2,:),'LineStyle','-.','LineWidth',linewidth, 'DisplayName', 'Optimal Velocity')
legend ('Location', 'southeast')

figure(5),
plot(q_history_NO(1,:),q_history_NO(2,:),'Color','r','LineWidth',linewidth, 'DisplayName', 'State for the Nominal Trajectory')
xlabel("x[m]"), ylabel('y[m]'), grid minor
title('Compare state of the robot'),fontsize(fontSize,"points"), hold on 

plot(q_history(1,:),q_history(2,:),'Color','b','LineStyle','-.','LineWidth',linewidth, 'DisplayName', 'State for the Optimal Trajectory') 
legend ('Location', 'southeast')


