close all; clc;
%% IDEAL CONTROL
doPerturbation = false;

%% Desired Trajectory Generation (Spline)
% Generate coefficients
[coeffMatrix,~] = coeff_generation(totalTime, dx, dy);

% Generate trajectory
doPlotsSave = true;
[r_d,dr_d,ddr_d] = trajectory_generation(coeffMatrix, timeVec, totalTime,...
                                         linewidth, colors, doPlotsSave);

% Run the simulation loop in the IDEAL CASE
[q_history,u_history,xhi_history,e] = simulation_loop(initialPositionVec,initialVelocityVec,...
                                                      delta,...
                                                      nominal_params, perturbed_params, doPerturbation,...
                                                      r_d,dr_d,ddr_d);

%% Create and display video animation and plots.
% Plot comparison between state variables (vector q) and desired state.
plot_function([q_history(1,:); r_d(1,:)],'Comparison of x in time','x [m] ; x_des [m]', timeVec, linewidth, colors, counter) 
plot_function([q_history(2,:); r_d(2,:)],'Comparison of y in time','y [m] ; y_des [m]', timeVec, linewidth, colors, counter) 
%plot_function([q_history(3,:); desiredTheta_time'],'Comparison of theta in time','theta [rad] ; theta_des [rad]', timeVec, linewidth, colors, counter)
% Plot input (vector u).
plot_function(u_history,'Input variation in time','wr [rad/s] ; wl [rad/s]', timeVec, linewidth, colors, counter)
% Plot errors.
plot_function(e','Error variation over time','e_x [m] ; e_y [m] ; e_tot [m]; e_theta [rad]',timeVec, linewidth, colors, counter)

% The video function just needs the distance between the wheels in order to plot the robot.
b_n = nominal_params(2); % we are using the nominal params
video(q_history,r_d,b_n,timeVec,linewidth,delta,'Ideal Control')

% Save the ideal control data for the OPTIMIZATION.
save('data/IDEALcontrol','u_history','q_history','xhi_history')