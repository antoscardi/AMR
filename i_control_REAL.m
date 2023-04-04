close all; clc;
% IDEAL parameters ONLY used by the CONTROLLER.
nominal_params = [wheelRadius; wheelDistance];
% REAL PARAMS
% Generate uniform distribution and sample the exstremes: 80% or 120% of the nominal value
[chosenCase, perturbed_params] = switch_case(wheelDistance, wheelRadius);

%% Nominal trajectory
nominal_trajectory = load('desired_trajectory','p');
r_d = nominal_trajectory.p;

%% REAL CONTROL obtained using the REAL parameters inside the robot system of the NOMINAL trajectory.
if chosenCase == 120
    real120 = load('REALcontrolperturbed120%','q_history','u_history','xhi_history','all_e'); 
    q_history = real120.q_history;
    u_history = real120.u_history;
    xhi_history = real120.xhi_history;
    all_e = real120.all_e;
end
if chosenCase == 80
    real80 = load('REALcontrolperturbed80%','q_history','u_history','xhi_history','all_e'); 
    q_history = real80.q_history;
    u_history = real80.u_history;
    xhi_history = real80.xhi_history;
    all_e = real80.all_e;
end

%% Create and display video animation and plots for the NON-Optimal trajectory.
% Plot state variables (vector q).
plot_function(q_history,'State variation, non-optimal case','x [m] ; y [m] ; theta [rad/s]', timeVec, linewidth, colors, counter) 
% Plot input (vector u).
plot_function(u_history,'Input variation, non-optimal case','wr [rad/s] ; wl [rad/s]', timeVec, linewidth, colors, counter)
% Plot errors.
plot_function(all_e,'Error variation, non-optimal case','e_x [m] ; e_y [m] ; e_tot [m]',timeVec, linewidth, colors, counter)

% The video function just needs the distance between the wheels in order to plot the robot.
b_n = perturbed_params(2); 
video(q_history,r_d,b_n,timeVec,linewidth,delta, 'Non Optimized Trajectory Following')

%% CONTROL with REAL PARAMS of the OPTIMIZED TRAJECTORY
%% Optimized trajectory 
file = 'data/new_traj';
optimTraj = load(file,'p','dp','ddp');
optimr_d = optimTraj.p; optimdr_d = optimTraj.dp; optimddr_d = optimTraj.ddp;

%% Inizializations
% Initial state (x,y,theta)
initialTheta = atan2(initialVelocityVec(2),initialVelocityVec(1));
initialState = [initialPositionVec(1);initialPositionVec(2);initialTheta];
initialVelocity = sqrt_of_quadratics(initialVelocityVec);

% Optimized case
[optimInitialVelocity_flatness, optimInitialAngularVelocity] = flatness(optimdr_d, optimddr_d);
% Check that initial velocities are equal if calculated from flatness with the given ones
if abs(optimInitialVelocity_flatness - initialVelocity) < 0.000001
    disp("All good, the velocities coincide also in the optimal case")
else 
    disp("There is a problem, the two velocities do not coincide")
end

%% Vectors initializations
% u_k = [w_r; w_l].
initialInput = [(initialVelocity + optimInitialAngularVelocity*wheelDistance/2)/wheelRadius;
                (initialVelocity - optimInitialAngularVelocity*wheelDistance/2)/wheelRadius];

% u_history = [u0, u1, u2, ..., uN] dimension (2)x(Nstep)
u_history_opt = zeros(2, Nstep); u_history_opt(:,1) = initialInput;
% q_history = [q0, q1, q2, ..., qN] dimension (3)x(Nstep)
q_history_opt = zeros(3, Nstep); q_history_opt(:,1) = initialState;
% Controller state
xhi_history_opt = zeros(3, Nstep); xhi_history_opt(:,1) = [initialVelocity;0.1;0.1];
% Error
e_opt = zeros(2,Nstep); e_tot_opt = zeros(Nstep,1);

% Simulation loop
for k=2:Nstep
    % Initilize input and states.
    oldInput_opt = u_history_opt(:,k-1);
    oldState_opt = q_history_opt(:,k-1);
    oldXhi_opt = xhi_history_opt(:,k-1);

    % IDEAL robot system outputs the next state of the system.
    currentState_opt = robot_system(oldInput_opt,oldState_opt,delta,perturbed_params);

    % CONTROLLER block, to avoid error always set this to the nominal ones.
    oldDesiredPos_opt = optimr_d(:,k-1); oldDesiredVel_opt = optimdr_d(:,k-1); oldDesiredAcc_opt = optimddr_d(:,k-1);
    [currentInput_opt,currentXhi_opt] = controller(oldState_opt,oldDesiredPos_opt,oldDesiredVel_opt,oldDesiredAcc_opt,oldXhi_opt,delta,nominal_params);

    % Save q_k+1,u_k+1 and xhi_k+1 as last columns.
    q_history_opt(:,k) = currentState_opt;
    u_history_opt(:,k) = currentInput_opt;
    xhi_history_opt(:,k) = currentXhi_opt;

    % Error calculation.
    currentDesiredPos_opt = optimr_d(:,k);
    e_opt(:,k) = abs(currentDesiredPos_opt- currentState_opt(1:2));
    e_tot_opt(k) = sqrt_of_quadratics(e_opt(:,k));
end

%% Create and display video animation and plots for the OPTIMAL trajectory.
% Plot state variables (vector q).
plot_function(q_history_opt,'State variation, optimal case','x [m] ; y [m] ; theta [rad/s]', timeVec, linewidth, colors, counter) 
% Plot input (vector u).
plot_function(u_history_opt,'Input variation, optimal case','wr [rad/s] ; wl [rad/s]', timeVec, linewidth, colors, counter)
% Plot errors.
plot_function([e_opt; e_tot_opt'],'Error variation, optimal case','e_x [m] ; e_y [m] ; e_tot [m]',timeVec, linewidth, colors, counter)

% The video function just needs the distance between the wheels in order to plot the robot.
b_n = perturbed_params(2); 
video(q_history_opt,optimr_d,b_n,timeVec,linewidth,delta, 'Optimal Trajectory Following')


