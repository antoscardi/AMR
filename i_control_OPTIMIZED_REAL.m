close all; clc;
% IDEAL parameters ONLY used by the CONTROLLER.
nominal_params = [wheelRadius; wheelDistance];
% REAL PARAMS
% Generate uniform distribution and sample the exstremes: 80% or 120% of the nominal value
[chosenCase, perturbed_params] = switch_case(wheelDistance, wheelRadius);

%% Desired Trajectory Generation (Spline)
[r_d,dr_d,ddr_d] = trajectory_generation(initialPositionVec, initialVelocityVec, firstBreak, secondBreak,...
                                         finalPositionVec, finalVelocityVec,velocityFirstBreak, velocitySecondBreak,...
                                         totalTime, timeVec, linewidth,colors);

%% CONTROL with REAL PARAMS of the OPTIMIZED TRAJECTORY
%% Optimized trajectory 
file = 'data/new_traj';
optimTraj = load(file,'r','dr','ddr');
r_d_opt = optimTraj.r; dr_d_opt = optimTraj.dr; ddr_d_opt = optimTraj.ddr;

%% Inizializations
% Initial state (x,y,theta)
initialTheta = atan2(initialVelocityVec(2),initialVelocityVec(1));
initialState = [initialPositionVec(1);initialPositionVec(2);initialTheta];
initialVelocity = sqrt_of_quadratics(initialVelocityVec);

% Initial input (v,omega) through flatness
[initialVelocity_flatness, initialAngularVelocity] = flatness(dr_d, ddr_d);
% Check that initial velocities are equal if calculated from flatness with the given ones
if abs(initialVelocity_flatness - initialVelocity) < 0.000001
    disp("All good, the velocities coincide")
else 
    disp("There is a problem, the two velocities do not coincide")
end
% Optimized case
[optimInitialVelocity_flatness, optimInitialAngularVelocity] = flatness(dr_d_opt, ddr_d_opt);
% Check that initial velocities are equal if calculated from flatness with the given ones
if abs(optimInitialVelocity_flatness - initialVelocity) < 0.000001
    disp("All good, the velocities coincide also in the optimal case")
else 
    disp("There is a problem, the two velocities do not coincide")
end

% u_k = [w_r; w_l].
initialInput = [(initialVelocity + initialAngularVelocity*wheelDistance/2)/wheelRadius;
                (initialVelocity - initialAngularVelocity*wheelDistance/2)/wheelRadius];
   
% u_history = [u0, u1, u2, ..., uN] dimension (2)x(Nstep)
u_history = zeros(2, Nstep); u_history(:,1) = initialInput;
u_history_opt = zeros(2, Nstep); u_history_opt(:,1) = initialInput;
% q_history = [q0, q1, q2, ..., qN] dimension (3)x(Nstep)
q_history = zeros(3, Nstep); q_history(:,1) = initialState;
q_history_opt = zeros(3, Nstep); q_history_opt(:,1) = initialState;
% Controller state
xhi_history = zeros(3, Nstep); xhi_history(:,1) = [initialVelocity;0.03;0.03];
xhi_history_opt = zeros(3, Nstep); xhi_history_opt(:,1) = [initialVelocity;0.03;0.03];
% Error
e = zeros(2,Nstep); e_tot = zeros(Nstep,1); 
e_opt = zeros(2,Nstep); e_tot_opt = zeros(Nstep,1); 

%% IDEAL CONTROL obtained using the NOMINAL parameters inside the robot system.
%  The system parameters are well-known and do not change.
for k=2:Nstep
    %% Control of the NOMINAL trajectory

    % Initilize input and states.
    oldInput = u_history(:,k-1);
    oldState = q_history(:,k-1);
    oldXhi = xhi_history(:,k-1);
    
    % IDEAL robot system outputs the next state of the system.
    currentState = robot_system(oldInput,oldState,delta,perturbed_params);

    % CONTROLLER block, to avoid error always set this to the nominal ones.
    nominal_params = [wheelRadius; wheelDistance];
    oldDesiredPos = r_d(:,k-1); oldDesiredVel = dr_d(:,k-1); oldDesiredAcc = ddr_d(:,k-1);
    [currentInput,currentXhi] = controller(oldState,oldDesiredPos,oldDesiredVel,oldDesiredAcc,oldXhi,delta,nominal_params);
 
    % Save q_k+1,u_k+1 and xhi_k+1 as last columns.
    q_history(:,k) = currentState;
    u_history(:,k) = currentInput;
    xhi_history(:,k) = currentXhi;

    % Error calculation.
    currentDesiredPos = r_d(:,k);
    e(:,k) = abs(currentDesiredPos- currentState(1:2));
    e_tot(k) = sqrt_of_quadratics(e(:,k));
    
    
    %% Control of the OPTIMAL Trajectory

    % Initilize input and states.
    oldInput_opt = u_history_opt(:,k-1);
    oldState_opt = q_history_opt(:,k-1);
    oldXhi_opt = xhi_history_opt(:,k-1);
    
    % IDEAL robot system outputs the next state of the system.
    currentState_opt = robot_system(oldInput_opt,oldState_opt,delta,perturbed_params);

    % CONTROLLER block, to avoid error always set this to the nominal ones.
    nominal_params = [wheelRadius; wheelDistance];
    oldDesiredPos_opt = r_d_opt(:,k-1); oldDesiredVel_opt = dr_d_opt(:,k-1); oldDesiredAcc_opt = ddr_d_opt(:,k-1);
    [currentInput_opt,currentXhi_opt] = controller(oldState_opt,oldDesiredPos_opt,oldDesiredVel_opt,oldDesiredAcc_opt,oldXhi_opt,delta,nominal_params);
 
    % Save q_k+1,u_k+1 and xhi_k+1 as last columns.
    q_history_opt(:,k) = currentState_opt;
    u_history_opt(:,k) = currentInput_opt;
    xhi_history_opt(:,k) = currentXhi_opt;

    % Error calculation.
    currentDesiredPos_opt = r_d_opt(:,k);
    e_opt(:,k) = abs(currentDesiredPos_opt- currentState_opt(1:2));
    e_tot_opt(k) = sqrt_of_quadratics(e_opt(:,k));
end

%% Create and display video animation and plots for the NON-Optimal trajectory.
% Plot state variables (vector q).
plot_function(q_history,'State variation, non-optimal case','x [m] ; y [m] ; theta [rad/s]', timeVec, linewidth, colors, counter) 
% Plot input (vector u).
plot_function(u_history,'Input variation, non-optimal case','wr [rad/s] ; wl [rad/s]', timeVec, linewidth, colors, counter)
% Plot errors.
plot_function([e; e_tot'],'Error variation, non-optimal case','e_x [m] ; e_y [m] ; e_tot [m]',timeVec, linewidth, colors, counter)

% The video function just needs the distance between the wheels in order to plot the robot.
b_n = perturbed_params(2); 
video(q_history,r_d,b_n,timeVec,linewidth,delta, 'Non Optimized Trajectory Following')

%% Create and display video animation and plots for the OPTIMAL trajectory.
% Plot state variables (vector q).
plot_function(q_history_opt,'State variation, optimal case','x [m] ; y [m] ; theta [rad/s]', timeVec, linewidth, colors, counter) 
% Plot input (vector u).
plot_function(u_history_opt,'Input variation, optimal case','wr [rad/s] ; wl [rad/s]', timeVec, linewidth, colors, counter)
% Plot errors.
plot_function([e_opt; e_tot_opt'],'Error variation, optimal case','e_x [m] ; e_y [m] ; e_tot [m]',timeVec, linewidth, colors, counter)

% The video function just needs the distance between the wheels in order to plot the robot.
b_n = perturbed_params(2); 
video(q_history_opt,r_d_opt,b_n,timeVec,linewidth,delta, 'Optimal Trajectory Following')

