close all; clc;
% IDEAL robot system uses the NOMINAL parameters of the robot
params = [wheelRadius; 
          wheelDistance];

%% Desired Trajectory Generation (Spline)
[r_d,dr_d,ddr_d] = trajectory_generation(initialPositionVec, initialVelocityVec, firstBreak, secondBreak,...
                                     finalPositionVec, finalVelocityVec, totalTime, timeVec, linewidth);

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

% u_k = [w_r; w_l].
initialInput = [(initialVelocity + initialAngularVelocity*wheelDistance/2)/wheelRadius;
                (initialVelocity - initialAngularVelocity*wheelDistance/2)/wheelRadius];
   
% u_history = [u0, u1, u2, ..., uN] dimension (2)x(Nstep)
u_history = zeros(2, Nstep); u_history(:,1) = initialInput;
% q_history = [q0, q1, q2, ..., qN] dimension (3)x(Nstep)
q_history = zeros(3, Nstep); q_history(:,1) = initialState;
% Controller state
xhi_history = zeros(3, Nstep); xhi_history(:,1) = [initialVelocity;0.1;0.1];
% Error
e = zeros(2,Nstep); e_tot = zeros(Nstep,1); 

%% IDEAL CONTROL obtained using the NOMINAL parameters inside the robot system.
%  The system parameters are well-known and do not change.
for k=2:Nstep

    % Initilize input and states.
    oldInput = u_history(:,k-1);
    oldState = q_history(:,k-1);
    oldXhi = xhi_history(:,k-1);
    
    % IDEAL robot system outputs the next state of the system.
    currentState = robot_system(oldInput,oldState,delta,params);

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
end

%% Create and display video animation and plots.
% The video function just needs the distance between the wheels in order to plot the robot.
b_n = params(2); 
video(q_history,r_d,b_n,timeVec,linewidth)

% Plot state variables (vector q).
plot_function(q_history,'State variation in time','x [m] ; y [m] ; theta [rad/s]', timeVec, linewidth, colors, counter) 
% Plot input (vector u).
plot_function(u_history,'Input variation in time','wr [rad/s] ; wl [rad/s]', timeVec, linewidth, colors, counter)
% Plot errors.
plot_function([e;e_tot'],'Error variation over time','e_x [m] ; e_y [m] ; e_tot [m]',timeVec, linewidth, colors, counter)

% Don't save anything since it's not used in the optimization routine.