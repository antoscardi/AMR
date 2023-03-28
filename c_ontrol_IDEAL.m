close all; clc;
% IDEAL robot system uses the NOMINAL parameters of the robot
params = [wheelRadius; 
          wheelDistance];

%% Desired Trajectory Generation (Spline)
[p, dp, ddp] = trajectory_generation(initialPositionVec, initialVelocityVec, firstBreak, secondBreak,...
                                     finalPositionVec, finalVelocityVec, totalTime, timeVec, linewidth);

%% Inizializations
% Initial state (x,y,theta)
initialTheta = atan2(initialVelocityVec(2),initialVelocityVec(1));
initialState = [initialPositionVec(1);initialPositionVec(2);initialTheta];
initialVelocity = sqrt_of_quadratics(initialVelocityVec);
% Initial input (v,omega) through flatness
[initialVelocity_flatness, initialAngularVelocity] = flatness(dp,ddp);

% Check that initial velocities are equal if calculated from flatness with the given ones
if abs(initialVelocity_flatness - initialVelocity) < 0.000001
    disp("All good")
else 
    disp("There is a problem")
end

% u_k=[w_r; w_l]
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

%% Simulation loop
for k=1:Nstep-1

    % Initilize input and states
    currentInput = u_history(:,k);
    currentState = q_history(:,k);
    currentXhi = xhi_history(:,k);
    
    % IDEAL robot system outputs the next state of the system
    nextState = robot_system(currentInput,currentState,delta,params);

    % CONTROLLER block, to avoid error always set this to the nominal ones
    nominal_params = [wheelRadius; wheelDistance];
    [nextInput,nextXhi] = controller(nextState,p,dp,ddp,currentXhi,delta,nominal_params,k);
 
    % Save q_k+1,u_k+1 and xhi_k+1 as last columns.
    q_history(:,k+1) = nextState;
    u_history(:,k+1) = nextInput;
    xhi_history(:,k+1) = nextXhi;

    % Error calculation, done at the next step since the first is zero
    e(:,k+1) = abs(p(:,k+1)- nextState(1:2));
    e_tot(k+1) = sqrt_of_quadratics(e(:,k+1));
end


%% Create and display video animation and plots.
% The video function just needs the distance between the wheels in order to plot the robot
b_n = params(2); 
%video(q_history,p,b_n,timeVec)

% Plot state variables (vector q).
plot_function(q_history,'State variation in time','x [m] ; y [m] ; theta [rad/s]', timeVec, linewidth, colors, counter) 

% Plot input (vector u).
plot_function(u_history,'Input variation in time','wr [rad/s] ; wl [rad/s]', timeVec, linewidth, colors, counter)

% Plot errors.
plot_function([e;e_tot'],'Error variation over time','e_x [m] ; e_y [m] ; e_tot [m]',timeVec, linewidth, colors, counter)

%% Save variables for the optimization routine.
save('data/IDEALcontrolwithnominalparams','u_history','p','q_history','dp','ddp','xhi_history')
