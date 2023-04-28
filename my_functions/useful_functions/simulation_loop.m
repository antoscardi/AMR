function [q_evolution,u_evolution,xhi_evolution,e_evolution] = simulation_loop(initialPositionVec,initialVelocityVec,...
                                                                               delta,...
                                                                               nominal_params, perturbed_params, doPerturbation,...
                                                                               r_d, dr_d, ddr_d,theta_d, kv,ki,kp)

% Choose to have perturbed parameters or not. 
if doPerturbation == true
    params = perturbed_params;
end
if doPerturbation == false
    params = nominal_params;
end

% Setting the Nstep to be the same as the size of the trajectory.
[~,Nstep] = size(r_d);

%% Inizializations
% Initial state (x,y,theta)
initialTheta = theta_d(1);
initialState = [initialPositionVec(1); initialPositionVec(2); initialTheta];
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
initialInput = [(initialVelocity + initialAngularVelocity*nominal_params(2)/2)/nominal_params(1);
                (initialVelocity - initialAngularVelocity*nominal_params(2)/2)/nominal_params(1)];
   
% u_history = [u0, u1, u2, ..., uN] dimension (2)x(Nstep)
u_history = zeros(2, Nstep); u_history(:,1) = initialInput;
% q_history = [q0, q1, q2, ..., qN] dimension (3)x(Nstep)
q_history = zeros(3, Nstep); q_history(:,1) = initialState;
% Controller state
xhi_history = zeros(3, Nstep); xhi_history(:,1) = [initialVelocity;0.03;0.03];
% Error
e = zeros(2,Nstep); e_tot = zeros(Nstep,1); e_theta=zeros(Nstep,1);

%% IDEAL CONTROL obtained using the NOMINAL parameters inside the robot system.
%  The system parameters are well-known and do not change.
for k=2:Nstep

    % Initilize input and states.
    oldInput = u_history(:,k-1);
    oldState = q_history(:,k-1);
    oldXhi = xhi_history(:,k-1);
    % IDEAL robot system outputs the next state of the system.
    currentState = robot_system(oldInput,oldState,delta,params,kv,ki,kp);

    % CONTROLLER block, to avoid error always set this to the nominal ones.
    oldDesiredPos = r_d(:,k-1); oldDesiredVel = dr_d(:,k-1); oldDesiredAcc = ddr_d(:,k-1);
    [currentInput,currentXhi] = controller(oldState,oldDesiredPos,oldDesiredVel,oldDesiredAcc,oldXhi,delta,nominal_params,kv,ki,kp);
 
    % Save q_k+1,u_k+1 and xhi_k+1 as last columns.
    q_history(:,k) = currentState;
    u_history(:,k) = currentInput;
    xhi_history(:,k) = currentXhi;

    % Error calculation.
    currentDesiredPos = r_d(:,k);
    e(:,k) = abs(currentDesiredPos - currentState(1:2));
    e_tot(k) = sqrt_of_quadratics(e(:,k));
    e_theta(k) = abs(theta_d(k)- q_history(3,k));

end

q_evolution = q_history;
xhi_evolution = xhi_history;
u_evolution = u_history;
e_evolution = [e',e_tot,e_theta];
end 