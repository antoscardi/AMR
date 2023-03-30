close all; clc;
% IDEAL parameters ONLY used by the CONTROLLER.
params = [wheelRadius; wheelDistance];
% REAL PARAMS
% Generate uniform distribution and sample the exstremes: 80% or 120% of the nominal value
wheelRadius80 = wheelRadius*0.8;
wheelRadius120 = wheelRadius*1.2;
wheelDistance80 = wheelDistance*0.8;
wheelDistance120 = wheelDistance*1.2;

% Choose if you want to see the case in which the params are the 80% or the 120% of the nominal values.
choose = input('Enter either 80 or 120: ');
switch choose
    case 80
        %% FIRST CASE
        caseParams80 = true;
        perturbed_params = [wheelRadius80; wheelDistance80];
        disp('You have choosen to use 80% of the nominal value')
    case 120
        %% SECOND CASE
        caseParams120 = true;
        perturbed_params = [wheelRadius120; wheelDistance120];
        disp('You choose to use 120% of the nominal value')
    otherwise
        disp('You did not choose anything')
end

%% REAL CONTROL in the PERTURBED case, where the parameters of the robot differ from the nominal ones.
[r_d,dr_d,ddr_d] = trajectory_generation(initialPositionVec, initialVelocityVec, firstBreak, secondBreak,...
                                         finalPositionVec, finalVelocityVec, totalTime, timeVec, linewidth);

initialTheta = atan2(initialVelocityVec(2),initialVelocityVec(1));
initialState = [initialPositionVec(1);initialPositionVec(2);initialTheta];
[initialVelocity, initialAngularVelocity] = flatness(dr_d, ddr_d);
initialInput = [(initialVelocity + initialAngularVelocity*wheelDistance/2)/wheelRadius;
                (initialVelocity - initialAngularVelocity*wheelDistance/2)/wheelRadius];

% Reinitialize states and input and errors
q_history = zeros(3, Nstep); q_history(:,1) = initialState;
u_history = zeros(2, Nstep); u_history(:,1) = initialInput;
xhi_history = zeros(3, Nstep); xhi_history(:,1) = [initialVelocity;0.1;0.1];
e = zeros(2,Nstep); e_tot = zeros(Nstep,1);

for k=2:Nstep

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
end

%% Create and display video animation and plots.
% The video function just needs the distance between the wheels in order to plot the robot.
b_n = perturbed_params(2); 
video(q_history,r_d,b_n,timeVec,linewidth)

% Plot state variables (vector q).
plot_function(q_history,'State variation in time','x [m] ; y [m] ; theta [rad/s]', timeVec, linewidth, colors, counter) 
% Plot input (vector u).
plot_function(u_history,'Input variation in time','wr [rad/s] ; wl [rad/s]', timeVec, linewidth, colors, counter)
% Plot errors.
plot_function([e;e_tot'],'Error variation over time','e_x [m] ; e_y [m] ; e_tot [m]',timeVec, linewidth, colors, counter)

% %% Save variables for the optimization routine.
% if caseParams80 == true
% save('data/REALcontrolpertubed80%','u_history','q_history','xhi_history')
% end
% if caseParams120 == true
% save('data/REALcontrolpertubed120%','u_history','q_history','xhi_history')
% end

