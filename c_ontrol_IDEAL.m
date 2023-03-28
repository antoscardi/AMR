close all; clc;
%% Desired Trajectory Generation (Spline)
[p, dp, ddp] = trajectory_generation(initialPosition, initialVelocity, firstBreak, secondBreak,...
                                     finalPosition, finalVelocity, totalTime, timeVector, linewidth);

%% Inizializations
% Error
e = zeros(2,Nstep); e_tot = zeros(Nstep,1);
% Initial state (x,y,theta)
initialTheta = atan2(initialVelocity(2),initialVelocity(1));
initialState = [initialPosition(1);initialPosition(2);initialTheta];
% Initial input (v,omega) through flatness
[desideredLinearVelocity(1), desideredAngularVelocity(1)] = flatness(dp,ddp);
initialVelocity = [desideredLinearVelocity(1); desideredAngularVelocity(1)];
% u_k=[w_r; w_l]
initialInput = [(initialVelocity(1)+initialVelocity(2)*wheelDistance/2)/wheelRadius;
       (initialVelocity(1)-initialVelocity(2)*wheelDistance/2)/wheelRadius];
   
% u_history = [u0, u1, u2, ..., uN-1] dimension (2)x(Nstep)
u_history = zeros(2, Nstep); u_history(:,1) = initialInput;
v_history = zeros(2, Nstep); v_history(:,1) = initialVelocity;
% q_history = [q0, q1, q2, ..., qN] dimension (3)x(Nstep)
q_history = zeros(3, Nstep); q_history(:,1) = initialState;
% Controller state
xhi_history = zeros(3, Nstep); xhi_history(:,1) = [0.1;0.1;0.1];

%% Simulation loop
for k=1:Nstep

    % Initilize input and state
    currentInput = u_history(:,k);
    currentState = q_history(:,k);
    curretXhi = xhi_history(:,k);

    % Error calculation
    e(:,k) = abs(p(:,k)-currentState(1:2));
    e_tot(k) = sqrt(e(1,k)^2+e(2,k)^2);
    
    %IDEAL robot system uses the NOMINAL parameters of the robot
    params = [wheelRadius;
             wheelDistance];
    nextState = robot_system(currentInput,currentState,delta,params);

    % CONTROLLER
 
    % Save xk and uk as last columns.
    q_history(:,k+1) = nextState;
    u_history(:,k+1) = nextInput;
    xhi_history(:,k+1) = nextXhi;
end

%% Create and display video animation.

video(q_history,p,params(2,:),time)


%% Plots

% Plot state variables (vector q). 
figure 
s = stackedplot(time(1:end),q_history','LineWidth',linewidth);
s.DisplayLabels = ["x [m] ","y [m]",'theta [rad/s]']; grid on
for i=1:3
s.LineProperties(i).Color = colors(i,:);
end
xlabel("time [s]"), title('State variation in time')

% Plot input (vector u).
figure
s = stackedplot(time(1:end),u_history','LineWidth',linewidth);
s.DisplayLabels = ["wr [rad/s]", "wl [rad/s]"];grid on
for i=1:2
s.LineProperties(i).Color = colors(3+i,:);
end
xlabel("time [s]"), title('Input variation in time')

% Plot velocities (vector v).
figure
s = stackedplot(time(1:end),v_history','LineWidth',linewidth);
s.DisplayLabels = ["v [m/s]", "omega [rad/s]"];grid on
for i=1:2
s.LineProperties(i).Color = colors(7+i,:);
end
xlabel("time [s]"), title('Velocities variation in time')

% Plot errors.
figure
s = stackedplot(time(1:end),[e;e_tot']','LineWidth',linewidth);
s.DisplayLabels = ['e_x [m]',"e_y [m]","e_tot [m]"]; grid on
for i=1:3
s.LineProperties(i).Color = colors(9+i,:);
end
xlabel("time [s]"), title('Error variation over time')

% % Plot state derivative (q_dot). 
% figure 
% s = stackedplot(time(1:end),q_dot_history','LineWidth',linewidth);
% s.DisplayLabels = ["x_dot [m] ","y_dot [m]",'theta_dot [rad/s]']; grid on
% for i=1:3
% s.LineProperties(i).Color = colors(i,:);
% end
% xlabel("time [s]"), title('State derivation variation in time')

%% Save variables for the optimization routine.
if followOptim == false && haveNoise == false
    save('data/controlwithnominalparams','u_history','p','q_history','dp','ddp','xhi_history')
end

if haveNoise == true && followOptim == false
    save('data/controlwithnoise','u_history','p','q_history','dp','ddp','xhi_history')
end