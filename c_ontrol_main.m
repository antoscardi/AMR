close all; clc;
%% HYPERPARAMETERS:
% Initial position and velocity
p_0 = [2 4];
v_0 = [0.1 0.1];
% Position of the two break points
p_1 = [5 10];
p_2 = [7 15];
% Final position and velocity.
p_f = [10 20];
v_f = [0.1 0.1];

%% Choose if you want to have the disturbance in the robot parameters
doOptimization = true;

%% Choose if you want to follow the optimized trajectory or not
followOptim = false;

%% Desired Trajectory Generation (Spline)
if followOptim == false
    [p, dp, ddp] = trajectory_generation(p_0, v_0, p_1, p_2, p_f, v_f, tsim, time, linewidth);
else
    file = 'data/new_traj';
    p = load(file,'p'); dp = load(file,'dp'); ddp = load(file,'ddp');
    p =p.p; dp = dp.dp; ddp = ddp.ddp; 
end

%% Inizializations
% Error
e = zeros(2,Nstep); e_tot = zeros(Nstep,1);
% Initial state (x,y,theta)
theta_0 = atan2(v_0(2),v_0(1));
q_0 = [p_0(1);p_0(2);theta_0];
% Initial input (v,omega) through flatness
[v_d(1), omega_d(1)] = flatness(dp,ddp);
v_0 = [v_d(1); omega_d(1)]; 
% u_k=[w_r; w_l]
if doOptimization == false
    u_k = [(v_0(1)+v_0(2)*b_n/2)/r_n;
           (v_0(1)-v_0(2)*b_n/2)/r_n];
else
    u_k = [(v_0(1)+v_0(2)*params(2,1)/2)/params(1,1);
           (v_0(1)-v_0(2)*params(2,1)/2)/params(1,1)];
end 
% u_history = [u0, u1, u2, ..., uN-1] dimension (2)x(Nstep)
u_history = zeros(2, Nstep); u_history(:,1) = u_k;
v_history = zeros(2, Nstep); v_history(:,1) = v_0;
% q_history = [q0, q1, q2, ..., qN] dimension (3)x(Nstep)
q_history = zeros(3, Nstep); q_history(:,1) = q_0;
% Controller state
xhi_history = zeros(3, Nstep); xhi_history(:,1) = [0.1;0.1;0.1];
% q_dot
q_dot_history = zeros(3,Nstep);

%% Simulation loop
for k=2:Nstep

    % Initilize input and state
    u_k = u_history(:,k-1);
    q_k = q_history(:,k-1);
    xhi_k = xhi_history(:,k-1);
    
    % Integrate q_dot = f(q,u)
    if doOptimization == false
        q_dot_history(:,k) = q_dot(0,q_k,u_k);
        [~, qint] = ode45(@(t,q) q_dot(t,q,u_k),[0 delta],q_k);
        q_k = qint(end,:)';
    else
        q_dot_history(:,k) = q_dot_opt(0,q_k,u_k,params(:,k-1));
        q_k = q_k + delta*q_dot_opt(0,q_k,u_k,params(:,k-1));
    end

    % Dynamic Feedback Linearization Internal State
    [~, xhi_int] = ode45(@(t,xhi) xhi_dot(t,q_k,xhi,p(:,k),dp(:,k),ddp(:,k)),[0 delta],xhi_k);
    xhi_k = xhi_int(end,:)';
    
    % Change control input for next step
    u_k = new_u(q_k,xhi_k,p(:,k),dp(:,k),ddp(:,k));

    % velocities calculation for comparison
    if doOptimization == false
    v_k = velocities(u_k);
    v_history(:,k) = v_k;
    else
    v_k = velocities_opt(u_k, params(:,k-1));
    v_history(:,k) = v_k;
    end
    % Error calculation
    e(:,k) = abs(p(:,k)-q_k(1:2));
    e_tot(k) = sqrt(e(1,k)^2+e(2,k)^2);

    % Save xk and uk as last columns.
    q_history(:,k) = q_k;
    u_history(:,k) = u_k;
    xhi_history(:,k) = xhi_k;
end

%% Create and display video animation.
video(q_history,p,b_n,time)

%% Plots
colors = linspecer(12,'sequential');
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

% Plot state derivative (q_dot). 
figure 
s = stackedplot(time(1:end),q_dot_history','LineWidth',linewidth);
s.DisplayLabels = ["x_dot [m] ","y_dot [m]",'theta_dot [rad/s]']; grid on
for i=1:3
s.LineProperties(i).Color = colors(i,:);
end
xlabel("time [s]"), title('State derivation variation in time')

%% Save variables for the optimization routine.
if followOptim == false && doOptimization == false
    save('data/controlwithnominalparams','u_history','p','q_history','dp','ddp','xhi_history')
end

if doOptimization == true && followOptim == false
    save('data/controlwithnoise','u_history','p','q_history','dp','ddp','xhi_history')
end 

%% Utility functions
function [v_in, omega_in] = flatness(dp,ddp)
% Flatness formula
v_in = sqrt((dp(1,1))^2+(dp(2,1))^2);
omega_in = (ddp(2,1)*dp(1,1)-ddp(1,1)*dp(2,1))/v_in^2;
end





