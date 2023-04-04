close all; clc;
% Load from data
file = 'data/IDEALcontrol2';
%file1 = 'data/desired_trajectory';

params=[wheelRadius; wheelDistance];

u_history = load(file,'u_history');
xhi_history = load(file,'xhi_history');
q_history = load(file,'q_history');
desired_traj = load(file,'traj'); 
dp = load(file,'dtraj'); 
ddp = load(file,'ddtraj');

% Create variables
q_history = q_history.q_history; 
u_history = u_history.u_history;
desired_traj = desired_traj.traj; 
dp = dp.dtraj; ddp = ddp.ddtraj; 
xhi_history = xhi_history.xhi_history;

% Make the substitution to compute the values
f_q = zeros(3,3,Nstep,Nstep); f_p = zeros(3,1,Nstep,Nstep); f_u = zeros(3,2,Nstep,Nstep); g_q = zeros(3,3,Nstep,Nstep); g_xhi = zeros(3,3,Nstep); h_xhi = zeros(2,3,Nstep); h_q = zeros(2,3,Nstep,Nstep);
h_ax1 = zeros(2,1,Nstep,Nstep); h_ay1 = zeros(2,1,Nstep,Nstep); h_ax2 = zeros(2,1,Nstep,Nstep); h_ay2= zeros(2,1,Nstep); h_ax3 = zeros(2,1,Nstep); h_ay3 = zeros(2,1,Nstep); h_ax4 = zeros(2,1,Nstep); h_ay4 = zeros(2,1,Nstep);
g_ax1= zeros(3,1,Nstep); g_ay1 = zeros(3,1,Nstep); g_ax2 = zeros(3,1,Nstep); g_ay2 = zeros(3,1,Nstep); g_ax3 = zeros(3,1,Nstep); g_ay3 = zeros(3,1,Nstep); g_ax4 = zeros(3,1,Nstep); g_ay4 = zeros(3,1,Nstep);


% Sensitivity
for j=1:Nstep
for k = 1:Nstep
% Sensitivity
f_p(:,:,k,j) = ff_p(q_history(:,k,j),u_history(:,k,j),params(1:2));
f_q(:,:,k,j) = ff_q(q_history(:,k,j),u_history(:,k,j),params(1:2));
f_u(:,:,k,j) = ff_u(q_history(:,k,j),params(1:2));
h_q(:,:,k,j) = fh_q(q_history(:,k,j),xhi_history(:,k,j),desired_traj(:,k,j),dp(:,k,j),ddp(:,k,j),params(1:2));
h_xhi(:,:,k,j) = fh_xhi(q_history(:,k,j),xhi_history(:,k,j),desired_traj(:,k,j),dp(:,k,j),ddp(:,k,j), params(1:2));
g_q(:,:,k,j) = fg_q(q_history(:,k,j),xhi_history(:,k,j),desired_traj(:,k,j),dp(:,k,j),ddp(:,k,j));
g_xhi(:,:,k,j) = fg_xhi(q_history(:,k,j),xhi_history(:,k,j),desired_traj(:,k,j),dp(:,k,j),ddp(:,k,j));
end
end

% Save sensitivity derivatives for the integration routine.
save('data/sensitivity_deriv2','f_q','f_p','f_u','g_q','g_xhi','h_q','h_xhi')