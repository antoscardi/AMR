close all; clc;
% Load from data
file = 'data/IDEALcontrol';
file1 = 'data/desired_trajectory';

params=[wheelRadius; wheelDistance];

u_history = load(file,'u_history');
xhi_history = load(file,'xhi_history');
q_history = load(file,'q_history');
desired_traj = load(file1,'p'); 
dp = load(file1,'dp'); 
ddp = load(file1,'ddp');

%  Create variables
q_history = q_history.q_history; 
u_history = u_history.u_history;
desired_traj = desired_traj.p; 
dp = dp.dp; ddp = ddp.ddp; 
xhi_history = xhi_history.xhi_history;

% Make the substitution to compute the values
f_q = zeros(3,3,Nstep); f_p = zeros(3,1,Nstep); f_u = zeros(3,2,Nstep); g_q = zeros(3,3,Nstep); g_xhi = zeros(3,3,Nstep); h_xhi = zeros(2,3,Nstep); h_q = zeros(2,3,Nstep);
h_ax1 = zeros(2,1,Nstep); h_ay1 = zeros(2,1,Nstep); h_ax2 = zeros(2,1,Nstep); h_ay2= zeros(2,1,Nstep); h_ax3 = zeros(2,1,Nstep); h_ay3 = zeros(2,1,Nstep); h_ax4 = zeros(2,1,Nstep); h_ay4 = zeros(2,1,Nstep);
g_ax1= zeros(3,1,Nstep); g_ay1 = zeros(3,1,Nstep); g_ax2 = zeros(3,1,Nstep); g_ay2 = zeros(3,1,Nstep); g_ax3 = zeros(3,1,Nstep); g_ay3 = zeros(3,1,Nstep); g_ax4 = zeros(3,1,Nstep); g_ay4 = zeros(3,1,Nstep);

for k = 1:Nstep
% Sensitivity
f_p(:,:,k) = ff_p(q_history(:,k),u_history(:,k),params(1:2));
f_q(:,:,k) = ff_q(q_history(:,k),u_history(:,k),params(1:2));
f_u(:,:,k) = ff_u(q_history(:,k),params(1:2));
h_q(:,:,k) = fh_q(q_history(:,k),xhi_history(:,k),desired_traj(:,k),dp(:,k),ddp(:,k),params(1:2));
h_xhi(:,:,k) = fh_xhi(q_history(:,k),xhi_history(:,k),desired_traj(:,k),dp(:,k),ddp(:,k), params(1:2));
g_q(:,:,k) = fg_q(q_history(:,k),xhi_history(:,k),desired_traj(:,k),dp(:,k),ddp(:,k));
g_xhi(:,:,k) = fg_xhi(q_history(:,k),xhi_history(:,k),desired_traj(:,k),dp(:,k),ddp(:,k));
% Gamma
h_ax1(:,:,k) = h_a_1x(q_history(:,k),timeVec(k),xhi_history(:,k), params(1:2));
h_ay1(:,:,k) = h_a_1y(q_history(:,k),timeVec(k),xhi_history(:,k), params(1:2));
h_ax2(:,:,k) = h_a_2x(q_history(:,k),timeVec(k),xhi_history(:,k), params(1:2));
h_ay2(:,:,k) = h_a_2y(q_history(:,k),timeVec(k),xhi_history(:,k), params(1:2));
h_ax3(:,:,k) = h_a_3x(q_history(:,k),timeVec(k),xhi_history(:,k), params(1:2));
h_ay3(:,:,k) = h_a_3y(q_history(:,k),timeVec(k),xhi_history(:,k), params(1:2));
h_ax4(:,:,k) = h_a_4x(q_history(:,k),timeVec(k),xhi_history(:,k), params(1:2));
h_ay4(:,:,k) = h_a_4y(q_history(:,k),timeVec(k),xhi_history(:,k), params(1:2));
g_ax1(:,:,k) = g_a_1x(q_history(:,k),timeVec(k));
g_ay1(:,:,k) = g_a_1y(q_history(:,k),timeVec(k));
g_ax2(:,:,k) = g_a_2x(q_history(:,k),timeVec(k));
g_ay2(:,:,k) = g_a_2y(q_history(:,k),timeVec(k));
g_ax3(:,:,k) = g_a_3x(q_history(:,k),timeVec(k));
g_ay3(:,:,k) = g_a_3y(q_history(:,k),timeVec(k));
g_ax4(:,:,k) = g_a_4x(q_history(:,k));
g_ay4(:,:,k) = g_a_4y(q_history(:,k));
end

% Double derivatives respect to the parameters
dhqa1x = zeros(2,3,Nstep); dhqa1y = zeros(2,3,Nstep); dhqa2x = zeros(2,3,Nstep); dhqa2y = zeros(2,3,Nstep); dhqa3x = zeros(2,3,Nstep); dhqa3y = zeros(2,3,Nstep); dhqa4x = zeros(2,3,Nstep); dhqa4y = zeros(2,3,Nstep);
dhxhia1x = zeros(2,3,Nstep); dhxhia1y = zeros(2,3,Nstep); dhxhia2x = zeros(2,3,Nstep); dhxhia2y = zeros(2,3,Nstep); dhxhia3x = zeros(2,3,Nstep); dhxhia3y = zeros(2,3,Nstep); dhxhia4x = zeros(2,3,Nstep); dhxhia4y = zeros(2,3,Nstep); 
dgqa1x = zeros(3,3,Nstep); dgqa1y = zeros(3,3,Nstep); dgqa2x = zeros(3,3,Nstep); dgqa2y = zeros(3,3,Nstep); dgqa3x = zeros(3,3,Nstep); dgqa3y = zeros(3,3,Nstep); dgqa4x = zeros(3,3,Nstep); dgqa4y = zeros(3,3,Nstep); 
dgxhia1x = zeros(3,3,Nstep); dgxhia1y = zeros(3,3,Nstep); dgxhia2x = zeros(3,3,Nstep); dgxhia2y = zeros(3,3,Nstep); dgxhia3x = zeros(3,3,Nstep); dgxhia3y = zeros(3,3,Nstep); dgxhia4x = zeros(3,3,Nstep); dgxhia4y = zeros(3,3,Nstep);
for k=1:Nstep
    dhqa1x(:,:,k) = dhq_a1x(params(2), params(1), timeVec(k), q_history(3,k), xhi_history(1,k));
    dhqa2x(:,:,k) = dhq_a2x(params(2), params(1), timeVec(k), q_history(3,k), xhi_history(1,k));
    dhqa3x(:,:,k) = dhq_a3x(params(2), params(1), timeVec(k), q_history(3,k), xhi_history(1,k));
    dhqa4x(:,:,k) = dhq_a4x(params(2), params(1), q_history(3,k), xhi_history(1,k));
    dhqa1y(:,:,k) = dhq_a1y(params(2), params(1), timeVec(k), q_history(3,k), xhi_history(1,k));
    dhqa2y(:,:,k) = dhq_a2y(params(2), params(1), timeVec(k), q_history(3,k), xhi_history(1,k));
    dhqa3y(:,:,k) = dhq_a3y(params(2), params(1), timeVec(k), q_history(3,k), xhi_history(1,k));
    dhqa4y(:,:,k) = dhq_a4y(params(2), params(1), q_history(3,k), xhi_history(1,k));

    dhxhia1x(:,:,k) = dhxhi_a1x(params(2), params(1), timeVec(k), q_history(3,k), xhi_history(1,k));
    dhxhia2x(:,:,k) = dhxhi_a2x(params(2), params(1), timeVec(k), q_history(3,k), xhi_history(1,k));
    dhxhia3x(:,:,k) = dhxhi_a3x(params(2), params(1), timeVec(k), q_history(3,k), xhi_history(1,k));
    dhxhia4x(:,:,k) = dhxhi_a4x(params(2), params(1), q_history(3,k), xhi_history(1,k));
    dhxhia1y(:,:,k) = dhxhi_a1y(params(2), params(1), timeVec(k), q_history(3,k), xhi_history(1,k));
    dhxhia2y(:,:,k) = dhxhi_a2y(params(2), params(1), timeVec(k), q_history(3,k), xhi_history(1,k));
    dhxhia3y(:,:,k) = dhxhi_a3y(params(2), params(1), timeVec(k), q_history(3,k), xhi_history(1,k));
    dhxhia4y(:,:,k) = dhxhi_a4y(params(2), params(1), q_history(3,k), xhi_history(1,k));

    dgqa1x(:,:,k) = dgq_a1x(timeVec(k), q_history(3,k));
    dgqa2x(:,:,k) = dgq_a2x(timeVec(k), q_history(3,k));
    dgqa3x(:,:,k) = dgq_a3x(timeVec(k), q_history(3,k));
    dgqa4x(:,:,k) = dgq_a4x(q_history(3,k));
    dgqa1y(:,:,k) = dgq_a1y(timeVec(k), q_history(3,k));
    dgqa2y(:,:,k) = dgq_a2y(timeVec(k), q_history(3,k));
    dgqa3y(:,:,k) = dgq_a3y(timeVec(k), q_history(3,k));
    dgqa4y(:,:,k) = dgq_a4y(q_history(3,k));

    dgxhia1x(:,:,k)=dgxhi_a1x();
    dgxhia2x(:,:,k)=dgxhi_a2x();
    dgxhia3x(:,:,k)=dgxhi_a3x();
    dgxhia4x(:,:,k)=dgxhi_a4x();
    dgxhia1y(:,:,k)=dgxhi_a1y();
    dgxhia2y(:,:,k)=dgxhi_a2y();
    dgxhia3y(:,:,k)=dgxhi_a3y();
    dgxhia4y(:,:,k)=dgxhi_a4y();
end


% Save sensitivity derivatives for the integration routine.
save('data/sensitivity_deriv','f_q','f_p','f_u','g_q','g_xhi','h_q','h_xhi')
% Save gamma derivatives for the integration routine.
save('data/gamma_deriv','h_ax1','h_ay1','h_ax2','h_ay2','h_ax3','h_ay3', 'h_ax4','h_ay4',"g_ax1", "g_ay1","g_ax2", "g_ay2", "g_ax3", "g_ay3" , "g_ax4", "g_ay4")
% Save double derivatives for the optimization routine.
all_vars = who; saving = cell(32,1); count = 0;
for i = 1:length(all_vars)
    if startsWith(all_vars{i},'dg') || startsWith(all_vars{i},'dh')
       count = count +1; saving{count} = all_vars{i};
    end
end
save('data/doublederivatives.mat',saving{:})