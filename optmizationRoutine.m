%% In this file we computed all the thing necessary for the optimization
% At first we computed all the matrix necessary for the computation of the
% two sensitivity, with the use of the MatlabFunction created in the file
% b. And then we make the integration of the two sensitivity that we used
% to compute the new coefficients of the optimal trajectory at the end of
% the file 

close all; clc;
% Load nominal data and the desired trajectory (needed to make substitution
% in the parametric function computed in b_utility_funct_creation)
file = 'data/IDEALcontrol';
file1 = 'data/desired_trajectory';
file2 ='data/coeff_a';

% We set the params vector to the nominal parameters
params=[wheelRadius; wheelDistance];

u_history = load(file,'u_history');
xhi_history = load(file,'xhi_history');
q_history = load(file,'q_history');
desired_traj = load(file1,'p'); 
dp = load(file1,'dp'); 
ddp = load(file1,'ddp');

% Load of the params of the nominal trajectory --> they are used to set the
% initial values of the optimal trajectory (computed at the end of this
% file)
ax = load(file2,'a_x');
ax0 = ax. a_x;
ay = load(file2,'a_y');
ay0 = ay.a_y;

% Create variables
q_history = q_history.q_history; 
u_history = u_history.u_history;v_x1,v_y1,v_x2,v_y2
desired_traj = desired_traj.p; 
dp = dp.dp; ddp = ddp.ddp; 
xhi_history = xhi_history.xhi_history;

%   In this case we are substituting parameters within the functions we created, 
%   so we get the elements to create the sensitivity.
f_q = zeros(3,3,Nstep); f_p = zeros(3,2,Nstep); f_u = zeros(3,2,Nstep); g_q = zeros(3,3,Nstep); g_xhi = zeros(3,3,Nstep); h_xhi = zeros(2,3,Nstep); h_q = zeros(2,3,Nstep);
h_ax1 = zeros(2,1,Nstep); h_ay1 = zeros(2,1,Nstep); h_ax2 = zeros(2,1,Nstep); h_ay2= zeros(2,1,Nstep); h_ax3 = zeros(2,1,Nstep); h_ay3 = zeros(2,1,Nstep); h_ax4 = zeros(2,1,Nstep); h_ay4 = zeros(2,1,Nstep); h_ax5 = zeros(2,1,Nstep); h_ay5 = zeros(2,1,Nstep);
g_ax1= zeros(3,1,Nstep); g_ay1 = zeros(3,1,Nstep); g_ax2 = zeros(3,1,Nstep); g_ay2 = zeros(3,1,Nstep); g_ax3 = zeros(3,1,Nstep); g_ay3 = zeros(3,1,Nstep); g_ax4 = zeros(3,1,Nstep); g_ay4 = zeros(3,1,Nstep); g_ax5 = zeros(3,1,Nstep); g_ay5 = zeros(3,1,Nstep);

for k = 1:Nstep  tic
% Creation of the matrix needed in the integration of the Sensitivity
f_p(:,:,k) = ff_p(q_history(:,k),u_history(:,k),params(1:2));
f_q(:,:,k) = ff_q(q_history(:,k),u_history(:,k),params(1:2));
f_u(:,:,k) = ff_u(q_history(:,k),params(1:2));
h_q(:,:,k) = fh_q(q_history(:,k),xhi_history(:,k),desired_traj(:,k),dp(:,k),ddp(:,k),params(1:2));
h_xhi(:,:,k) = fh_xhi(q_history(:,k),xhi_history(:,k),desired_traj(:,k),dp(:,k),ddp(:,k), params(1:2));
g_q(:,:,k) = fg_q(q_history(:,k),xhi_history(:,k),desired_traj(:,k),dp(:,k),ddp(:,k));
g_xhi(:,:,k) = fg_xhi(q_history(:,k),xhi_history(:,k),desired_traj(:,k),dp(:,k),ddp(:,k));

% Creation of the matrix needed in the integration of the Gamma
h_ax1(:,:,k) = h_a_1x(q_history(:,k),timeVec(k),xhi_history(:,k), params(1:2));
h_ay1(:,:,k) = h_a_1y(q_history(:,k),timeVec(k),xhi_history(:,k), params(1:2));
h_ax2(:,:,k) = h_a_2x(q_history(:,k),timeVec(k),xhi_history(:,k), params(1:2));
h_ay2(:,:,k) = h_a_2y(q_history(:,k),timeVec(k),xhi_history(:,k), params(1:2));
h_ax3(:,:,k) = h_a_3x(q_history(:,k),timeVec(k),xhi_history(:,k), params(1:2));
h_ay3(:,:,k) = h_a_3y(q_history(:,k),timeVec(k),xhi_history(:,k), params(1:2));
h_ax4(:,:,k) = h_a_4x(q_history(:,k),timeVec(k),xhi_history(:,k), params(1:2));
h_ay4(:,:,k) = h_a_4y(q_history(:,k),timeVec(k),xhi_history(:,k), params(1:2));
h_ax5(:,:,k) = h_a_5x(q_history(:,k),timeVec(k),xhi_history(:,k), params(1:2));
h_ay5(:,:,k) = h_a_5y(q_history(:,k),timeVec(k),xhi_history(:,k), params(1:2));

g_ax1(:,:,k) = g_a_1x(q_history(:,k),timeVec(k));
g_ay1(:,:,k) = g_a_1y(q_history(:,k),timeVec(k));
g_ax2(:,:,k) = g_a_2x(q_history(:,k),timeVec(k));
g_ay2(:,:,k) = g_a_2y(q_history(:,k),timeVec(k));
g_ax3(:,:,k) = g_a_3x(q_history(:,k),timeVec(k));
g_ay3(:,:,k) = g_a_3y(q_history(:,k),timeVec(k));
g_ax4(:,:,k) = g_a_4x(q_history(:,k),timeVec(k));
g_ay4(:,:,k) = g_a_4y(q_history(:,k),timeVec(k));
g_ax5(:,:,k) = g_a_5x(q_history(:,k),timeVec(k));
g_ay5(:,:,k) = g_a_5y(q_history(:,k),timeVec(k));
end

%% Computation of the Sensitivity
% We do there the integration with the use of integralSens function
% presented in my_function folder
%sens_k = zeros(12,1); sens_int= zeros(12, Nstep); 
sens_k = zeros(12,1); sens_int= zeros(12, Nstep); 
for k=1:Nstep
    [t_s, i] = ode45(@(t,i) integralSens(t, i, f_q, f_p, f_u, g_q, g_xhi, h_q, h_xhi, k), [0 delta], sens_k);
    sens_k= i(end, :)'; sens_int(:,k) = sens_k;
end

%% Gamma calculation from integration for every parameter x
% In this part, we are going to calculate the integration of the gamma 
% to obtain its value to be used in the final differential system
gamma_k1 = zeros(6,1); gammax1_int= zeros(6,1,Nstep); gamma_k2 = zeros(6,1); gammax2_int= zeros(6,1,Nstep);
gamma_k3 = zeros(6,1); gammax3_int= zeros(6,1,Nstep); gamma_k4 = zeros(6,1); gammax4_int= zeros(6,1,Nstep);
gamma_k5 = zeros(6,1); gammax5_int= zeros(6,1,Nstep); gamma_k6 = zeros(6,1); gammax6_int= zeros(6,1,Nstep);
gamma_k7 = zeros(6,1); gammax7_int= zeros(6,1,Nstep); gamma_k8 = zeros(6,1); gammax8_int= zeros(6,1,Nstep);
gamma_k9 = zeros(6,1); gammax9_int= zeros(6,1,Nstep); gamma_k10 = zeros(6,1); gammax10_int= zeros(6,1,Nstep);
gamma_k11 = zeros(6,1); gammax11_int= zeros(6,1,Nstep); gamma_k12 = zeros(6,1); gammax12_int= zeros(6,1,Nstep);
gamma_k13 = zeros(6,1); gammax13_int= zeros(6,1,Nstep); gamma_k14 = zeros(6,1); gammax14_int= zeros(6,1,Nstep);
gamma_k15 = zeros(6,1); gammax15_int= zeros(6,1,Nstep);
for k=1:Nstep
    [~, gamma1] = ode45(@(t,gamma1) integralGamma(t,gamma1, f_q, f_u, g_q, g_xhi, h_q, h_xhi, h_ax1, g_ax1, k), [0 delta], gamma_k1);
    gamma_k1= gamma1(end, :)'; gammax1_int(:,:,k) = gamma_k1;
    [~, gamma2] = ode45(@(t,gamma2) integralGamma(t,gamma2, f_q, f_u, g_q, g_xhi, h_q, h_xhi, h_ax2, g_ax2, k), [0 delta], gamma_k2);
    gamma_k2= gamma2(end, :)'; gammax2_int(:,:,k) = gamma_k2;
    [~, gamma3] = ode45(@(t,gamma3) integralGamma(t,gamma3, f_q, f_u, g_q, g_xhi, h_q, h_xhi, h_ax3, g_ax3, k), [0 delta], gamma_k3);
    gamma_k3= gamma3(end, :)'; gammax3_int(:,:,k) = gamma_k3;
    [~, gamma4] = ode45(@(t,gamma4) integralGamma(t,gamma4, f_q, f_u, g_q, g_xhi, h_q, h_xhi, h_ax4, g_ax4, k), [0 delta], gamma_k4);
    gamma_k4= gamma4(end, :)'; gammax4_int(:,:,k) = gamma_k4;
    [~, gamma5] = ode45(@(t,gamma5) integralGamma(t,gamma5, f_q, f_u, g_q, g_xhi, h_q, h_xhi, h_ax5, g_ax5, k), [0 delta], gamma_k5);
    gamma_k5= gamma5(end, :)'; gammax5_int(:,:,k) = gamma_k5;
    [~, gamma6] = ode45(@(t,gamma6) integralGamma(t,gamma6, f_q, f_u, g_q, g_xhi, h_q, h_xhi, h_ax1, g_ax1, k), [0 delta], gamma_k6);
    gamma_k6= gamma6(end, :)'; gammax6_int(:,:,k) = gamma_k6;
    [~, gamma7] = ode45(@(t,gamma7) integralGamma(t,gamma7, f_q, f_u, g_q, g_xhi, h_q, h_xhi, h_ax2, g_ax2, k), [0 delta], gamma_k7);
    gamma_k7= gamma7(end, :)'; gammax7_int(:,:,k) = gamma_k7;
    [~, gamma8] = ode45(@(t,gamma8) integralGamma(t,gamma8, f_q, f_u, g_q, g_xhi, h_q, h_xhi, h_ax3, g_ax3, k), [0 delta], gamma_k8);
    gamma_k8= gamma8(end, :)'; gammax8_int(:,:,k) = gamma_k8;
    [~, gamma9] = ode45(@(t,gamma9) integralGamma(t,gamma9, f_q, f_u, g_q, g_xhi, h_q, h_xhi, h_ax4, g_ax4, k), [0 delta], gamma_k9);
    gamma_k9= gamma9(end, :)'; gammax9_int(:,:,k) = gamma_k9;
    [~, gamma10] = ode45(@(t,gamma10) integralGamma(t,gamma10, f_q, f_u, g_q, g_xhi, h_q, h_xhi, h_ax5, g_ax5, k), [0 delta], gamma_k10);
    gamma_k10= gamma10(end, :)'; gammax10_int(:,:,k) = gamma_k10;
    [~, gamma11] = ode45(@(t,gamma11) integralGamma(t,gamma11, f_q, f_u, g_q, g_xhi, h_q, h_xhi, h_ax1, g_ax1, k), [0 delta], gamma_k11);
    gamma_k11= gamma11(end, :)'; gammax11_int(:,:,k) = gamma_k11;
    [~, gamma12] = ode45(@(t,gamma12) integralGamma(t,gamma12, f_q, f_u, g_q, g_xhi, h_q, h_xhi, h_ax2, g_ax2, k), [0 delta], gamma_k12);
    gamma_k12= gamma12(end, :)'; gammax12_int(:,:,k) = gamma_k12;
    [~, gamma13] = ode45(@(t,gamma13) integralGamma(t,gamma13, f_q, f_u, g_q, g_xhi, h_q, h_xhi, h_ax3, g_ax3, k), [0 delta], gamma_k13);
    gamma_k13= gamma13(end, :)'; gammax13_int(:,:,k) = gamma_k13;
    [~, gamma14] = ode45(@(t,gamma14) integralGamma(t,gamma14, f_q, f_u, g_q, g_xhi, h_q, h_xhi, h_ax4, g_ax4, k), [0 delta], gamma_k14);
    gamma_k14= gamma14(end, :)'; gammax14_int(:,:,k) = gamma_k14;
    [~, gamma15] = ode45(@(t,gamma15) integralGamma(t,gamma15, f_q, f_u, g_q, g_xhi, h_q, h_xhi, h_ax5, g_ax5, k), [0 delta], gamma_k15);
    gamma_k15= gamma15(end, :)'; gammax15_int(:,:,k) = gamma_k15;
end

%% Gamma calculation from integration for every parameter y
%% Gamma calculation from integration for every parameter x
% In this part, we are going to calculate the integration of the gamma 
% to obtain its value to be used in the final differential system
gamma_k1 = zeros(6,1); gammay1_int= zeros(6,1,Nstep); gamma_k2 = zeros(6,1); gammay2_int= zeros(6,1,Nstep);
gamma_k3 = zeros(6,1); gammay3_int= zeros(6,1,Nstep); gamma_k4 = zeros(6,1); gammay4_int= zeros(6,1,Nstep);
gamma_k5 = zeros(6,1); gammay5_int= zeros(6,1,Nstep); gamma_k6 = zeros(6,1); gammay6_int= zeros(6,1,Nstep);
gamma_k7 = zeros(6,1); gammay7_int= zeros(6,1,Nstep); gamma_k8 = zeros(6,1); gammay8_int= zeros(6,1,Nstep);
gamma_k9 = zeros(6,1); gammay9_int= zeros(6,1,Nstep); gamma_k10 = zeros(6,1); gammay10_int= zeros(6,1,Nstep);
gamma_k11 = zeros(6,1); gammay11_int= zeros(6,1,Nstep); gamma_k12 = zeros(6,1); gammay12_int= zeros(6,1,Nstep);
gamma_k13 = zeros(6,1); gammay13_int= zeros(6,1,Nstep); gamma_k14 = zeros(6,1); gammay14_int= zeros(6,1,Nstep);
gamma_k15 = zeros(6,1); gammay15_int= zeros(6,1,Nstep);
for k=1:Nstep
    [~, gamma1] = ode45(@(t,gamma1) integralGamma(t,gamma1, f_q, f_u, g_q, g_xhi, h_q, h_xhi, h_ax1, g_ax1, k), [0 delta], gamma_k1);
    gamma_k1= gamma1(end, :)'; gammay1_int(:,:,k) = gamma_k1;
    [~, gamma2] = ode45(@(t,gamma2) integralGamma(t,gamma2, f_q, f_u, g_q, g_xhi, h_q, h_xhi, h_ax2, g_ax2, k), [0 delta], gamma_k2);
    gamma_k2= gamma2(end, :)'; gammay2_int(:,:,k) = gamma_k2;
    [~, gamma3] = ode45(@(t,gamma3) integralGamma(t,gamma3, f_q, f_u, g_q, g_xhi, h_q, h_xhi, h_ax3, g_ax3, k), [0 delta], gamma_k3);
    gamma_k3= gamma3(end, :)'; gammay3_int(:,:,k) = gamma_k3;
    [~, gamma4] = ode45(@(t,gamma4) integralGamma(t,gamma4, f_q, f_u, g_q, g_xhi, h_q, h_xhi, h_ax4, g_ax4, k), [0 delta], gamma_k4);
    gamma_k4= gamma4(end, :)'; gammay4_int(:,:,k) = gamma_k4;
    [~, gamma5] = ode45(@(t,gamma5) integralGamma(t,gamma5, f_q, f_u, g_q, g_xhi, h_q, h_xhi, h_ax5, g_ax5, k), [0 delta], gamma_k5);
    gamma_k5= gamma5(end, :)'; gammay5_int(:,:,k) = gamma_k5;
    [~, gamma6] = ode45(@(t,gamma6) integralGamma(t,gamma6, f_q, f_u, g_q, g_xhi, h_q, h_xhi, h_ax1, g_ax1, k), [0 delta], gamma_k6);
    gamma_k6= gamma6(end, :)'; gammay6_int(:,:,k) = gamma_k6;
    [~, gamma7] = ode45(@(t,gamma7) integralGamma(t,gamma7, f_q, f_u, g_q, g_xhi, h_q, h_xhi, h_ax2, g_ax2, k), [0 delta], gamma_k7);
    gamma_k7= gamma7(end, :)'; gammay7_int(:,:,k) = gamma_k7;
    [~, gamma8] = ode45(@(t,gamma8) integralGamma(t,gamma8, f_q, f_u, g_q, g_xhi, h_q, h_xhi, h_ax3, g_ax3, k), [0 delta], gamma_k8);
    gamma_k8= gamma8(end, :)'; gammay8_int(:,:,k) = gamma_k8;
    [~, gamma9] = ode45(@(t,gamma9) integralGamma(t,gamma9, f_q, f_u, g_q, g_xhi, h_q, h_xhi, h_ax4, g_ax4, k), [0 delta], gamma_k9);
    gamma_k9= gamma9(end, :)'; gammay9_int(:,:,k) = gamma_k9;
    [~, gamma10] = ode45(@(t,gamma10) integralGamma(t,gamma10, f_q, f_u, g_q, g_xhi, h_q, h_xhi, h_ax5, g_ax5, k), [0 delta], gamma_k10);
    gamma_k10= gamma10(end, :)'; gammay10_int(:,:,k) = gamma_k10;
    [~, gamma11] = ode45(@(t,gamma11) integralGamma(t,gamma11, f_q, f_u, g_q, g_xhi, h_q, h_xhi, h_ax1, g_ax1, k), [0 delta], gamma_k11);
    gamma_k11= gamma11(end, :)'; gammay11_int(:,:,k) = gamma_k11;
    [~, gamma12] = ode45(@(t,gamma12) integralGamma(t,gamma12, f_q, f_u, g_q, g_xhi, h_q, h_xhi, h_ax2, g_ax2, k), [0 delta], gamma_k12);
    gamma_k12= gamma12(end, :)'; gammay12_int(:,:,k) = gamma_k12;
    [~, gamma13] = ode45(@(t,gamma13) integralGamma(t,gamma13, f_q, f_u, g_q, g_xhi, h_q, h_xhi, h_ax3, g_ax3, k), [0 delta], gamma_k13);
    gamma_k13= gamma13(end, :)'; gammay13_int(:,:,k) = gamma_k13;
    [~, gamma14] = ode45(@(t,gamma14) integralGamma(t,gamma14, f_q, f_u, g_q, g_xhi, h_q, h_xhi, h_ax4, g_ax4, k), [0 delta], gamma_k14);
    gamma_k14= gamma14(end, :)'; gammay14_int(:,:,k) = gamma_k14;
    [~, gamma15] = ode45(@(t,gamma15) integralGamma(t,gamma15, f_q, f_u, g_q, g_xhi, h_q, h_xhi, h_ax5, g_ax5, k), [0 delta], gamma_k15);
    gamma_k15= gamma15(end, :)'; gammay15_int(:,:,k) = gamma_k15;
end


%% u_ai vectors creation
% which is used to differentiate the whole system with respect to the coefficients a_i
% We use these vectors in the integration of the Sensitivity_ai

%for coefficients x
u_ax1 = pagemtimes(h_q,gammax1_int(1:3,:,:)) + pagemtimes(h_xhi,gammax1_int(4:6,:,:)) + h_ax1;
u_ax2 = pagemtimes(h_q,gammax2_int(1:3,:,:)) + pagemtimes(h_xhi,gammax2_int(4:6,:,:)) + h_ax2;
u_ax3 = pagemtimes(h_q,gammax3_int(1:3,:,:)) + pagemtimes(h_xhi,gammax3_int(4:6,:,:)) + h_ax3;
u_ax4 = pagemtimes(h_q,gammax4_int(1:3,:,:)) + pagemtimes(h_xhi,gammax4_int(4:6,:,:)) + h_ax4;
u_ax5 = pagemtimes(h_q,gammax5_int(1:3,:,:)) + pagemtimes(h_xhi,gammax5_int(4:6,:,:)) + h_ax5;

u_ax6 = pagemtimes(h_q,gammax6_int(1:3,:,:)) + pagemtimes(h_xhi,gammax6_int(4:6,:,:)) + h_ax1;
u_ax7 = pagemtimes(h_q,gammax7_int(1:3,:,:)) + pagemtimes(h_xhi,gammax7_int(4:6,:,:)) + h_ax2;
u_ax8 = pagemtimes(h_q,gammax8_int(1:3,:,:)) + pagemtimes(h_xhi,gammax8_int(4:6,:,:)) + h_ax3;
u_ax9 = pagemtimes(h_q,gammax9_int(1:3,:,:)) + pagemtimes(h_xhi,gammax9_int(4:6,:,:)) + h_ax4;
u_ax10 = pagemtimes(h_q,gammax10_int(1:3,:,:)) + pagemtimes(h_xhi,gammax10_int(4:6,:,:)) + h_ax5;

u_ax11 = pagemtimes(h_q,gammax11_int(1:3,:,:)) + pagemtimes(h_xhi,gammax11_int(4:6,:,:)) + h_ax1;
u_ax12 = pagemtimes(h_q,gammax12_int(1:3,:,:)) + pagemtimes(h_xhi,gammax12_int(4:6,:,:)) + h_ax2;
u_ax13 = pagemtimes(h_q,gammax13_int(1:3,:,:)) + pagemtimes(h_xhi,gammax13_int(4:6,:,:)) + h_ax3;
u_ax14 = pagemtimes(h_q,gammax14_int(1:3,:,:)) + pagemtimes(h_xhi,gammax14_int(4:6,:,:)) + h_ax4;
u_ax15 = pagemtimes(h_q,gammax15_int(1:3,:,:)) + pagemtimes(h_xhi,gammax15_int(4:6,:,:)) + h_ax5;

% for coefficients y
u_ay1 = pagemtimes(h_q,gammay1_int(1:3,:,:)) + pagemtimes(h_xhi,gammay1_int(4:6,:,:)) + h_ay1;
u_ay2 = pagemtimes(h_q,gammay2_int(1:3,:,:)) + pagemtimes(h_xhi,gammay2_int(4:6,:,:)) + h_ay2;
u_ay3 = pagemtimes(h_q,gammay3_int(1:3,:,:)) + pagemtimes(h_xhi,gammay3_int(4:6,:,:)) + h_ay3;
u_ay4 = pagemtimes(h_q,gammay4_int(1:3,:,:)) + pagemtimes(h_xhi,gammay4_int(4:6,:,:)) + h_ay4;
u_ay5 = pagemtimes(h_q,gammay5_int(1:3,:,:)) + pagemtimes(h_xhi,gammay5_int(4:6,:,:)) + h_ay5;

u_ay6 = pagemtimes(h_q,gammay6_int(1:3,:,:)) + pagemtimes(h_xhi,gammay6_int(4:6,:,:)) + h_ay1;
u_ay7 = pagemtimes(h_q,gammay7_int(1:3,:,:)) + pagemtimes(h_xhi,gammay7_int(4:6,:,:)) + h_ay2;
u_ay8 = pagemtimes(h_q,gammay8_int(1:3,:,:)) + pagemtimes(h_xhi,gammay8_int(4:6,:,:)) + h_ay3;
u_ay9 = pagemtimes(h_q,gammay9_int(1:3,:,:)) + pagemtimes(h_xhi,gammay9_int(4:6,:,:)) + h_ay4;
u_ay10 = pagemtimes(h_q,gammay10_int(1:3,:,:)) + pagemtimes(h_xhi,gammay10_int(4:6,:,:)) + h_ay5;

u_ay11 = pagemtimes(h_q,gammay11_int(1:3,:,:)) + pagemtimes(h_xhi,gammay11_int(4:6,:,:)) + h_ay1;
u_ay12 = pagemtimes(h_q,gammay12_int(1:3,:,:)) + pagemtimes(h_xhi,gammay12_int(4:6,:,:)) + h_ay2;
u_ay13 = pagemtimes(h_q,gammay13_int(1:3,:,:)) + pagemtimes(h_xhi,gammay13_int(4:6,:,:)) + h_ay3;
u_ay14 = pagemtimes(h_q,gammay14_int(1:3,:,:)) + pagemtimes(h_xhi,gammay14_int(4:6,:,:)) + h_ay4;
u_ay15 = pagemtimes(h_q,gammay15_int(1:3,:,:)) + pagemtimes(h_xhi,gammay15_int(4:6,:,:)) + h_ay5;

%% Tensor product calculations, one for each parameter
% initialize 3D matrixes, which are overwritten 
% how to read: 
% - dfqqgamma--> tensor product between the derivate of f_q respect to q and gamma
% - dfpqgamma--> tensor product between the derivate of f_p respect to q wand gamma
% - dhxhiqgamma--> tensor product between the derivate of h_xhi respect to q with gamma
% ...
curr_dfqqgamma = zeros(3,3,Nstep); curr_dfquuai = zeros(3,3,Nstep); 
curr_dfpqgamma = zeros(3,2,Nstep); curr_dfpuuai = zeros(3,2,Nstep); 
curr_dfuqgamma = zeros(3,2,Nstep); curr_dfuuuai = zeros(3,2,Nstep); 
curr_dhqqgamma = zeros(2,3,Nstep); curr_dhqxhigammaxhi = zeros(2,3,Nstep); 
curr_dhxhiqgamma = zeros(2,3,Nstep); curr_dhxhixhigammaxhi = zeros(2,3,Nstep); 
curr_dgqqgamma = zeros(3,3,Nstep); curr_dgqxhigammaxhi = zeros(3,3,Nstep); 
curr_dgxhiqgamma = zeros(3,3,Nstep); curr_dgxhixhigammaxhi = zeros(3,3,Nstep);

% Initialize cell array to store all x double derivatives, one for each x
allx_dfqqgamma =cell(1,12); allx_dfquuai = cell(1,12); 
allx_dfpqgamma =cell(1,12); allx_dfpuuai = cell(1,12); 
allx_dfuqgamma = cell(1,12); allx_dfuuuai = cell(1,12); 
allx_dhqqgamma = cell(1,12); allx_dhqxhigammaxhi = cell(1,12); 
allx_dhxhiqgamma = cell(1,12); allx_dhxhixhigammaxhi = cell(1,12); 
allx_dgqqgamma = cell(1,12); allx_dgqxhigammaxhi = cell(1,12); 
allx_dgxhiqgamma = cell(1,12); allx_dgxhixhigammaxhi = cell(1,12);

% We compute this for all the 12 trajectory coefficients x and for the 12
% trajectory coefficients y
% X
for i = 1:12
    % Get name of each gamma integrated variable
    gamma_name = sprintf('gammax%d_int', i); uai_name = sprintf('u_ax%d', i);
    % Get variable
    var = eval(gamma_name); var2 = eval(uai_name);
    % Loop over time steps
    for k = 1:Nstep
        % For the computation we use the Matlab Function created in the
        % file b
        gamma = var(1:3,k); gammaxhi = var(4:6,k); u_ai = var2(:,:,k);
        curr_dfqqgamma(:,:,k) = df_q_q_gamma(gamma, params(1:2),u_history(:,k),q_history(:,k));
        curr_dfquuai(:,:,k) = df_q_u_uai(q_history(:,k),u_ai,params(1:2));
        curr_dfpqgamma(:,:,k) = df_p_q_gamma(gamma, u_history(:,k), q_history(:,k));
        curr_dfpuuai(:,:,k) = df_p_u_uai(u_ai, q_history(:,k), params(1:2));
        curr_dfuqgamma(:,:,k) = df_u_q_gamma(gamma, params(1:2), q_history(:,k));
        curr_dfuuuai(:,:,k) = df_u_u_uai();
        curr_dhqqgamma(:,:,k) = dh_q_q_gamma(gamma,q_history(:,k),xhi_history(:,k),desired_traj(:,k),dp(:,k),ddp(:,k), params(1:2));
        curr_dhqxhigammaxhi(:,:,k) =dh_q_xhi_gammaxhi(gammaxhi,q_history(:,k),xhi_history(:,k),desired_traj(:,k),dp(:,k),ddp(:,k), params(1:2));
        curr_dhxhiqgamma(:,:,k) =dh_xhi_q_gamma(gamma,q_history(:,k),xhi_history(:,k),desired_traj(:,k),dp(:,k),ddp(:,k), params(1:2));
        curr_dhxhixhigammaxhi(:,:,k) = dh_xhi_xhi_gammaxhi(gammaxhi,q_history(:,k),xhi_history(:,k),desired_traj(:,k),dp(:,k),ddp(:,k), params(1:2));
        curr_dgqqgamma(:,:,k) = dg_q_q_gamma(gamma,q_history(:,k),xhi_history(:,k),desired_traj(:,k),dp(:,k),ddp(:,k));
        curr_dgqxhigammaxhi(:, :, k) = dg_q_xhi_gammaxhi(gammaxhi,q_history(:,k));
        curr_dgxhiqgamma(:,:,k) = dg_xhi_q_gamma(gamma,q_history(:,k));
        curr_dgxhixhigammaxhi(:,:,k) = dg_xhi_xhi_gammaxhi(); 
    end
    % Store 3D matrix in cell array
    allx_dfqqgamma{i} = curr_dfqqgamma; allx_dfquuai{i} =  curr_dfquuai; 
    allx_dfpqgamma{i} = curr_dfpqgamma; allx_dfpuuai{i} =  curr_dfpuuai;
    allx_dfuqgamma{i} = curr_dfuqgamma; allx_dfuuuai{i} =  curr_dfuuuai; 
    allx_dhqqgamma{i} =  curr_dhqqgamma; allx_dhqxhigammaxhi{i} =  curr_dhqxhigammaxhi;
    allx_dhxhiqgamma{i} =  curr_dhxhiqgamma; allx_dhxhixhigammaxhi{i} =  curr_dhxhixhigammaxhi; 
    allx_dgqqgamma{i} =  curr_dgqqgamma; allx_dgqxhigammaxhi{i} = curr_dgqxhigammaxhi;
    allx_dgxhiqgamma{i} =  curr_dgxhiqgamma; allx_dgxhixhigammaxhi{i} = curr_dgxhixhigammaxhi;
end

% Initialize cell array to store all y double derivatives, one for each y
ally_dfqqgamma =cell(1,12);  ally_dfquuai = cell(1,12); 
ally_dfpqgamma =cell(1,12);  ally_dfpuuai = cell(1,12);  
ally_dfuqgamma = cell(1,12);  ally_dfuuuai = cell(1,12); 
ally_dhqqgamma = cell(1,12); ally_dhqxhigammaxhi = cell(1,12); 
ally_dhxhiqgamma = cell(1,12); ally_dhxhixhigammaxhi = cell(1,12); 
ally_dgqqgamma = cell(1,12);  ally_dgqxhigammaxhi = cell(1,12); 
ally_dgxhiqgamma = cell(1,12); ally_dgxhixhigammaxhi = cell(1,12);
% Y
for i = 1:12
    % Get name of each gamma integrated variable
    gamma_name = sprintf('gammay%d_int', i); uai_name = sprintf('u_ay%d', i);
    % Get variable
    var = eval(gamma_name); var2 = eval(uai_name);
    % Loop over time steps
    for k = 1:Nstep
        % For the computation we use the Matlab Function created in the
        % file b
        gamma = var(1:3,k); gammaxhi = var(4:6,k); u_ai = var2(:,:,k);
        curr_dfqqgamma(:,:,k) = df_q_q_gamma(gamma, params(1:2),u_history(:,k),q_history(:,k));
        curr_dfquuai(:,:,k) = df_q_u_uai(q_history(:,k),u_ai,params(1:2));
        curr_dfpqgamma(:,:,k) = df_p_q_gamma(gamma, u_history(:,k), q_history(:,k));
        curr_dfpuuai(:,:,k) = df_p_u_uai(u_ai, q_history(:,k), params(1:2));
        curr_dfuqgamma(:,:,k) = df_u_q_gamma(gamma, params(1:2), q_history(:,k));
        curr_dfuuuai(:,:,k) = df_u_u_uai();
        curr_dhqqgamma(:,:,k) = dh_q_q_gamma(gamma,q_history(:,k),xhi_history(:,k),desired_traj(:,k),dp(:,k),ddp(:,k), params(1:2));
        curr_dhqxhigammaxhi(:,:,k) =dh_q_xhi_gammaxhi(gammaxhi,q_history(:,k),xhi_history(:,k),desired_traj(:,k),dp(:,k),ddp(:,k), params(1:2));
        curr_dhxhiqgamma(:,:,k) =dh_xhi_q_gamma(gamma,q_history(:,k),xhi_history(:,k),desired_traj(:,k),dp(:,k),ddp(:,k), params(1:2));
        curr_dhxhixhigammaxhi(:,:,k) = dh_xhi_xhi_gammaxhi(gammaxhi,q_history(:,k),xhi_history(:,k),desired_traj(:,k),dp(:,k),ddp(:,k), params(1:2));
        curr_dgqqgamma(:,:,k) = dg_q_q_gamma(gamma,q_history(:,k),xhi_history(:,k),desired_traj(:,k),dp(:,k),ddp(:,k));
        curr_dgqxhigammaxhi(:, :, k) = dg_q_xhi_gammaxhi(gammaxhi,q_history(:,k));
        curr_dgxhiqgamma(:,:,k) = dg_xhi_q_gamma(gamma,q_history(:,k));
        curr_dgxhixhigammaxhi(:,:,k) = dg_xhi_xhi_gammaxhi(); 
    end
    % Store 3D matrix in cell array
    ally_dfqqgamma{i} = curr_dfqqgamma; ally_dfquuai{i} =  curr_dfquuai; 
    ally_dfpqgamma{i} = curr_dfpqgamma; ally_dfpuuai{i} =  curr_dfpuuai;
    ally_dfuqgamma{i} = curr_dfuqgamma; ally_dfuuuai{i} =  curr_dfuuuai; 
    ally_dhqqgamma{i} =  curr_dhqqgamma; ally_dhqxhigammaxhi{i} =  curr_dhqxhigammaxhi;
    ally_dhxhiqgamma{i} =  curr_dhxhiqgamma; ally_dhxhixhigammaxhi{i} =  curr_dhxhixhigammaxhi; 
    ally_dgqqgamma{i} =  curr_dgqqgamma; ally_dgqxhigammaxhi{i} = curr_dgqxhigammaxhi;
    ally_dgxhiqgamma{i} =  curr_dgxhiqgamma; ally_dgxhixhigammaxhi{i} = curr_dgxhixhigammaxhi;
end

%% Creation of derivate of h_q, h_xhi, g_q, g_xhi respect to coefficents [a1_x, a1_y, a2_x, ...]
% These are necessary to make the integration of Sensitivity_ai

% Double derivatives respect to the parameters 
dhqa1x = zeros(2,3,Nstep); dhqa1y = zeros(2,3,Nstep); dhqa2x = zeros(2,3,Nstep); dhqa2y = zeros(2,3,Nstep); dhqa3x = zeros(2,3,Nstep); dhqa3y = zeros(2,3,Nstep); dhqa4x = zeros(2,3,Nstep); dhqa4y = zeros(2,3,Nstep); dhqa5x = zeros(2,3,Nstep); dhqa5y = zeros(2,3,Nstep);
dhxhia1x = zeros(2,3,Nstep); dhxhia1y = zeros(2,3,Nstep); dhxhia2x = zeros(2,3,Nstep); dhxhia2y = zeros(2,3,Nstep); dhxhia3x = zeros(2,3,Nstep); dhxhia3y = zeros(2,3,Nstep); dhxhia4x = zeros(2,3,Nstep); dhxhia4y = zeros(2,3,Nstep); dhxhia5x = zeros(2,3,Nstep); dhxhia5y = zeros(2,3,Nstep); 
dgqa1x = zeros(3,3,Nstep); dgqa1y = zeros(3,3,Nstep); dgqa2x = zeros(3,3,Nstep); dgqa2y = zeros(3,3,Nstep); dgqa3x = zeros(3,3,Nstep); dgqa3y = zeros(3,3,Nstep); dgqa4x = zeros(3,3,Nstep); dgqa4y = zeros(3,3,Nstep); dgqa5x = zeros(3,3,Nstep); dgqa5y = zeros(3,3,Nstep); 
dgxhia1x = zeros(3,3,Nstep); dgxhia1y = zeros(3,3,Nstep); dgxhia2x = zeros(3,3,Nstep); dgxhia2y = zeros(3,3,Nstep); dgxhia3x = zeros(3,3,Nstep); dgxhia3y = zeros(3,3,Nstep); dgxhia4x = zeros(3,3,Nstep); dgxhia4y = zeros(3,3,Nstep); dgxhia5x = zeros(3,3,Nstep); dgxhia5y = zeros(3,3,Nstep);
for k=1:Nstep
    dhqa1x(:,:,k) = dhq_a1x(params(2), params(1), timeVec(k), q_history(3,k), xhi_history(1,k));
    dhqa2x(:,:,k) = dhq_a2x(params(2), params(1), timeVec(k), q_history(3,k), xhi_history(1,k));
    dhqa3x(:,:,k) = dhq_a3x(params(2), params(1), timeVec(k), q_history(3,k), xhi_history(1,k));
    dhqa4x(:,:,k) = dhq_a4x(params(2), params(1), timeVec(k), q_history(3,k), xhi_history(1,k));
    dhqa5x(:,:,k) = dhq_a5x(params(2), params(1), q_history(3,k), xhi_history(1,k));
    
    dhqa1y(:,:,k) = dhq_a1y(params(2), params(1), timeVec(k), q_history(3,k), xhi_history(1,k));
    dhqa2y(:,:,k) = dhq_a2y(params(2), params(1), timeVec(k), q_history(3,k), xhi_history(1,k));
    dhqa3y(:,:,k) = dhq_a3y(params(2), params(1), timeVec(k), q_history(3,k), xhi_history(1,k));
    dhqa4y(:,:,k) = dhq_a4y(params(2), params(1), timeVec(k), q_history(3,k), xhi_history(1,k));
    dhqa5y(:,:,k) = dhq_a5y(params(2), params(1), q_history(3,k), xhi_history(1,k));

    dhxhia1x(:,:,k) = dhxhi_a1x(params(2), params(1), timeVec(k), q_history(3,k), xhi_history(1,k));
    dhxhia2x(:,:,k) = dhxhi_a2x(params(2), params(1), timeVec(k), q_history(3,k), xhi_history(1,k));
    dhxhia3x(:,:,k) = dhxhi_a3x(params(2), params(1), timeVec(k), q_history(3,k), xhi_history(1,k));
    dhxhia4x(:,:,k) = dhxhi_a4x(params(2), params(1), timeVec(k), q_history(3,k), xhi_history(1,k));
    dhxhia5x(:,:,k) = dhxhi_a5x(params(2), params(1), q_history(3,k), xhi_history(1,k));

    dhxhia1y(:,:,k) = dhxhi_a1y(params(2), params(1), timeVec(k), q_history(3,k), xhi_history(1,k));
    dhxhia2y(:,:,k) = dhxhi_a2y(params(2), params(1), timeVec(k), q_history(3,k), xhi_history(1,k));
    dhxhia3y(:,:,k) = dhxhi_a3y(params(2), params(1), timeVec(k), q_history(3,k), xhi_history(1,k));
    dhxhia4y(:,:,k) = dhxhi_a4y(params(2), params(1), timeVec(k), q_history(3,k), xhi_history(1,k));
    dhxhia5y(:,:,k) = dhxhi_a5y(params(2), params(1), q_history(3,k), xhi_history(1,k));

    dgqa1x(:,:,k) = dgq_a1x(timeVec(k), q_history(3,k));
    dgqa2x(:,:,k) = dgq_a2x(timeVec(k), q_history(3,k));
    dgqa3x(:,:,k) = dgq_a3x(timeVec(k), q_history(3,k));
    dgqa4x(:,:,k) = dgq_a4x(timeVec(k), q_history(3,k));
    dgqa5x(:,:,k) = dgq_a5x(q_history(3,k));
    dgqa1y(:,:,k) = dgq_a1y(timeVec(k), q_history(3,k));
    dgqa2y(:,:,k) = dgq_a2y(timeVec(k), q_history(3,k));
    dgqa3y(:,:,k) = dgq_a3y(timeVec(k), q_history(3,k));
    dgqa4y(:,:,k) = dgq_a4y(timeVec(k), q_history(3,k));
    dgqa5y(:,:,k) = dgq_a5y(q_history(3,k));

    dgxhia1x(:,:,k)=dgxhi_a1x();
    dgxhia2x(:,:,k)=dgxhi_a2x();
    dgxhia3x(:,:,k)=dgxhi_a3x();
    dgxhia4x(:,:,k)=dgxhi_a4x();
    dgxhia5x(:,:,k)=dgxhi_a5x();
    dgxhia1y(:,:,k)=dgxhi_a1y();
    dgxhia2y(:,:,k)=dgxhi_a2y();
    dgxhia3y(:,:,k)=dgxhi_a3y();
    dgxhia4y(:,:,k)=dgxhi_a4y();
    dgxhia5y(:,:,k)=dgxhi_a5y();
 end

 % We use now all these matrix to computed the Sensitivity_ai 

%% FINAL SENSITIVITY_Ai CALCULATION THROUGHT INTEGRATION OF ODEs
% For the x parameters
all_sensaix = cell(1,12);
% Repeat the calculations 3 times, one for each trajectory
z=1;
for i=1:12
if z==5
    z=1;
end
sens_k = zeros(12,1); sensai_tot = zeros(12,1,Nstep); 
dgq = eval(sprintf('dgqa%dx', z)); dgxhi = eval(sprintf('dgxhia%dx', z)); dhq = eval(sprintf('dhqa%dx', z)); dhxhi = eval(sprintf('dhxhia%dx', z));
for k=1:Nstep
[~, j] = ode45(@(t,j) integralSens_ai(t,j, allx_dfqqgamma{i}(:,:,k),allx_dfquuai{i}(:,:,k),sens_int(1:6,k),...
         f_q(:,:,k), allx_dfpqgamma{i}(:,:,k), allx_dfpuuai{i}(:,:,k),allx_dfuqgamma{i}(:,:,k),allx_dfuuuai{i}(:,:,k),...
         h_q(:,:,k), h_xhi(:,:,k),sens_int(7:12,k), f_u(:,:,k), allx_dhqqgamma{i}(:,:,k),allx_dhqxhigammaxhi{i}(:,:,k),...
         dhq(:,:,k),allx_dhxhiqgamma{i}(:,:,k), allx_dhxhixhigammaxhi{i}(:,:,k),dhxhi(:,:,k),allx_dgqqgamma{i}(:,:,k),allx_dgqxhigammaxhi{i}(:,:,k),...
         dgq(:,:,k),allx_dgxhiqgamma{i}(:,:,k),allx_dgxhixhigammaxhi{i}(:,:,k),dgxhi(:,:,k), g_q(:,:,k), g_xhi(:,:,k)), [0 delta],sens_k);
sens_k = j(end, :)'; sensai_tot(:,:,k) = sens_k;
end
z=z+1;
all_sensaix{i} = sensai_tot;
end

% For the y parameters
all_sensaiy = cell(1,12);
% Repeat the calculations 3 times, one for each trajectory
z=1;
for i=1:12
if z==5
    z=1;
end
sens_k = zeros(12,1); sensai_tot = zeros(12,1,Nstep); 
dgq = eval(sprintf('dgqa%dy', z)); dgxhi = eval(sprintf('dgxhia%dy', z)); dhq = eval(sprintf('dhqa%dy', z)); dhxhi = eval(sprintf('dhxhia%dy', z));
for k=1:Nstep
[~, j] = ode45(@(t,j) integralSens_ai(t,j, ally_dfqqgamma{i}(:,:,k),ally_dfquuai{i}(:,:,k),sens_int(1:6,k),...
         f_q(:,:,k), ally_dfpqgamma{i}(:,:,k), ally_dfpuuai{i}(:,:,k),ally_dfuqgamma{i}(:,:,k),ally_dfuuuai{i}(:,:,k),...
         h_q(:,:,k), h_xhi(:,:,k),sens_int(7:12,k), f_u(:,:,k), ally_dhqqgamma{i}(:,:,k),ally_dhqxhigammaxhi{i}(:,:,k),...
         dhq(:,:,k),ally_dhxhiqgamma{i}(:,:,k), ally_dhxhixhigammaxhi{i}(:,:,k),dhxhi(:,:,k),ally_dgqqgamma{i}(:,:,k),ally_dgqxhigammaxhi{i}(:,:,k),...
         dgq(:,:,k),ally_dgxhiqgamma{i}(:,:,k),ally_dgxhixhigammaxhi{i}(:,:,k),dgxhi(:,:,k), g_q(:,:,k), g_xhi(:,:,k)), [0 delta],sens_k);
sens_k = j(end, :)'; sensai_tot(:,:,k) = sens_k;
end
z=z+1;
all_sensaiy{i} = sensai_tot;
end

%% At least, we compute the OPTIMAL TRAJECTORY'S coefficients, going to use the last value of sensitivity and sensitivity_ai at the last time t_f

%% Hyperparameters 
% Chosen in this way, to make a scaling to the size we are interested in
k1 = 1e-3; k2 = 1e-3;

%% Constraints vectors

dx = [initialPositionVec(1) initialVelocityVec(1) 0 0 0 0 finalPositionVec(1) finalVelocityVec(1)]';
dy = [initialPositionVec(2) initialVelocityVec(2) 0 0 0 0 finalPositionVec(2) finalVelocityVec(2)]';

M = [0 0 0 0 1 0 0 0 0 0 0 0 0 0 0; 
    % This is the definition of the initial position
     0 0 0 1 0 0 0 0 0 0 0 0 0 0 0;
    % This is the definition of the initial velocity
     (totalTime/3)^4 (totalTime/3)^3 (totalTime/3)^2 (totalTime/3) 1 0 0 0 0 -1 0 0 0 0 0;
    % This is the constraint on the position in the first polynomial ->
    % p1(t/3) = p2(0)
     4*(totalTime/3)^3 3*(totalTime/3)^2 2*(totalTime/3) 1 0 0 0 0 -1 0 0 0 0 0 0;
    % This is the constraint on the velocity in the first polynomial ->
    % v1(t/3) = v2(0)
     0 0 0 0 0 (totalTime/3)^4 (totalTime/3)^3 (totalTime/3)^2 (totalTime/3) 1 0 0 0 0 -1;
    % This is the constraint on the position in the second polynomial ->
    % p2(t/3) = p3(0)
     0 0 0 0 0 4*(totalTime/3)^3 3*(totalTime/3)^2 2*totalTime/3 1 0 0 0 0 -1 0;
    % This is the constraint on the velocity in the second polynomial ->
    % v2(t/3) = v3(0)
     0 0 0 0 0 0 0 0 0 0 (totalTime/3)^4 (totalTime/3)^3 (totalTime/3)^2 (totalTime/3) 1;
    % This is the definition of the final position
     0 0 0 0 0 0 0 0 0 0 4*(totalTime/3)^3 3*(totalTime/3)^2 2*(totalTime/3) 1 0];
    % This is the definition of the final velocity

%% Calculate vi for each x and y trajectory's coefficient
%which is the negative gradient of the cost function
sens_last = [sens_int(1,Nstep) sens_int(2,Nstep);
             sens_int(3,Nstep) sens_int(4,Nstep);
             sens_int(5,Nstep) sens_int(6,Nstep)];

vx = zeros(15,1);
for i= 1: length(all_sensaix)
    sensai_last = reshape(all_sensaix{i}(1:6,Nstep),2,[])';
    vx(i) = -trace(sens_last'*sensai_last);
end

vy = zeros(15,1);
for i= 1: length(all_sensaiy)
    sensai_last = reshape(all_sensaiy{i}(1:6,Nstep),2,[])';
    vy(i) = -trace(sens_last'*sensai_last);
end

%% Compute new optimized parameter vectors ax and ay
I = eye(15); 
ax_k = zeros(15,Nstep); ax_k(:,1)= ax0; 
ay_k = zeros(15,Nstep); ay_k(:,1)= ay0;

for n = 2:Nstep
ax_k(:,n) = ax_k(:,n-1) + delta*(k1*pinv(M)*(dx-M*ax_k(:,n-1)) + k2*(I - pinv(M)*M)*vx);
ay_k(:,n) = ay_k(:,n-1) + delta*(k1*pinv(M)*(dy-M*ay_k(:,n-1)) + k2*(I - pinv(M)*M)*vy);
end 

ax_star = ax_k(:,Nstep);
ay_star = ay_k(:,Nstep);

%% Visualize optimized trajectories
breaks = 0:totalTime/3:totalTime;

polyx = mkpp(breaks,[ax_star(1) ax_star(2) ax_star(3) ax_star(4) ax_star(5);
                     ax_star(6) ax_star(7) ax_star(8) ax_star(9) ax_star(10);
                     ax_star(11) ax_star(12) ax_star(13) ax_star(14) ax_star(15)]);
polyy = mkpp(breaks,[ay_star(1) ay_star(2) ay_star(3) ay_star(4) ay_star(5);
                     ay_star(6) ay_star(7) ay_star(8) ay_star(9) ay_star(10);
                     ay_star(11) ay_star(12) ay_star(13) ay_star(14) ay_star(15)]);

figure(),
fnplt(polyx,linewidth), hold on
fnplt(polyy,linewidth)
line([totalTime/3 totalTime/3],ylim,'LineStyle','--','Color','k','LineWidth',1), grid minor
line([2*totalTime/3 2*totalTime/3],ylim,'LineStyle','--','Color','k','LineWidth',1)
xlabel('time [sec]'), ylabel('trajecory [m]')
legend('trajectory in x', 'trajectory in y')
title('Trajectory varation in time'),hold off

% Differentiation first order
dpolyx = fnder(polyx);
dpolyy = fnder(polyy);

% Differentiation second order
ddpolyx = fnder(dpolyx);
ddpolyy = fnder(dpolyy);

r = [ppval(polyx,timeVec);ppval(polyy,timeVec)];
dr = [ppval(dpolyx,timeVec);ppval(dpolyy,timeVec)];
ddr = [ppval(ddpolyx,timeVec);ppval(ddpolyy,timeVec)];

% Optimized trajectory and non optimized comparison
figure(1)
plot(r(1,:),r(2,:)); hold on 
plot(desired_traj(1,:),desired_traj(2,:))
xlabel('x [m]'), ylabel('y [m]'), grid minor
title('Optimized and Non-Optimized Trajectory')
legend('Optimized','Non-Optimized'), hold off

toc

%% Save optimized coefficients and new trajectory
save('data/coeff_a_star',"ay_star","ay_star")
save('data/new_traj',"r","dr","ddr")
