close all; clc;
% Load from data
filelist = ["data/sensitivity_deriv.mat","data/gamma_deriv.mat"];
for elem = 1:length(filelist)
data = load(filelist(elem));
% Find all variables in the struct 
varNames = fieldnames(data);
% Extract all variables
for j = 1:length(varNames)
    eval(sprintf('%s = data.%s;', varNames{j}, varNames{j}));
end
end
%% Sensitivity calculation from integration
% sens_int = zeros(12,Nstep);
% for k=2:Nstep
% sens_int(:,k) = sens_int(:,k-1) + delta*integralSens(sens_int(:,k-1),f_q,f_p,f_u,g_q,g_xhi,h_q,h_xhi,k-1);
% end
sens_k = zeros(12,1); sens_int= zeros(12, Nstep); tic
for k=1:Nstep
    [t_s, i] = ode45(@(t,i) integralSens(t, i, f_q, f_p, f_u, g_q, g_xhi, h_q, h_xhi, k), [0 delta], sens_k);
    sens_k= i(end, :)'; sens_int(:,k) = sens_k;
end
%% Gamma calculation from integration for every parameter x
gamma_k1 = zeros(6,1); gammax1_int= zeros(6,1,Nstep); gamma_k2 = zeros(6,1); gammax2_int= zeros(6,1,Nstep);
gamma_k3 = zeros(6,1); gammax3_int= zeros(6,1,Nstep); gamma_k4 = zeros(6,1); gammax4_int= zeros(6,1,Nstep);
gamma_k5 = zeros(6,1); gammax5_int= zeros(6,1,Nstep); gamma_k6 = zeros(6,1); gammax6_int= zeros(6,1,Nstep);
gamma_k7 = zeros(6,1); gammax7_int= zeros(6,1,Nstep); gamma_k8 = zeros(6,1); gammax8_int= zeros(6,1,Nstep);
gamma_k9 = zeros(6,1); gammax9_int= zeros(6,1,Nstep); gamma_k10 = zeros(6,1); gammax10_int= zeros(6,1,Nstep);
gamma_k11 = zeros(6,1); gammax11_int= zeros(6,1,Nstep); gamma_k12 = zeros(6,1); gammax12_int= zeros(6,1,Nstep);
for k=1:Nstep
    [~, gamma1] = ode45(@(t,gamma1) integralGamma(t,gamma1, f_q, f_u, g_q, g_xhi, h_q, h_xhi, h_ax1, g_ax1, k), [0 delta], gamma_k1);
    gamma_k1= gamma1(end, :)'; gammax1_int(:,:,k) = gamma_k1;
    [~, gamma2] = ode45(@(t,gamma2) integralGamma(t,gamma2, f_q, f_u, g_q, g_xhi, h_q, h_xhi, h_ax2, g_ax2, k), [0 delta], gamma_k2);
    gamma_k2= gamma2(end, :)'; gammax2_int(:,:,k) = gamma_k2;
    [~, gamma3] = ode45(@(t,gamma3) integralGamma(t,gamma3, f_q, f_u, g_q, g_xhi, h_q, h_xhi, h_ax3, g_ax3, k), [0 delta], gamma_k3);
    gamma_k3= gamma3(end, :)'; gammax3_int(:,:,k) = gamma_k3;
    [~, gamma4] = ode45(@(t,gamma4) integralGamma(t,gamma4, f_q, f_u, g_q, g_xhi, h_q, h_xhi, h_ax4, g_ax4, k), [0 delta], gamma_k4);
    gamma_k4= gamma4(end, :)'; gammax4_int(:,:,k) = gamma_k4;
    [~, gamma5] = ode45(@(t,gamma5) integralGamma(t,gamma5, f_q, f_u, g_q, g_xhi, h_q, h_xhi, h_ax1, g_ax1, k), [0 delta], gamma_k5);
    gamma_k5= gamma5(end, :)'; gammax5_int(:,:,k) = gamma_k5;
    [~, gamma6] = ode45(@(t,gamma6) integralGamma(t,gamma6, f_q, f_u, g_q, g_xhi, h_q, h_xhi, h_ax2, g_ax2, k), [0 delta], gamma_k6);
    gamma_k6= gamma6(end, :)'; gammax6_int(:,:,k) = gamma_k6;
    [~, gamma7] = ode45(@(t,gamma7) integralGamma(t,gamma7, f_q, f_u, g_q, g_xhi, h_q, h_xhi, h_ax3, g_ax3, k), [0 delta], gamma_k7);
    gamma_k7= gamma7(end, :)'; gammax7_int(:,:,k) = gamma_k7;
    [~, gamma8] = ode45(@(t,gamma8) integralGamma(t,gamma8, f_q, f_u, g_q, g_xhi, h_q, h_xhi, h_ax4, g_ax4, k), [0 delta], gamma_k8);
    gamma_k8= gamma8(end, :)'; gammax8_int(:,:,k) = gamma_k8;
    [~, gamma9] = ode45(@(t,gamma9) integralGamma(t,gamma9, f_q, f_u, g_q, g_xhi, h_q, h_xhi, h_ax1, g_ax1, k), [0 delta], gamma_k9);
    gamma_k9= gamma9(end, :)'; gammax9_int(:,:,k) = gamma_k9;
    [~, gamma10] = ode45(@(t,gamma10) integralGamma(t,gamma10, f_q, f_u, g_q, g_xhi, h_q, h_xhi, h_ax2, g_ax2, k), [0 delta], gamma_k10);
    gamma_k10= gamma10(end, :)'; gammax10_int(:,:,k) = gamma_k10;
    [~, gamma11] = ode45(@(t,gamma11) integralGamma(t,gamma11, f_q, f_u, g_q, g_xhi, h_q, h_xhi, h_ax3, g_ax3, k), [0 delta], gamma_k11);
    gamma_k11= gamma11(end, :)'; gammax11_int(:,:,k) = gamma_k11;
    [~, gamma12] = ode45(@(t,gamma12) integralGamma(t,gamma12, f_q, f_u, g_q, g_xhi, h_q, h_xhi, h_ax4, g_ax4, k), [0 delta], gamma_k12);
    gamma_k12= gamma12(end, :)'; gammax12_int(:,:,k) = gamma_k12;
end

%% Gamma calculation from integration for every parameter y
gamma_k1 = zeros(6,1); gammay1_int= zeros(6,1,Nstep); gamma_k2 = zeros(6,1); gammay2_int= zeros(6,1,Nstep);
gamma_k3 = zeros(6,1); gammay3_int= zeros(6,1,Nstep); gamma_k4 = zeros(6,1); gammay4_int= zeros(6,1,Nstep);
gamma_k5 = zeros(6,1); gammay5_int= zeros(6,1,Nstep); gamma_k6 = zeros(6,1); gammay6_int= zeros(6,1,Nstep);
gamma_k7 = zeros(6,1); gammay7_int= zeros(6,1,Nstep); gamma_k8 = zeros(6,1); gammay8_int= zeros(6,1,Nstep);
gamma_k9 = zeros(6,1); gammay9_int= zeros(6,1,Nstep); gamma_k10 = zeros(6,1); gammay10_int= zeros(6,1,Nstep);
gamma_k11 = zeros(6,1); gammay11_int= zeros(6,1,Nstep); gamma_k12 = zeros(6,1); gammay12_int= zeros(6,1,Nstep);
for k=1:Nstep
    [~, gamma1] = ode45(@(t,gamma1) integralGamma(t,gamma1, f_q, f_u, g_q, g_xhi, h_q, h_xhi, h_ay1, g_ay1, k), [0 delta], gamma_k1);
    gamma_k1= gamma1(end, :)'; gammay1_int(:,:,k) = gamma_k1;
    [~, gamma2] = ode45(@(t,gamma2) integralGamma(t,gamma2, f_q, f_u, g_q, g_xhi, h_q, h_xhi, h_ay2, g_ay2, k), [0 delta], gamma_k2);
    gamma_k2= gamma2(end, :)'; gammay2_int(:,:,k) = gamma_k2;
    [~, gamma3] = ode45(@(t,gamma3) integralGamma(t,gamma3, f_q, f_u, g_q, g_xhi, h_q, h_xhi, h_ay3, g_ay3, k), [0 delta], gamma_k3);
    gamma_k3= gamma3(end, :)'; gammay3_int(:,:,k) = gamma_k3;
    [~, gamma4] = ode45(@(t,gamma4) integralGamma(t,gamma4, f_q, f_u, g_q, g_xhi, h_q, h_xhi, h_ay4, g_ay4, k), [0 delta], gamma_k4);
    gamma_k4= gamma4(end, :)'; gammay4_int(:,:,k) = gamma_k4;
    [~, gamma5] = ode45(@(t,gamma5) integralGamma(t,gamma5, f_q, f_u, g_q, g_xhi, h_q, h_xhi, h_ay1, g_ay1, k), [0 delta], gamma_k5);
    gamma_k5= gamma5(end, :)'; gammay5_int(:,:,k) = gamma_k5;
    [~, gamma6] = ode45(@(t,gamma6) integralGamma(t,gamma6, f_q, f_u, g_q, g_xhi, h_q, h_xhi, h_ay2, g_ay2, k), [0 delta], gamma_k6);
    gamma_k6= gamma6(end, :)'; gammay6_int(:,:,k) = gamma_k6;
    [~, gamma7] = ode45(@(t,gamma7) integralGamma(t,gamma7, f_q, f_u, g_q, g_xhi, h_q, h_xhi, h_ay3, g_ay3, k), [0 delta], gamma_k7);
    gamma_k7= gamma7(end, :)'; gammay7_int(:,:,k) = gamma_k7;
    [~, gamma8] = ode45(@(t,gamma8) integralGamma(t,gamma8, f_q, f_u, g_q, g_xhi, h_q, h_xhi, h_ay4, g_ay4, k), [0 delta], gamma_k8);
    gamma_k8= gamma8(end, :)'; gammay8_int(:,:,k) = gamma_k8;
    [~, gamma9] = ode45(@(t,gamma9) integralGamma(t,gamma9, f_q, f_u, g_q, g_xhi, h_q, h_xhi, h_ay1, g_ay1, k), [0 delta], gamma_k9);
    gamma_k9= gamma9(end, :)'; gammay9_int(:,:,k) = gamma_k9;
    [~, gamma10] = ode45(@(t,gamma10) integralGamma(t,gamma10, f_q, f_u, g_q, g_xhi, h_q, h_xhi, h_ay2, g_ay2, k), [0 delta], gamma_k10);
    gamma_k10= gamma10(end, :)'; gammay10_int(:,:,k) = gamma_k10;
    [~, gamma11] = ode45(@(t,gamma11) integralGamma(t,gamma11, f_q, f_u, g_q, g_xhi, h_q, h_xhi, h_ay3, g_ay3, k), [0 delta], gamma_k11);
    gamma_k11= gamma11(end, :)'; gammay11_int(:,:,k) = gamma_k11;
    [~, gamma12] = ode45(@(t,gamma12) integralGamma(t,gamma12, f_q, f_u, g_q, g_xhi, h_q, h_xhi, h_ay4, g_ay4, k), [0 delta], gamma_k12);
    gamma_k12= gamma12(end, :)'; gammay12_int(:,:,k) = gamma_k12;
end
toc

%% u_ai vectors creation
u_ax1 = pagemtimes(h_q,gammax1_int(1:3,:,:)) + pagemtimes(h_xhi,gammax1_int(4:6,:,:)) + h_ax1;
u_ax2 = pagemtimes(h_q,gammax2_int(1:3,:,:)) + pagemtimes(h_xhi,gammax2_int(4:6,:,:)) + h_ax2;
u_ax3 = pagemtimes(h_q,gammax3_int(1:3,:,:)) + pagemtimes(h_xhi,gammax3_int(4:6,:,:)) + h_ax3;
u_ax4 = pagemtimes(h_q,gammax4_int(1:3,:,:)) + pagemtimes(h_xhi,gammax4_int(4:6,:,:)) + h_ax4;
u_ax5 = pagemtimes(h_q,gammax5_int(1:3,:,:)) + pagemtimes(h_xhi,gammax5_int(4:6,:,:)) + h_ax1;
u_ax6 = pagemtimes(h_q,gammax6_int(1:3,:,:)) + pagemtimes(h_xhi,gammax6_int(4:6,:,:)) + h_ax2;
u_ax7 = pagemtimes(h_q,gammax7_int(1:3,:,:)) + pagemtimes(h_xhi,gammax7_int(4:6,:,:)) + h_ax3;
u_ax8 = pagemtimes(h_q,gammax8_int(1:3,:,:)) + pagemtimes(h_xhi,gammax8_int(4:6,:,:)) + h_ax4;
u_ax9 = pagemtimes(h_q,gammax9_int(1:3,:,:)) + pagemtimes(h_xhi,gammax9_int(4:6,:,:)) + h_ax1;
u_ax10 = pagemtimes(h_q,gammax10_int(1:3,:,:)) + pagemtimes(h_xhi,gammax10_int(4:6,:,:)) + h_ax2;
u_ax11 = pagemtimes(h_q,gammax11_int(1:3,:,:)) + pagemtimes(h_xhi,gammax11_int(4:6,:,:)) + h_ax3;
u_ax12 = pagemtimes(h_q,gammax12_int(1:3,:,:)) + pagemtimes(h_xhi,gammax12_int(4:6,:,:)) + h_ax4;
%y
u_ay1 = pagemtimes(h_q,gammay1_int(1:3,:,:)) + pagemtimes(h_xhi,gammay1_int(4:6,:,:)) + h_ay1;
u_ay2 = pagemtimes(h_q,gammay2_int(1:3,:,:)) + pagemtimes(h_xhi,gammay2_int(4:6,:,:)) + h_ay2;
u_ay3 = pagemtimes(h_q,gammay3_int(1:3,:,:)) + pagemtimes(h_xhi,gammay3_int(4:6,:,:)) + h_ay3;
u_ay4 = pagemtimes(h_q,gammay4_int(1:3,:,:)) + pagemtimes(h_xhi,gammay4_int(4:6,:,:)) + h_ay4;
u_ay5 = pagemtimes(h_q,gammay5_int(1:3,:,:)) + pagemtimes(h_xhi,gammay5_int(4:6,:,:)) + h_ay1;
u_ay6 = pagemtimes(h_q,gammay6_int(1:3,:,:)) + pagemtimes(h_xhi,gammay6_int(4:6,:,:)) + h_ay2;
u_ay7 = pagemtimes(h_q,gammay7_int(1:3,:,:)) + pagemtimes(h_xhi,gammay7_int(4:6,:,:)) + h_ay3;
u_ay8 = pagemtimes(h_q,gammay8_int(1:3,:,:)) + pagemtimes(h_xhi,gammay8_int(4:6,:,:)) + h_ay4;
u_ay9 = pagemtimes(h_q,gammay9_int(1:3,:,:)) + pagemtimes(h_xhi,gammay9_int(4:6,:,:)) + h_ay1;
u_ay10 = pagemtimes(h_q,gammay10_int(1:3,:,:)) + pagemtimes(h_xhi,gammay10_int(4:6,:,:)) + h_ay2;
u_ay11 = pagemtimes(h_q,gammay11_int(1:3,:,:)) + pagemtimes(h_xhi,gammay11_int(4:6,:,:)) + h_ay3;
u_ay12 = pagemtimes(h_q,gammay12_int(1:3,:,:)) + pagemtimes(h_xhi,gammay12_int(4:6,:,:)) + h_ay4;

%% Save for optimization
save('data/sens','sens_int')
save('data/gamma',"gammax1_int", "gammay1_int","gammax2_int", "gammay2_int","gammax3_int", "gammay3_int","gammax4_int", "gammay4_int","gammax5_int", "gammay5_int","gammax6_int", "gammay6_int",...
    "gammax7_int", "gammay7_int","gammax8_int", "gammay8_int","gammax9_int", "gammay9_int","gammax10_int", "gammay10_int","gammax11_int", "gammay11_int","gammax12_int", "gammay12_int")
save('data/u_ai',"u_ax1", "u_ay1","u_ax2", "u_ay2","u_ax3", "u_ay3","u_ax4", "u_ay4","u_ax5", "u_ay5","u_ax6", "u_ay6",...
    "u_ax7", "u_ay7","u_ax8", "u_ay8","u_ax9", "u_ay9","u_ax10", "u_ay10","u_ax11", "u_ay11","u_ax12", "u_ay12")

%% Utility Functions 
function didt=integralSens(~, i, f_q, f_p, f_u, g_q, g_xhi, h_q, h_xhi, k)
    % Create matrixes from column vector
    sens = [i(1) i(2);
            i(3) i(4);
            i(5) i(6)];
    sensxhi = [i(7) i(8);
               i(9) i(10);
               i(11) i(12)];
    % Sensitivity component of the ODEs  
    dsens = f_p(:,:,k) + f_q(:,:,k)*sens + f_u(:,:,k)*(h_q(:,:,k)*sens + h_xhi(:,:,k)*sensxhi);
    % sensitivity_xhi component of the ODEs
    dsensxhi = g_q(:,:,k)*sens + g_xhi(:,:,k)*sensxhi;
    % Output
    didt = [dsens(1,1); dsens(1,2); dsens(2,1);dsens(2,2);dsens(3,1); dsens(3,2);dsensxhi(1,1); dsensxhi(1,2); dsensxhi(2,1);dsensxhi(2,2);dsensxhi(3,1);dsensxhi(3,2)];
end 

function dgamma_tot=integralGamma(~, gamma, f_q, f_u, g_q, g_xhi, h_q, h_xhi, h_a_i, g_a_i, k) 
    t_maius = [gamma(1:3)]; t_maius_xhi = [gamma(4:6)]; 
    %Gamma component of the ODEs
    dgamma = f_q(:,:,k)*t_maius + f_u(:,:,k)*(h_q(:,:,k)*t_maius + h_xhi(:,:,k)*t_maius_xhi + h_a_i(:,:,k));
    %Gamma_xhi component of the ODEs
    dgammaxhi = g_q(:,:,k)*t_maius + g_xhi(:,:,k)*t_maius_xhi + g_a_i(:,:,k);
    %Output
    dgamma_tot = [dgamma(1); dgamma(2); dgamma(3); dgammaxhi(1); dgammaxhi(2); dgammaxhi(3)];
end