close all; clc;
%% Load from data
file = 'data/sens';
sens = load(file,'sens_int' );
sens = sens.sens_int;
file2 = 'data/sensitivity_ai';
sens_aix = load(file2, "all_sensaix");
sens_aiy = load(file2, "all_sensaiy");
sens_aix = sens_aix.all_sensaix;
sens_aiy = sens_aiy.all_sensaiy;
file3 ='data/coeff_a';
ax = load(file3,'a_x');
ax0 = ax. a_x;
ay = load(file3,'a_y');
ay0 = ay.a_y;
%% Hyperparameters 
k1 = 0.1; k2 = 0.1;

%% Optimization
dx = [2 0.1 0 0 0 0 10 0.1]';
dy = [4 0.1 0 0 0 0 20 0.1]';

M = [0 0 0 1 0 0 0 0 0 0 0 0;
     0 0 1 0 0 0 0 0 0 0 0 0;
     (tsim/3)^3 (tsim/3)^2 tsim/3 1 0 0 0 -1 0 0 0 0;
     3*(tsim/3)^2 2*tsim/3 1 0 0 0 -1 0 0 0 0 0;
     0 0 0 0 (tsim/3)^3 (tsim/3)^2 tsim/3 1 0 0 0 -1;
     0 0 0 0 3*((tsim/3)^2) 2*(tsim/3) 1 0 0 0 -1 0;
     0 0 0 0 0 0 0 0 (tsim/3)^3 (tsim/3)^2 (tsim/3) 1;
     0 0 0 0 0 0 0 0 3*((tsim/3)^2) 2*(tsim/3) 1 0];

%% Calculate vi for x and y parameters
sens_last = [sens(1,Nstep) sens(2,Nstep);
             sens(3,Nstep) sens(4,Nstep);
             sens(5,Nstep) sens(6,Nstep)];

vx = zeros(12,1);
for i= 1: length(sens_aix)
    sensai_last = reshape(sens_aix{i}(1:6,Nstep),2,[])';
    vx(i) = -trace(sens_last'*sensai_last);
end

vy = zeros(12,1);
for i= 1: length(sens_aiy)
    sensai_last = reshape(sens_aiy{i}(1:6,Nstep),2,[])';
    vy(i) = -trace(sens_last'*sensai_last);
end

%% Compute new optimized parameter vectors ax and ay
I = eye(12); 
ax_k = zeros(12,Nstep); ax_k(:,1)= ax0; 
ay_k = zeros(12,Nstep); ay_k(:,1)= ay0;

for n = 2:Nstep
ax_k(:,n) = ax_k(:,n-1) + delta/1000*(k1*pinv(M)*(dx-M*ax_k(:,n-1)) + k2*(I - pinv(M)*M)*vx);
ay_k(:,n) = ay_k(:,n-1) + delta/1000*(k1*pinv(M)*(dx-M*ay_k(:,n-1)) + k2*(I - pinv(M)*M)*vy);
end 

ax_star = ax_k(:,Nstep);
ay_star = ay_k(:,Nstep);

%% Visualize optimized trajectories
breaks = 0:tsim/3:tsim;
polyx = mkpp(breaks,[ax_star(1) ax_star(2) ax_star(3) ax_star(4);
                     ax_star(5) ax_star(6) ax_star(7) ax_star(8);
                     ax_star(9) ax_star(10) ax_star(11) ax_star(12)]);
polyy = mkpp(breaks,[ay_star(1) ay_star(2) ay_star(3) ay_star(4);
                     ay_star(5) ay_star(6) ay_star(7) ay_star(8);
                     ay_star(9) ay_star(10) ay_star(11) ay_star(12)]);

figure(),
fnplt(polyx,linewidth), hold on
fnplt(polyy,linewidth)
line([tsim/3 tsim/3],ylim,'LineStyle','--','Color','k','LineWidth',1), grid minor
line([2*tsim/3 2*tsim/3],ylim,'LineStyle','--','Color','k','LineWidth',1)
xlabel('time [sec]'), ylabel('trajecory [m]')
legend('trajectory in x', 'trajectory in y')
title('Trajectory varation in time'),hold off

% Differentiation first order
dpolyx = fnder(polyx);
dpolyy = fnder(polyy);

% Differentiation second order
ddpolyx = fnder(dpolyx);
ddpolyy = fnder(dpolyy);

p = [ppval(polyx,time);ppval(polyy,time)];
dp = [ppval(dpolyx,time);ppval(dpolyy,time)];
ddp = [ppval(ddpolyx,time);ppval(ddpolyy,time)];

% Optimized trajectory
figure()
plot(p(1,:),p(2,:));
xlabel('x [m]'), ylabel('y [m]'), grid minor
title('Optimized Trajectory')

%% Save optimized coefficients and new trajectory
save('data/coeff_a_star',"ay_star","ay_star")
save('data/new_traj',"p","dp","ddp")
