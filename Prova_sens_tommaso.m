close all; clc;
file = 'data/sensitivity_deriv2';
f_q = load(file,'f_q' );
f_p = load(file,'f_p' );
f_u = load(file, 'f_u');
g_q = load(file,'g_q' );
g_xhi = load(file,'g_xhi' );
h_q = load(file,'h_q' );
h_xhi = load(file,'h_xhi' );

f_q = f_q.f_q;
f_p = f_p.f_p;
f_u =f_u.f_u;
g_q = g_q.g_q;
g_xhi = g_xhi.g_xhi;
h_q = h_q.h_q;
h_xhi = h_xhi.h_xhi;


sens_k = zeros(6,1); sens_int= zeros(6, Nstep,Nstep); sens_fin=zeros(3,Nstep);
for j=1:Nstep
    disp(j)
for k=1:Nstep
    [t_s, i] = ode45(@(t,i) integralSens_Tommy(t, i, f_q, f_p, f_u, g_q, g_xhi, h_q, h_xhi, k,j), [0 delta], sens_k);
    sens_k= i(end, :)'; sens_int(:,k,j) = sens_k;
end
sens_fin(:,j)=sens_int(1:3,end,j);
end
% Optimized trajectory
sens_plot=zeros(1,Nstep);
for i=1:Nstep
    sens_plot(:,i)=0.5*trace(sens_fin(:,i)'*sens_fin(:,i));
end
plot_function(sens_plot,'Sensitivity variation in time','cost function', timeVec, linewidth, colors, counter)
