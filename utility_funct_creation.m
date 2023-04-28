%% In this file we compute all the derivation matrix used in the calculation of the two sensitivity in a simbolic way to generate 
%% then the Matlab Functions. With the use of them, in optimizationRoutine file, we compute the values of the different matrices in time.
close all; clc;
% Symbolic variables declaration 
syms x_d y_d dx_d dy_d ddx_d ddy_d t x y theta b r wl wr xhi_v xhi_x xhi_y
syms q [3 1], syms p [2 1], syms u [2 1], syms xhi [3 1] 
syms r_d [2 1], syms dr_d [2 1], syms ddr_d [2 1]
syms a1x a1y a2x a2y a3x a3y a4x a4y a5x a5y
syms k_i k_p k_v

% Assignment
q(1) = x; q(2) = y; q(3) = theta;
p(1) = r; p(2) = b;
u(1) = wr; u(2) = wl;
xhi(1) = xhi_v; xhi(2) = xhi_x; xhi(3) = xhi_y;
r_d(1) = x_d; r_d(2) = y_d;
dr_d(1) = dx_d; dr_d(2) = dy_d;
ddr_d(1) = ddx_d; ddr_d(2) = ddy_d;

% Equations
G = [ cos(theta) 0;
      sin(theta) 0;
      0          1];

S = [r/2            r/2;        
     r/(2*b)   -r/(2*b)];

A = [cos(theta) -xhi_v*sin(theta);
     sin(theta)  xhi_v*cos(theta)];

% Dynamic model
q_dot = G*S*u;

% Controller
dr_xhi = [cos(theta)*xhi_v;
          sin(theta)*xhi_v];

eta = ddr_d + k_v*(dr_d - dr_xhi) + k_p*(r_d - q(1:2)) + k_i*xhi(2:3);

% xhi_dot = g(xhi,q,r_d)
xhi_dot = [[1 0]*inv(A)*eta; 
            r_d - q(1:2)]

% u = h(xhi,q,r_d,p)
new_u = inv(S)*[xhi_v; 
               [0 1]*inv(A)*eta];
               
% Partial Derivatives for sensitivity
f_p = jacobian(q_dot,p)
f_q = jacobian(q_dot,q)
f_u = jacobian(q_dot,u)

h_q = jacobian(new_u,q)
h_xhi = jacobian(new_u,xhi)

g_q = jacobian(xhi_dot,q)
g_xhi = jacobian(xhi_dot,xhi)


%% Function creation
% State and Control
matlabFunction(q_dot,'File','auto_functions/q_dot','Vars',{q,u,p,k_v,k_i,k_p});
matlabFunction(new_u,'File','auto_functions/new_u','Vars',{q,xhi,r_d,dr_d,ddr_d,p,k_v,k_i,k_p});
matlabFunction(xhi_dot,'File','auto_functions/xhi_dot','Vars',{q,xhi,r_d,dr_d,ddr_d,k_v,k_i,k_p});

% Derivatives for sensitivity
matlabFunction(f_p,'File','auto_functions/ff_p', 'Vars',{q,u,p});
matlabFunction(f_q,'File','auto_functions/ff_q', 'Vars',{q,u,p});
matlabFunction(f_u,'File','auto_functions/ff_u', 'Vars',{q,p});
matlabFunction(h_q,'File','auto_functions/fh_q', 'Vars',{q,xhi,r_d,dr_d,ddr_d,p,k_v,k_i,k_p});
matlabFunction(h_xhi,'File','auto_functions/fh_xhi', 'Vars', {q,xhi,r_d,dr_d,ddr_d, p,k_v,k_i,k_p});
matlabFunction(g_q,'File','auto_functions/fg_q', 'Vars', {q,xhi,r_d,dr_d,ddr_d,k_v,k_i,k_p});
matlabFunction(g_xhi,'File','auto_functions/fg_xhi','Vars',{q,k_v,k_i,k_p});

% Tensor product creation 
syms g [3 1], syms u_ai [2 1], 
df_q_q_gamma_ = tensor_product(f_q,q,g)
df_q_u_uai_ = tensor_product(f_q,u,u_ai)
df_p_q_gamma_ = tensor_product(f_p,q,g)
df_p_u_uai_ = tensor_product(f_p,u,u_ai)
df_u_q_gamma_ = tensor_product(f_u,q,g)
df_u_u_uai_ = tensor_product(f_u,u,u_ai)
dh_q_q_gamma_ = tensor_product(h_q,q,g)
dh_q_xhi_gammaxhi_ = tensor_product(h_q,xhi,g)
dh_xhi_q_gamma_ = tensor_product(h_xhi, q,g)
dh_xhi_xhi_gammaxhi_ = tensor_product(h_xhi,xhi,g)
dg_q_q_gamma_ = tensor_product(g_q,q,g)
dg_q_xhi_gammaxhi_ = tensor_product(g_q,xhi, g)
dg_xhi_q_gamma_ = tensor_product(g_xhi,q,g)
dg_xhi_xhi_gammaxhi_ = tensor_product(g_xhi,xhi,g)

% Tensor Products
matlabFunction(df_q_q_gamma_,'File','auto_functions/df_q_q_gamma','Vars',{g,p,u,q});
matlabFunction(df_q_u_uai_,'File','auto_functions/df_q_u_uai','Vars',{q,u_ai,p});
matlabFunction(df_p_q_gamma_,'File','auto_functions/df_p_q_gamma','Vars',{g,u,q});
matlabFunction(df_p_u_uai_,'File','auto_functions/df_p_u_uai','Vars',{u_ai,q,p});
matlabFunction(df_u_q_gamma_,'File','auto_functions/df_u_q_gamma','Vars',{g,p,q});
matlabFunction(df_u_u_uai_,'File','auto_functions/df_u_u_uai','Vars',{});
matlabFunction(dh_q_q_gamma_,'File','auto_functions/dh_q_q_gamma','Vars',{g,q,xhi,r_d,dr_d,ddr_d,p,k_v,k_i,k_p});
matlabFunction(dh_q_xhi_gammaxhi_,'File','auto_functions/dh_q_xhi_gammaxhi','Vars',{g,q,xhi,r_d,dr_d,ddr_d,p,k_v,k_i,k_p});
matlabFunction(dh_xhi_q_gamma_,'File','auto_functions/dh_xhi_q_gamma','Vars',{g,q,xhi,r_d,dr_d,ddr_d,p,k_v,k_i,k_p});
matlabFunction(dh_xhi_xhi_gammaxhi_,'File','auto_functions/dh_xhi_xhi_gammaxhi','Vars',{g,q,xhi,r_d,dr_d,ddr_d, p,k_v,k_i,k_p});
matlabFunction(dg_q_q_gamma_,'File','auto_functions/dg_q_q_gamma','Vars',{g,q,xhi,r_d,dr_d,ddr_d,k_v,k_i,k_p});
matlabFunction(dg_q_xhi_gammaxhi_,'File','auto_functions/dg_q_xhi_gammaxhi','Vars',{g,q,k_v,k_i,k_p});
matlabFunction(dg_xhi_q_gamma_,'File','auto_functions/dg_xhi_q_gamma','Vars',{g,q,k_v,k_i,k_p});
matlabFunction(dg_xhi_xhi_gammaxhi_,'File','auto_functions/dg_xhi_xhi_gammaxhi','Vars',{});

%% Here we make explicit the dependence with respect to the coefficients of the trajectory ai so as to properly perform the derivatives with respect to these
% Desired parametric trajectory
x_d = a1x*(t^4) + a2x*(t^3) + a3x*(t^2) + a4x*t + a5x;
y_d = a1y*(t^4) + a2y*(t^3) + a3y*(t^2) + a4y*t + a5y;
dx_d = diff(x_d,t); dy_d = diff(y_d,t);
ddx_d = diff(x_d,2,t); ddy_d = diff(y_d,2,t);

% Reassign
r_d(1) = x_d; r_d(2) = y_d;
dr_d(1) = dx_d; dr_d(2) = dy_d;
ddr_d(1) = ddx_d; ddr_d(2) = ddy_d;

% Rewrite functions to find new equations
eta = ddr_d + k_v*(dr_d - dr_xhi) + k_p*(r_d - q(1:2)) + k_i*xhi(2:3);

xhi_dot = [[1 0]*inv(A)*eta;
           r_d - q(1:2)];

new_u = inv(S)*[xhi_v; 
               [0 1]*inv(A)*eta]

% Partial Derivatives for gamma
h_a_1x_ = jacobian(new_u,a1x) 
h_a_1y_ = jacobian(new_u,a1y)
h_a_2x_ = jacobian(new_u,a2x)
h_a_2y_ = jacobian(new_u,a2y)
h_a_3x_ = jacobian(new_u,a3x)
h_a_3y_ = jacobian(new_u,a3y)
h_a_4x_ = jacobian(new_u,a4x)
h_a_4y_ = jacobian(new_u,a4y)
h_a_5x_ = jacobian(new_u,a5x)
h_a_5y_ = jacobian(new_u,a5y)

g_a_1x_ = jacobian(xhi_dot,a1x) 
g_a_1y_ = jacobian(xhi_dot,a1y)
g_a_2x_ = jacobian(xhi_dot,a2x) 
g_a_2y_ = jacobian(xhi_dot,a2y)
g_a_3x_ = jacobian(xhi_dot,a3x) 
g_a_3y_ = jacobian(xhi_dot,a3y)
g_a_4x_ = jacobian(xhi_dot,a4x) 
g_a_4y_ = jacobian(xhi_dot,a4y)
g_a_5x_ = jacobian(xhi_dot,a5x) 
g_a_5y_ = jacobian(xhi_dot,a5y)

%% Function creation
% Derivatives for gamma
matlabFunction(h_a_1x_,'File','auto_functions/h_a_1x','Vars',{q,t,xhi,p,k_v,k_p});
matlabFunction(h_a_1y_,'File','auto_functions/h_a_1y','Vars',{q,t,xhi,p,k_v,k_p});
matlabFunction(h_a_2x_,'File','auto_functions/h_a_2x','Vars',{q,t,xhi,p,k_v,k_p});
matlabFunction(h_a_2y_,'File','auto_functions/h_a_2y','Vars',{q,t,xhi,p,k_v,k_p});
matlabFunction(h_a_3x_,'File','auto_functions/h_a_3x','Vars',{q,t,xhi,p,k_v,k_p});
matlabFunction(h_a_3y_,'File','auto_functions/h_a_3y','Vars',{q,t,xhi,p,k_v,k_p});
matlabFunction(h_a_4x_,'File','auto_functions/h_a_4x','Vars',{q,t,xhi,p,k_v,k_p});
matlabFunction(h_a_4y_,'File','auto_functions/h_a_4y','Vars',{q,t,xhi,p,k_v,k_p});
matlabFunction(h_a_5x_,'File','auto_functions/h_a_5x','Vars',{q,xhi,p,k_p});
matlabFunction(h_a_5y_,'File','auto_functions/h_a_5y','Vars',{q,xhi,p,k_p});

matlabFunction(g_a_1x_,'File','auto_functions/g_a_1x','Vars',{q,t,k_v,k_p});
matlabFunction(g_a_1y_,'File','auto_functions/g_a_1y','Vars',{q,t,k_v,k_p});
matlabFunction(g_a_2x_,'File','auto_functions/g_a_2x','Vars',{q,t,k_v,k_p});
matlabFunction(g_a_2y_,'File','auto_functions/g_a_2y','Vars',{q,t,k_v,k_p});
matlabFunction(g_a_3x_,'File','auto_functions/g_a_3x','Vars',{q,t,k_v,k_p});
matlabFunction(g_a_3y_,'File','auto_functions/g_a_3y','Vars',{q,t,k_v,k_p});
matlabFunction(g_a_4x_,'File','auto_functions/g_a_4x','Vars',{q,t,k_v,k_p});
matlabFunction(g_a_4y_,'File','auto_functions/g_a_4y','Vars',{q,t,k_v,k_p});
matlabFunction(g_a_5x_,'File','auto_functions/g_a_5x','Vars',{q,k_p});
matlabFunction(g_a_5y_,'File','auto_functions/g_a_5y','Vars',{q,k_p});


% Derivatives for h,g that depends on parameters ai
% derivatives of h_q with respect to the coefficients ai --> dh_q_ai
hq = jacobian(new_u,q);
for i= 1:5
    ai = eval(sprintf('a%dx', i));
    func = my_jacobian(hq,ai);
    matlabFunction(func,'File',sprintf('auto_functions/dhq_a%dx', i));
end
for i= 1:5
    ai = eval(sprintf('a%dy', i));
    func = my_jacobian(hq,ai);
    matlabFunction(func,'File',sprintf('auto_functions/dhq_a%dy', i));
end

% derivatives of h_xhi with respect to the coefficients ai --> dh_xhi_ai
hxhi = jacobian(new_u,xhi);
for i= 1:5
    ai = eval(sprintf('a%dx', i));
    func = my_jacobian(hxhi,ai);
    matlabFunction(func,'File',sprintf('auto_functions/dhxhi_a%dx', i));
end
for i= 1:5
    ai = eval(sprintf('a%dy', i));
    func = my_jacobian(hxhi,ai);
    matlabFunction(func,'File',sprintf('auto_functions/dhxhi_a%dy', i));
end

% derivatives of g_q with respect to the coefficients ai --> dg_q_ai
gq = jacobian(xhi_dot,q);
for i= 1:5
    ai = eval(sprintf('a%dx', i));
    func = my_jacobian(gq,ai);
    matlabFunction(func,'File',sprintf('auto_functions/dgq_a%dx', i));
end
for i= 1:5
    ai = eval(sprintf('a%dy', i));
    func = my_jacobian(gq,ai);
    matlabFunction(func,'File',sprintf('auto_functions/dgq_a%dy', i));
end

% % derivatives of g_q with respect to the coefficients ai --> dg_xhi_ai
gxhi = jacobian(xhi_dot,xhi);
for i= 1:5
    ai = eval(sprintf('a%dx', i));
    func = my_jacobian(gxhi,ai);
    matlabFunction(func,'File',sprintf('auto_functions/dgxhi_a%dx', i));
end
for i= 1:5
    ai = eval(sprintf('a%dy', i));
    func = my_jacobian(gxhi,ai);
    matlabFunction(func,'File',sprintf('auto_functions/dgxhi_a%dy', i));
end
