close all; clc;
% Symbolic variables declaration 
syms x_d y_d dx_d dy_d ddx_d ddy_d t x y theta b r wl wr xhi_v xhi_x xhi_y
syms q [3 1], syms p [2 1], syms u [2 1] , syms xhi [3 1] 
syms r_d [2 1], syms dr_d [2 1], syms ddr_d [2 1]
syms a1x a1y a2x a2y a3x a3y a4x a4y 

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

eta = ddr_d + kv*(dr_d - dr_xhi) + kp*(r_d - q(1:2)) + ki*xhi(2:3);

% xhi_dot = g(xhi,q,r_d)
xhi_dot = [[1 0]*inv(A)*eta; 
            r_d - q(1:2)];

% u = h(xhi,q,r_d,p)
new_u = inv(S)*[xhi_v; [0 1]*inv(A)*eta];

% Partial Derivatives for sensitivity
f_p = jacobian(q_dot,p); f_q = jacobian(q_dot,q); f_u = jacobian(q_dot,u);
h_q = jacobian(new_u,q);  h_xhi = jacobian(new_u,xhi);
g_q = jacobian(xhi_dot,q); g_xhi = jacobian(xhi_dot,xhi);

% Second Partial Derivative
syms g [3 1], syms u_ai [2 1], 
df_q_q_gamma = tensor_product(f_q,q,g);
df_q_u_uai = tensor_product(f_q,u,u_ai);
df_p_q_gamma = tensor_product(f_p,q,g);
df_p_u_uai = tensor_product(f_p,u,u_ai);
df_u_q_gamma = tensor_product(f_u,q,g);
df_u_u_uai = tensor_product(f_u,u,u_ai);
dh_q_q_gamma = tensor_product(h_q,q,g);
dh_q_xhi_gammaxhi = tensor_product(h_q,xhi,g);
dh_xhi_q_gamma = tensor_product(h_xhi, q,g);
dh_xhi_xhi_gammaxhi = tensor_product(h_xhi,xhi,g);
dg_q_q_gamma = tensor_product(g_q,q,g);
dg_q_xhi_gammaxhi = tensor_product(g_q,xhi, g);
dg_xhi_q_gamma = tensor_product(g_xhi,q,g);
dg_xhi_xhi_gammaxhi = tensor_product(g_xhi,xhi,g);

%% Function creation
% State and Control
matlabFunction(q_dot,'File','auto_functions/q_dot','Vars',{q,u,p});
matlabFunction(new_u,'File','auto_functions/new_u','Vars',{q,xhi,r_d,dr_d,ddr_d,p});
matlabFunction(xhi_dot,'File','auto_functions/xhi_dot','Vars',{q,xhi,r_d,dr_d,ddr_d});

% Derivatives for sensitivity
matlabFunction(f_p,'File','auto_functions/ff_p');
matlabFunction(f_q,'File','auto_functions/ff_q');
matlabFunction(f_u,'File','auto_functions/ff_u');
matlabFunction(h_q,'File','auto_functions/fh_q');
matlabFunction(h_xhi,'File','auto_functions/fh_xhi');
matlabFunction(g_q,'File','auto_functions/fg_q');
matlabFunction(g_xhi,'File','auto_functions/fg_xhi');

% Tensor Products
matlabFunction(df_q_q_gamma,'File','auto_functions/df_q_q_gamma');
matlabFunction(df_q_u_uai,'File','auto_functions/df_q_u_uai');
matlabFunction(df_p_q_gamma,'File','auto_functions/df_p_q_gamma');
matlabFunction(df_p_u_uai,'File','auto_functions/df_p_u_uai');
matlabFunction(df_u_q_gamma,'File','auto_functions/df_u_q_gamma');
matlabFunction(df_u_u_uai,'File','auto_functions/df_u_u_uai');
matlabFunction(dh_q_q_gamma,'File','auto_functions/dh_q_q_gamma');
matlabFunction(dh_q_xhi_gammaxhi,'File','auto_functions/dh_q_xhi_gammaxhi');
matlabFunction(dh_xhi_q_gamma,'File','auto_functions/dh_xhi_q_gamma');
matlabFunction(dh_xhi_xhi_gammaxhi,'File','auto_functions/dh_xhi_xhi_gammaxhi');
matlabFunction(dg_q_q_gamma,'File','auto_functions/dg_q_q_gamma');
matlabFunction(dg_q_xhi_gammaxhi,'File','auto_functions/dg_q_xhi_gammaxhi');
matlabFunction(dg_xhi_q_gamma,'File','auto_functions/dg_xhi_q_gamma');
matlabFunction(dg_xhi_xhi_gammaxhi,'File','auto_functions/dg_xhi_xhi_gammaxhi');

% Desired parametric trajectory
x_d = a1x*(t^3) + a2x*(t^2) + a3x*t + a4x;
y_d = a1y*(t^3) + a2y*(t^2) + a3y*t + a4y;
dx_d = diff(x_d,t); dy_d = diff(y_d,t);
ddx_d = diff(x_d,2,t); ddy_d = diff(y_d,2,t);

% Reassign
r_d(1) = x_d; r_d(2) = y_d;
dr_d(1) = dx_d; dr_d(2) = dy_d;
ddr_d(1) = ddx_d; ddr_d(2) = ddy_d;

% Rewrite functions to find new equations
eta = ddr_d + kv*(dr_d - dr_xhi) + kp*(r_d - q(1:2)) + ki*xhi(2:3);
xhi_dot = [[1 0]*inv(A)*eta; r_d - q(1:2)];
new_u = inv(S)*[xhi_v; [0 1]*inv(A)*eta];
