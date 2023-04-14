%% In this file we compute all the derivation matrix used in the calculation of the two sensitivity in a simbolic way to generate 
%% then the Matlab Functions. With the use of them, in optimizationRoutine file, we compute the values of the different matrices in time.
close all; clc;
% Symbolic variables declaration 
syms x_d y_d dx_d dy_d ddx_d ddy_d t x y theta b r wl wr xhi_v xhi_x xhi_y
syms q [3 1], syms p [2 1], syms u [2 1], syms xhi [3 1] 
syms r_d [2 1], syms dr_d [2 1], syms ddr_d [2 1]
syms a1x a1y a2x a2y a3x a3y a4x a4y a5x a5y 

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
new_u = inv(S)*[xhi_v; 
               [0 1]*inv(A)*eta];

% Partial Derivatives for sensitivity
f_p = jacobian(q_dot,p); f_q = jacobian(q_dot,q); f_u = jacobian(q_dot,u);
h_q = jacobian(new_u,q); h_xhi = jacobian(new_u,xhi);
g_q = jacobian(xhi_dot,q); g_xhi = jacobian(xhi_dot,xhi);

%% Function creation
% State and Control
matlabFunction(q_dot,'File','auto_functions/q_dot','Vars',{q,u,p});
matlabFunction(new_u,'File','auto_functions/new_u','Vars',{q,xhi,r_d,dr_d,ddr_d,p});
matlabFunction(xhi_dot,'File','auto_functions/xhi_dot','Vars',{q,xhi,r_d,dr_d,ddr_d});
