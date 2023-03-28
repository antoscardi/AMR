close all; clc;
% Symbolic variables declaration 
syms x_d y_d dx_d dy_d ddx_d ddy_d t x y theta b r wl wr xhi_v xhi_x xhi_y
syms q [3 1], syms p [2 1], syms u [2 1] real, syms xhi [2 1] 
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

S_c = [ r_n/2          r_n/2;        
        r_n/(2*b_n)   -r_n/(2*b_n)];

S = [r/2            r/2;        
     r/(2*b)   -r/(2*b)];

A = [cos(theta) -xhi_v*sin(theta);
     sin(theta)  xhi_v*cos(theta)];

% Dynamic model
q_dot = G*S*u;
% Calculate v and w
velocities = S*u;

% Controller
dr_xhi = [cos(theta)*xhi_v;
          sin(theta)*xhi_v];

eta1 = ddx_d + kv*(dx_d - dr_xhi(1)) + kp*(x_d - x) + ki*xhi_x;
eta2 = ddy_d + kv*(dy_d - dr_xhi(2)) + kp*(y_d - y) + ki*xhi_y;
eta = [eta1; 
       eta2];

% xhi_dot = g(xhi,q,r_d)
xhi_dot = [[1 0]*inv(A)*eta; 
            [x_d; y_d] - [x; y]];

% u = h(xhi,q,r_d,p)
new_u = inv(S_c)*[xhi_v;
        [0 1]*inv(A)*eta];

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
dh_xhi_q_gamma = tensor_product(h_xhi,q,g);
dh_xhi_xhi_gammaxhi = tensor_product(h_xhi,xhi,g);
dg_q_q_gamma = tensor_product(g_q,q,g);
dg_q_xhi_gammaxhi = tensor_product(g_q,xhi,g);
dg_xhi_q_gamma = tensor_product(g_xhi,q,g);
dg_xhi_xhi_gammaxhi = tensor_product(g_xhi,xhi,g);

%% Function creation
% State and Control
matlabFunction(velocities,'File','functions/velocities','Vars',{p,u});
matlabFunction(q_dot,'File','functions/q_dot','Vars',{q,u,p});
matlabFunction(new_u,'File','functions/new_u','Vars',{q,xhi,r_d,dr_d,ddr_d});
matlabFunction(xhi_dot,'File','functions/xhi_dot','Vars',{q,xhi,r_d,dr_d,ddr_d});

% Derivatives for sensitivity
matlabFunction(f_p,'File','functions/ff_p','Vars',{q,u,p});
matlabFunction(f_q,'File','functions/ff_q','Vars',{q,u,p});
matlabFunction(f_u,'File','functions/ff_u','Vars',{q,p});
matlabFunction(h_q,'File','functions/fh_q','Vars',{q,xhi,r_d,dr_d,ddr_d});
matlabFunction(h_xhi,'File','functions/fh_xhi','Vars',{q,xhi,r_d,dr_d,ddr_d});
matlabFunction(g_q,'File','functions/fg_q','Vars',{q,xhi,r_d,dr_d,ddr_d});
matlabFunction(g_xhi,'File','functions/fg_xhi','Vars',{q,xhi,r_d,dr_d,ddr_d});

% Tensor Products
matlabFunction(df_q_q_gamma,'File','functions/df_q_q_gamma','Vars',{g,p,u,q});
matlabFunction(df_q_u_uai,'File','functions/df_q_u_uai','Vars',{q,u_ai,p});
matlabFunction(df_p_q_gamma,'File','functions/df_p_q_gamma','Vars',{g,u,q});
matlabFunction(df_p_u_uai,'File','functions/df_p_u_uai','Vars',{u_ai,q,p});
matlabFunction(df_u_q_gamma,'File','functions/df_u_q_gamma','Vars',{g,p,q});
matlabFunction(df_u_u_uai,'File','functions/df_u_u_uai','Vars',{});
matlabFunction(dh_q_q_gamma,'File','functions/dh_q_q_gamma','Vars',{g,q,xhi,r_d,dr_d,ddr_d});
matlabFunction(dh_q_xhi_gammaxhi,'File','functions/dh_q_xhi_gammaxhi','Vars',{g,q,xhi,r_d,dr_d,ddr_d});
matlabFunction(dh_xhi_q_gamma,'File','functions/dh_xhi_q_gamma','Vars',{g,q,xhi,r_d,dr_d,ddr_d});
matlabFunction(dh_xhi_xhi_gammaxhi,'File','functions/dh_xhi_xhi_gammaxhi','Vars',{g,q,xhi,r_d,dr_d,ddr_d});
matlabFunction(dg_q_q_gamma,'File','functions/dg_q_q_gamma','Vars',{g,q,xhi,r_d,dr_d,ddr_d});
matlabFunction(dg_q_xhi_gammaxhi,'File','functions/dg_q_xhi_gammaxhi','Vars',{g,q});
matlabFunction(dg_xhi_q_gamma,'File','functions/dg_xhi_q_gamma','Vars',{g,q});
matlabFunction(dg_xhi_xhi_gammaxhi,'File','functions/dg_xhi_xhi_gammaxhi','Vars',{});

% Desired parametric trajectory
x_d = a1x*(t^3) + a2x*(t^2) + a3x*t + a4x;
y_d = a1y*(t^3) + a2y*(t^2) + a3y*t + a4y;
dx_d = diff(x_d,t); dy_d = diff(y_d,t);
ddx_d = diff(x_d,2,t); ddy_d = diff(y_d,2,t);

% Rewrite functions to find new equations
dr_xhi = [cos(theta)*xhi_v;
          sin(theta)*xhi_v];
eta1 = ddx_d + kv*(dx_d - dr_xhi(1)) + kp*(x_d - x) + ki*xhi_x;
eta2 = ddy_d + kv*(dy_d - dr_xhi(2)) + kp*(y_d - y) + ki*xhi_y;
eta = [eta1; eta2];
xhi_dot = [[1 0]*inv(A)*eta; 
            [x_d; y_d] - [x; y]];
new_u = inv(S_c)*[xhi_v; [0 1]*inv(A)*eta];

% Partial Derivatives for gamma
h_a_1x = jacobian(new_u,a1x); h_a_1y = jacobian(new_u,a1y);
h_a_2x = jacobian(new_u,a2x); h_a_2y = jacobian(new_u,a2y);
h_a_3x = jacobian(new_u,a3x); h_a_3y = jacobian(new_u,a3y);
h_a_4x = jacobian(new_u,a4x); h_a_4y = jacobian(new_u,a4y);
g_a_1x = jacobian(xhi_dot,a1x); g_a_1y = jacobian(xhi_dot,a1y);
g_a_2x = jacobian(xhi_dot,a2x); g_a_2y = jacobian(xhi_dot,a2y);
g_a_3x = jacobian(xhi_dot,a3x); g_a_3y = jacobian(xhi_dot,a3y);
g_a_4x = jacobian(xhi_dot,a4x); g_a_4y = jacobian(xhi_dot,a4y);

%% Function creation
% Derivatives for gamma
matlabFunction(h_a_1x,'File','functions/h_a_1x','Vars',{q,t,xhi});
matlabFunction(h_a_1y,'File','functions/h_a_1y','Vars',{q,t,xhi});
matlabFunction(h_a_2x,'File','functions/h_a_2x','Vars',{q,t,xhi});
matlabFunction(h_a_2y,'File','functions/h_a_2y','Vars',{q,t,xhi});
matlabFunction(h_a_3x,'File','functions/h_a_3x','Vars',{q,t,xhi});
matlabFunction(h_a_3y,'File','functions/h_a_3y','Vars',{q,t,xhi});
matlabFunction(h_a_4x,'File','functions/h_a_4x','Vars',{q,t,xhi});
matlabFunction(h_a_4y,'File','functions/h_a_4y','Vars',{q,t,xhi});
matlabFunction(g_a_1x,'File','functions/g_a_1x','Vars',{q,t});
matlabFunction(g_a_1y,'File','functions/g_a_1y','Vars',{q,t});
matlabFunction(g_a_2x,'File','functions/g_a_2x','Vars',{q,t});
matlabFunction(g_a_2y,'File','functions/g_a_2y','Vars',{q,t});
matlabFunction(g_a_3x,'File','functions/g_a_3x','Vars',{q,t});
matlabFunction(g_a_3y,'File','functions/g_a_3y','Vars',{q,t});
matlabFunction(g_a_4x,'File','functions/g_a_4x','Vars',{q});
matlabFunction(g_a_4y,'File','functions/g_a_4y','Vars',{q});

% Derivatives for h,g that depends on parameters ai
% dh_q_ai
hq = jacobian(new_u,q);
for i= 1:4
    ai = eval(sprintf('a%dx', i));
    func = my_jacobian(hq,ai);
    matlabFunction(func,'File',sprintf('functions/dhq_a%dx', i));
end
for i= 1:4
    ai = eval(sprintf('a%dy', i));
    func = my_jacobian(hq,ai);
    matlabFunction(func,'File',sprintf('functions/dhq_a%dy', i));
end

% dh_xhi_ai
hxhi = jacobian(new_u,xhi);
for i= 1:4
    ai = eval(sprintf('a%dx', i));
    func = my_jacobian(hxhi,ai);
    matlabFunction(func,'File',sprintf('functions/dhxhi_a%dx', i));
end
for i= 1:4
    ai = eval(sprintf('a%dy', i));
    func = my_jacobian(hxhi,ai);
    matlabFunction(func,'File',sprintf('functions/dhxhi_a%dy', i));
end

% dg_q_ai
gq = jacobian(xhi_dot,q);
for i= 1:4
    ai = eval(sprintf('a%dx', i));
    func = my_jacobian(gq,ai);
    matlabFunction(func,'File',sprintf('functions/dgq_a%dx', i));
end
for i= 1:4
    ai = eval(sprintf('a%dy', i));
    func = my_jacobian(gq,ai);
    matlabFunction(func,'File',sprintf('functions/dgq_a%dy', i));
end

% dg_xhi_ai
gxhi = jacobian(xhi_dot,xhi);
for i= 1:4
    ai = eval(sprintf('a%dx', i));
    func = my_jacobian(gxhi,ai);
    matlabFunction(func,'File',sprintf('functions/dgxhi_a%dx', i));
end
for i= 1:4
    ai = eval(sprintf('a%dy', i));
    func = my_jacobian(gxhi,ai);
    matlabFunction(func,'File',sprintf('functions/dgxhi_a%dy', i));
end


function out = my_jacobian(A,x)
syms out [size(A)], [n,m] = size(A);
for i=1:n
    for j = 1:m
    out(i,j) = jacobian(A(i,j),x);
    end
end 
end
