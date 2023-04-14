function [p,dp,ddp] = trajectory_generation(p_0, v_0, p_1, p_2, p_f, v_f,v_f_b, v_s_b,tsim,time,linewidth)
% Generation of a trajectory divided into 3 polinomial pieces.
breaks = 0:tsim/3:tsim;                         

% Assign variables names
px_0 = p_0(1); py_0 = p_0(2);
vx_0 = v_0(1); vy_0 = v_0(2);
p_x1 = p_1(1); p_y1 = p_1(2);
p_x2 = p_2(1); p_y2 = p_2(2);
px_f = p_f(1); py_f = p_f(2);
vx_f = v_f(1); vy_f = v_f(2);
v_f_b_x = v_f_b(1); v_f_b_y = v_f_b(2);
v_s_b_x = v_s_b(1); v_s_b_y = v_s_b(2);

% The coefficients of the polynomial are found as solution of the system: Ma = d.
% 
% %% This is the constraints matrix
% % p(t) = a_4*t^4 + a_3*t^3 + a_2*t^2 + a_1*t + a_0;
% % v(t) = 4a_4*t^3 + 3a_3*t^2 + 2_a_2*t + a_1
% % a(t) = 12a_3*t^2 +6a_2*t + 2a_2

M = [0 0 0 0 1 0 0 0 0 0 0 0 0 0 0;
    % this correspond to define the polinomial to the initial position -> p(0) = a_01;
     0 0 0 1 0 0 0 0 0 0 0 0 0 0 0;
     % this correspond to define the polinomial to the initial velocity -> v(0) = a_11;
     (tsim/3)^4 (tsim/3)^3 (tsim/3)^2 (tsim/3) 1 0 0 0 0 -1 0 0 0 0 0;
     % this correspond to the constraint for which the point on the end of the first polynomial is equal to the start of the second polynomial -> p1(t/3) = p2(0);
     4*(tsim/3)^3 3*(tsim/3)^2 2*(tsim/3) 1 0 0 0 0 -1 0 0 0 0 0 0;
     % this correspond to the constraint for which the velocity on the end of the first polynomial is equal to the start of the second polynomial -> v1(t/3) = v2(0);
     4*(tsim/3)^3 3*(tsim/3)^2 2*(tsim/3) 1 0 0 0 0 0 0 0 0 0 0 0;
     % this correspond to the constraint for which the velocity on the
     % first break point have to assume a determined positive value
     0 0 0 0 0 0 0 0 0 1 0 0 0 0 0;
     % This is the definition of the first break point
     0 0 0 0 0 (tsim/3)^4 (tsim/3)^3 (tsim/3)^2 (tsim/3) 1 0 0 0 0 -1;
     % this correspond to the constraint for which the point on the end of the first polynomial is equal to the start of the second polynomial -> p1(t/3) = p2(0);
     0 0 0 0 0 4*(tsim/3)^3 3*(tsim/3)^2 2*tsim/3 1 0 0 0 0 -1 0;
     % this correspond to the constraint for which the velocity on the end of the second polynomial is equal to the start of the third polynomial -> v2(t/3) = v3(0);
     0 0 0 0 0 0 0 0 0 0 0 0 0 0 1;
     % This is the definition of the second break point
     0 0 0 0 0 4*(tsim/3)^3 3*(tsim/3)^2 2*(tsim/3) 1 0 0 0 0 0 0;
     % this correspond to the constraint for which the velocity on the
     % second break point have to assume a determined positive value
     0 0 0 0 0 0 0 0 0 0 (tsim/3)^4 (tsim/3)^3 (tsim/3)^2 (tsim/3) 1;
     % this correspond to define the polinomial to the final position -> p(t/3) = a_03;
     0 0 0 0 0 0 0 0 0 0 4*(tsim/3)^3 3*(tsim/3)^2 2*(tsim/3) 1 0];
     % this correspond to define the polinomial to the final velocity -> v(t/3) = a_13;

dx = [px_0 vx_0 0 0 v_f_b_x p_x1 0 0 p_x2 v_s_b_x px_f vx_f]';
dy = [py_0 vy_0 0 0 v_f_b_y p_y1 0 0 p_y2 v_s_b_y py_f vy_f]';

a_x = pinv(M) * dx;
a_y = pinv(M) * dy;

polyx = mkpp(breaks,[a_x(1) a_x(2) a_x(3) a_x(4) a_x(5);
                     a_x(6) a_x(7) a_x(8) a_x(9) a_x(10);
                     a_x(11) a_x(12) a_x(13) a_x(14) a_x(15)]);
polyy = mkpp(breaks,[a_y(1) a_y(2) a_y(3) a_y(4) a_y(5);
                     a_y(6) a_y(7) a_y(8) a_y(9) a_y(10);
                     a_y(11) a_y(12) a_y(13) a_y(14) a_y(15)]);

%% Plots
fontSize = 16;                 
figure(1),
fnplt(polyx,linewidth), hold on
fnplt(polyy,linewidth)
xline(tsim/3 ,'LineStyle','--','Color','k','LineWidth',1.5),grid minor
xline(2*tsim/3 ,'LineStyle','--','Color','k','LineWidth',1.5)
xlabel('time [sec]'), ylabel('trajecory [m]')
legend('trajectory in x', 'trajectory in y')
title('Trajectory varation in time'),fontsize(fontSize,"points"), hold off

% Differentiation first order
dpolyx = fnder(polyx);
dpolyy = fnder(polyy);

% Differentiation second order
ddpolyx = fnder(dpolyx);
ddpolyy = fnder(dpolyy);

figure(2),
fnplt(dpolyx,linewidth), hold on
fnplt(dpolyy,linewidth)
xline(tsim/3 ,'LineStyle','--','Color','k','LineWidth',1.5),grid minor
xline(2*tsim/3 ,'LineStyle','--','Color','k','LineWidth',1.5)
xlabel("time [sec]"), ylabel('velocity [m/s]')
legend('velocity in x', 'velocity in y')
title('Velocity varation in time'),fontsize(fontSize,"points"), hold off

p = [ppval(polyx,time);ppval(polyy,time)];
dp = [ppval(dpolyx,time);ppval(dpolyy,time)];
ddp = [ppval(ddpolyx,time);ppval(ddpolyy,time)];

% Save variables for the optimization routine.
save('data/coeff_a','a_x','a_y')
save('data/desired_trajectory','p',"dp","ddp")
end