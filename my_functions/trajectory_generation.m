function [p,dp,ddp] = trajectory_generation(p_0, v_0, p_1, p_2, p_f, v_f,tsim,time,linewidth)
% Generation of a trajectory divided into 3 polinomial pieces.
breaks = 0:tsim/3:tsim;                         

% Assign variables names
px_0 = p_0(1); py_0 = p_0(2);
vx_0 = v_0(1); vy_0 = v_0(2);
p_x1 = p_1(1); p_y1 = p_1(2);
p_x2 = p_2(1); p_y2 = p_2(2);
px_f = p_f(1); py_f = p_f(2);
vx_f = v_f(1); vy_f = v_f(2);

% The coefficients of the polynomial are found as solution of the system: Ma = d.
dx = [px_0 vx_0 0 0 p_x1 0 0 p_x2 px_f vx_f]';
dy = [py_0 vy_0 0 0 p_y1 0 0 p_y2 py_f vy_f]';

M = [0 0 0 1 0 0 0 0 0 0 0 0;
     0 0 1 0 0 0 0 0 0 0 0 0;
     (tsim/3)^3 (tsim/3)^2 tsim/3 1 0 0 0 -1 0 0 0 0;
     3*(tsim/3)^2 2*tsim/3 1 0 0 0 -1 0 0 0 0 0;
     0 0 0 0 0 0 0 1 0 0 0 0 ;
     0 0 0 0 (tsim/3)^3 (tsim/3)^2 tsim/3 1 0 0 0 -1;
     0 0 0 0 3*((tsim/3)^2) 2*(tsim/3) 1 0 0 0 -1 0;
     0 0 0 0 0 0 0 0 0 0 0 1;
     0 0 0 0 0 0 0 0 (tsim/3)^3 (tsim/3)^2 (tsim/3) 1;
     0 0 0 0 0 0 0 0 3*((tsim/3)^2) 2*(tsim/3) 1 0];

a_x = pinv(M) * dx;
a_y = pinv(M) * dy;

polyx = mkpp(breaks,[a_x(1) a_x(2) a_x(3) a_x(4);
                     a_x(5) a_x(6) a_x(7) a_x(8);
                     a_x(9) a_x(10) a_x(11) a_x(12)]);
polyy = mkpp(breaks,[a_y(1) a_y(2) a_y(3) a_y(4);
                     a_y(5) a_y(6) a_y(7) a_y(8);
                     a_y(9) a_y(10) a_y(11) a_y(12)]);

%% Plots
fontSize = 18;                 
figure(), 
fnplt(polyx,linewidth), hold on
fnplt(polyy,linewidth)
line([tsim/3 tsim/3],[0 25],'LineStyle','--','Color','k','LineWidth',1),grid minor
line([2*tsim/3 2*tsim/3],ylim,'LineStyle','--','Color','k','LineWidth',1)
xlabel('time [sec]'), ylabel('trajecory [m]')
legend('trajectory in x', 'trajectory in y')
title('Trajectory varation in time'),fontsize(fontSize,'points'),hold off

% Differentiation first order
dpolyx = fnder(polyx);
dpolyy = fnder(polyy);

% Differentiation second order
ddpolyx = fnder(dpolyx);
ddpolyy = fnder(dpolyy);

figure(),
fnplt(dpolyx,linewidth), hold on
fnplt(dpolyy,linewidth), hold off
line([tsim/3 tsim/3],ylim,'LineStyle','--','Color','k','LineWidth',1)
line([2*tsim/3 2*tsim/3],ylim,'LineStyle','--','Color','k','LineWidth',1),grid minor
xlabel("time [sec]"), ylabel('velocity [m/s]')
legend('velocity in x', 'velocity in y')
title('Velocity varation in time'),fontsize(fontSize,'points'),hold off

p = [ppval(polyx,time);ppval(polyy,time)];
dp = [ppval(dpolyx,time);ppval(dpolyy,time)];
ddp = [ppval(ddpolyx,time);ppval(ddpolyy,time)];

% Save variables for the optimization routine.
save('data/coeff_a','a_x','a_y')
save('data/desired_trajectory','p',"dp","ddp")
end