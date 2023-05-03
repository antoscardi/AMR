function [pos,vel,acc,theta_d] = trajectory_generation(coeffMatrix, timeVec, totalTime,...
                                               linewidth, colors, doPlots)
% Function that calculates the trajectory given a set of parameters.
a_x = coeffMatrix(:,1);
a_y = coeffMatrix(:,2);

% Set break points 
breaks = 0:totalTime/3:totalTime;

polyx = mkpp(breaks,[a_x(1) a_x(2) a_x(3) a_x(4) a_x(5);
                     a_x(6) a_x(7) a_x(8) a_x(9) a_x(10);
                     a_x(11) a_x(12) a_x(13) a_x(14) a_x(15)]);
polyy = mkpp(breaks,[a_y(1) a_y(2) a_y(3) a_y(4) a_y(5);
                     a_y(6) a_y(7) a_y(8) a_y(9) a_y(10);
                     a_y(11) a_y(12) a_y(13) a_y(14) a_y(15)]);

% Differentiation first order
dpolyx = fnder(polyx);
dpolyy = fnder(polyy);

% Differentiation second order
ddpolyx = fnder(dpolyx);
ddpolyy = fnder(dpolyy);

% Evaluate the values on the timeVector
pos = [ppval(polyx,timeVec); ppval(polyy,timeVec)];
vel = [ppval(dpolyx,timeVec); ppval(dpolyy,timeVec)];
acc = [ppval(ddpolyx,timeVec); ppval(ddpolyy,timeVec)];

% Calculate the desired theta, orientation of the robot on the trajectory
len = length(vel);
theta_d = zeros(len,1);
for k = 1:len
    theta_d(k) = atan2(vel(2,k),vel(1,k));
end 

if doPlots == true
    
%% Plots
fontSize = 16;                 
figure(1),
fnplt(polyx,linewidth), hold on
fnplt(polyy,linewidth)
xline(breaks(2) ,'LineStyle','--','Color','k','LineWidth',1.5),grid minor
xline(breaks(3) ,'LineStyle','--','Color','k','LineWidth',1.5)
yline(0,'LineStyle','--','Color','k','LineWidth',1.5)
xlabel('time [sec]'), ylabel('trajecory [m]')
legend('trajectory in x', 'trajectory in y')
title('Trajectory varation in time'),fontsize(fontSize,"points"), hold off

figure(2),
fnplt(dpolyx,linewidth), hold on
fnplt(dpolyy,linewidth)
xline(breaks(2) ,'LineStyle','--','Color','k','LineWidth',1.5),grid minor
xline(breaks(3),'LineStyle','--','Color','k','LineWidth',1.5)
yline(0,'LineStyle','--','Color','k','LineWidth',1.5)
xlabel("time [sec]"), ylabel('velocity [m/s]')
legend('velocity in x', 'velocity in y')
title('Velocity varation in time'),fontsize(fontSize,"points"), hold off

figure(3),
plot(ppval(polyx,timeVec),ppval(polyy,timeVec),'Color',colors(3,:), 'LineWidth',linewidth)
yline(0,'LineStyle','--','Color','k','LineWidth',1.5)
xlabel("x[m]"), ylabel('y[m]'), grid minor
title('Trajectory'),fontsize(fontSize,"points")

res=zeros(1,2501);
for i=1:2501
    res(:,i)= sqrt_of_quadratics(vel(:,i));
end
figure(4),
plot(timeVec, res','Color',colors(4,:), 'LineWidth',linewidth)
%plot(ppval(dpolyx,timeVec),ppval(dpolyy,timeVec),'Color',colors(4,:), 'LineWidth',linewidth)
xlabel("xdot[m/s]"), ylabel('ydot[m/s]'), grid minor
title('Velocity of the Trajectory in time'),fontsize(fontSize,"points")

figure(5),
plot(ppval(dpolyx,timeVec),ppval(dpolyy,timeVec),'Color',colors(4,:), 'LineWidth',linewidth)
xlabel("xdot[m/s]"), ylabel('ydot[m/s]'), grid minor
title('Velocity of the Trajectory'),fontsize(fontSize,"points")
end 

end 