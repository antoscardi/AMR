function video(q, p, b_n, time, linewidth, delta)
% Setting up the Plot
figure(); hold on,
fontSize = 16;
colors = linspecer(4,'qualitative'); colororder(colors); 
title(sprintf('Trajectory\nTime: %0.2f sec', time(1)));
xlabel('x[m]'), ylabel('y[m]'), grid minor  
xlim([min(min(p(1,:),min(q(1,:))))-1 max(max(p(1,:),max(q(1,:))))+3])
ylim([min(min(p(2,:),min(q(2,:))))-1 max(max(p(2,:),max(q(2,:))))+3])

% Plot the entire desired trajectory at the beginning
desired = plot(p(1,:), p(2,:),'Color',colors(3,:));

% Setting variables
R = b_n/2;
x = q(1,:); y = q(2,:); theta = q(3,:);

% Plotting the first iteration
rob_trajectory = plot(q(1,1:1), q(2,1:1),'Color',colors(1,:));
orient = quiver(x(1),y(1),cos(theta(1)),sin(theta(1)),'Color',colors(2,:),'linewidth',linewidth-1);
orient.MaxHeadSize = linewidth+2;

% Use the proxy objects in the legend.
legend('desired trajectory','robot trajectory','unycicle orientation');
fontsize(fontSize, 'points'),

% Iterating through the length of the time array
tic;
for k = 2:length(time)
    % Updating the trajectories 
    rob_trajectory.XData = q(1,1:k);
    rob_trajectory.YData = q(2,1:k);
    % Updating the robot body and orientation
    robot = viscircles([x(k) y(k)],R,'Color',colors(4,:),"LineStyle","-.");
    orient.XData = x(k);
    orient.YData = y(k);
    orient.UData = cos(theta(k));
    orient.VData = sin(theta(k));

    % Updating the title
    title(sprintf('Trajectory\nTime: %0.2f sec', time(k)));

    % Delay using the real time frequency
    multiplierToMAkeupforComputations = 2.7;
    if toc > delta*multiplierToMAkeupforComputations
        drawnow;
        tic;
    end
    if k<length(time)
        delete(robot);
    end
end

end
