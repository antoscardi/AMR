function video(q_history,p,param,time)
% Setting up the Plot
figure; hold on, colors = linspecer(3,'qualitative'); colororder(colors),
title(sprintf('Trajectory\nTime: %0.2f sec', time(1)));
xlabel('x[m]'), ylabel('y[m]'), grid minor  
xlim([min(min(p(1,:),min(q_history(1,:))))-1 max(max(p(1,:),max(q_history(1,:))))+3])
ylim([min(min(p(2,:),min(q_history(2,:))))-1 max(max(p(2,:),max(q_history(2,:))))+3])

% Plotting the first iteration
R0 = param(1)/2;
p_r = q_history(1:2,:); theta = q_history(3,:);
trajectory = plot(q_history(1,1:1), q_history(2,1:1));
plot(p(1,:), p(2,:));
orient = line([p_r(1,1),p_r(1,1)+R0*cos(theta(1))],[p_r(2,1), p_r(2,1)+R0*sin(theta(1))],'Color',colors(3,:),"LineStyle","-."); orient.MarkerSize = 6;
legend('unicycle traj','desired traj','robot');

% Iterating through the length of the time array
for k = 2:length(time)
    % Updating the trajectories 
    trajectory.XData = q_history(1,1:k);
    trajectory.YData = q_history(2,1:k);
    % Updating the robot body and orientation
    R = param(k)/2; % radius is the distance between the wheels b divided by 2
    robot = viscircles([p_r(1,k) p_r(2,k)],R,'Color',colors(3,:),"LineStyle","-.");
    orient.XData = [p_r(1,k),p_r(1,k)+R*cos(theta(k))];
    orient.YData = [p_r(2,k), p_r(2,k)+R*sin(theta(k))];
    % Updating the title
    title(sprintf('Trajectory\nTime: %0.2f sec', time(k)));
    % Delay
    drawnow
    if k<length(time)
    delete(robot);
    end
end
end
