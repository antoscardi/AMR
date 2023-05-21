function video(q, p, b_n, time, linewidth, delta, figureName)
    % Setting up the Plot
    figure('Name', figureName,'units','normalized','outerposition',[0 0 1 1])

    fontSize = 16; Axes = axes('NextPlot', 'add');
    colors = linspecer(4,'qualitative'); colororder(colors); 
    th = title(sprintf('Trajectory\nTime: %0.2f sec', time(1)));
    xlabel('x[m]'), ylabel('y[m]'), grid minor
    
    [xMin,xMax] = bounds(p(1,:));
    [yMin,yMax] = bounds(p(2,:));
    yMin = yMin-b_n;
    yMax = yMax+b_n;
    ylim([yMin yMax+0.4]);
    xlim([xMin xMax+0.4]);
    
    % Plot the entire desired trajectory at the beginning
    plot(p(1,:), p(2,:),'Color',colors(3,:),'Parent',Axes);
    
    % Setting variables
    R = b_n/2;
    x = q(1,:); y = q(2,:); theta = q(3,:);
    
    % Plotting the first iteration
    rob_trajectory = plot(q(1,1:1), q(2,1:1),'Color',colors(1,:),'Parent',Axes);
    orient = quiver(x(1),y(1),cos(theta(1)),sin(theta(1)),0.5,'Color',colors(2,:),'linewidth',linewidth-1,'Parent',Axes);
    orient.MaxHeadSize = linewidth+1;
    
    legend('desired trajectory','robot trajectory','unycicle orientation','Location','northwest');
    fontsize(fontSize, 'points'),
    axis equal
    
    % Iterating through the length of the time array
    tic;
    for k = 2:length(time)
        % Updating the trajectories 
        rob_trajectory.XData = q(1,1:k);
        rob_trajectory.YData = q(2,1:k);
        % Updating the robot body and orientation
        robot = viscircles(Axes,[x(k) y(k)],R,'Color',colors(4,:),"LineStyle","-.");
        orient.XData = x(k);
        orient.YData = y(k);
        orient.UData = cos(theta(k));
        orient.VData = sin(theta(k));
    
        % Updating the title
        %title(sprintf('Trajectory\nTime: %0.2f sec', time(k)));
        set(th, 'String', sprintf('Trajectory\nTime: %0.2f sec', time(k)));
    
        % Delay using the real time frequency
        multiplierToMAkeupforComputations = 2.7*5;
        if toc > delta*multiplierToMAkeupforComputations
            drawnow();
            tic;
        end
        if k<length(time)
            delete(robot);
        end
    end
    
    end