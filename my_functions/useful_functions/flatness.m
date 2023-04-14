function [v, omega] = flatness(dp,ddp)
    %   The flatness function that takes as input the velocity and acceleration of the starting point 
    %   and calculates the initial linear velocity and initial angular velocity.
    %   Take only the first values of the matrixes.
    x_dot = dp(1,1);
    y_dot = dp(2,1);
    x_dot_dot = ddp(1,1);
    y_dot_dot = ddp(2,1);
    
    % Flatness formula
    v = sqrt((x_dot)^2+(y_dot)^2);
    omega = (y_dot_dot*x_dot-x_dot_dot*y_dot)/v^2;
    end