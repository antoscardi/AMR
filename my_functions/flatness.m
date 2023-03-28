function [v, omega] = flatness(dp,ddp)
    x_dot = dp(1,1);
    y_dot = dp(2,1);
    x_dot_dot = ddp(1,1);
    y_dot_dot = ddp(2,1);
    
    % Flatness formula
    v = sqrt((x_dot)^2+(y_dot)^2);
    omega = (y_dot_dot*x_dot-x_dot_dot*y_dot)/v^2;
    end