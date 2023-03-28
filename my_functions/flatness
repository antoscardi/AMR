function [v_in, omega_in] = flatness(dp,ddp)
% Flatness formula
v_in = sqrt((dp(1,1))^2+(dp(2,1))^2);
omega_in = (ddp(2,1)*dp(1,1)-ddp(1,1)*dp(2,1))/v_in^2;
end