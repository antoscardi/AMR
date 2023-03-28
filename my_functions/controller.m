function [nextInput, nextXhi] = controller(nextState,p,dp,ddp,currentXhi,delta,nominal_params,k)
%% Controller block which takes as input the current state of the robot system and outputs the new input
% commands at time step k+1 in order to control the robot.
% Inside this block the functions are ALWAYS evaluated using the NOMINAL prameters of the robot

% Dynamic Feedback Linearization Internal State
nextXhi = currentXhi + delta*xhi_dot(nextState,currentXhi,p(:,k+1),dp(:,k+1),ddp(:,k+1));
    
% Change control input for next step
nextInput = new_u(nextState,nextXhi,p(:,k+1),dp(:,k+1),ddp(:,k+1),nominal_params);

end 