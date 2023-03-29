function [currentInput, currentXhi] = controller(oldState,desiredPos,oldDesiredVel,oldDesiredAcc,oldXhi,delta,nominal_params,k)
%% Controller block which takes as input the current state of the robot system and outputs the new input
% commands at time step k+1 in order to control the robot.
% Inside this block the functions are ALWAYS evaluated using the NOMINAL prameters of the robot

% Dynamic Feedback Linearization Internal State
currentXhi = oldXhi + delta*xhi_dot(oldState,oldXhi,desiredPos,oldDesiredVel,oldDesiredAcc);
    
% Change control input for next step
currentInput = new_u(oldState,oldXhi,desiredPos,oldDesiredVel,oldDesiredAcc,nominal_params);

end 