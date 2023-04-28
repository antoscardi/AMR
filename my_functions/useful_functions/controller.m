function [currentInput, currentXhi] = controller(oldState,desiredPos,oldDesiredVel,oldDesiredAcc,oldXhi,delta,nominal_params,kv,ki,kp)
    %% Controller block which takes as input the current state of the robot system and outputs the new input
    % commands at time step k+1 in order to control the robot.
    % Inside this block the functions are ALWAYS evaluated using the NOMINAL prameters of the robot
    
    % Dynamic Feedback Linearization Internal State
    [~, xhiint] = ode45(@(t,xhi) xhi_dot(oldState,xhi,desiredPos,oldDesiredVel,oldDesiredAcc,kv,ki,kp),[0 delta],oldXhi);
    currentXhi = xhiint(end,:)';
        
    % Change control input for next step
    currentInput = new_u(oldState,oldXhi,desiredPos,oldDesiredVel,oldDesiredAcc,nominal_params,kv,ki,kp);
    
    end 