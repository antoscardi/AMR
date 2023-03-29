function currentState = robot_system( oldInput, oldState, delta, params)
%% Function that given the input and current state of the robot calculates the derivative of the state
%  and integrates q_dot = f(q,u) to obtain the next State vector q [x,y,theta] at  the succesive time step k+1
currentState = oldState + delta*q_dot(oldState,oldInput,params);
end