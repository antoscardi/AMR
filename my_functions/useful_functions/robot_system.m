function currentState = robot_system( oldInput, oldState, delta, params, kv, ki, kp)
    %% Function that given the input and current state of the robot calculates the derivative of the state
    %  and integrates q_dot = f(q,u) to obtain the next State vector q [x,y,theta] at  the succesive time step k+1

    [~, qint] = ode45(@(t,q) q_dot(q,oldInput,params,kv,ki,kp),[0 delta],oldState);
    currentState = qint(end,:)';
    
    end