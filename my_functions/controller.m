function


% Dynamic Feedback Linearization Internal State
    [~, xhi_int] = ode45(@(t,xhi) xhi_dot(t,q_k,xhi,p(:,k),dp(:,k),ddp(:,k)),[0 delta],xhi_k);
    xhi_k = xhi_int(end,:)';
    
    % Change control input for next step
    u_k = new_u(q_history(:,k-1),xhi_history(:,k-1),p(:,k),dp(:,k),ddp(:,k));