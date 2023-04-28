function [sensAtLastTimeInstant, sensitivity_history] = sensitivity_integration(Nstep,nominal_params,...
                                                                     q_history,xhi_history,u_history,...
                                                                     p,dp,ddp,...
                                                                     kv,ki,kp, delta)
                                                                     
% In this case we are substituting parameters within the functions we created, 
% so we get the elements to create the sensitivity.
%f_q = zeros(3,3); f_p = zeros(3,2); f_u = zeros(3,2); g_q = zeros(3,3); g_xhi = zeros(3,3); h_xhi = zeros(2,3); h_q = zeros(2,3);

%% Computation of the Sensitivity
% We do there the integration with the use of sensitivity_dot function present in my_function folder 
sens_int= zeros(12, Nstep);
for k=1:Nstep-1

    % Creation of the vectors of the partial derivaatives needed in the integration of the Sensitivity
    f_p = ff_p(q_history(:,k),u_history(:,k),nominal_params);
    f_q = ff_q(q_history(:,k),u_history(:,k),nominal_params);
    f_u = ff_u(q_history(:,k),nominal_params);
    h_q = fh_q(q_history(:,k),xhi_history(:,k),p(:,k),dp(:,k),ddp(:,k),nominal_params,kv,ki,kp);
    h_xhi = fh_xhi(q_history(:,k),xhi_history(:,k),p(:,k),dp(:,k),ddp(:,k),nominal_params,kv,ki,kp);
    g_q = fg_q(q_history(:,k),xhi_history(:,k),p(:,k),dp(:,k),ddp(:,k),kv,ki,kp);
    g_xhi = fg_xhi(q_history(:,k),kv,ki,kp);

    % Integration through ode45
    sens_int(:,k+1) = sens_int(:,k) + delta*sensitivity_dot(sens_int(:,k),f_q, f_p, f_u, g_q, g_xhi, h_q, h_xhi);
end

% Reshape sensitivity
%PER VELOCIZZARE RIFARE STO RESHAPE
sensAtLastTimeInstant = [sens_int(1,Nstep) sens_int(2,Nstep);
                         sens_int(3,Nstep) sens_int(4,Nstep);
                         sens_int(5,Nstep) sens_int(6,Nstep)];

sensitivity_history = sens_int;
end