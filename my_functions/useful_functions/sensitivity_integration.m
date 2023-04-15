function sensAtLastTimeInstant = sensitivity_integration(Nstep,nominal_params,...
                                                         q_history,xhi_history,u_history,...
                                                         p,dp,ddp,...
                                                         delta)
% In this case we are substituting parameters within the functions we created, 
% so we get the elements to create the sensitivity.
f_q = zeros(3,3);   f_p = zeros(3,2);    f_u = zeros(3,2); 
g_q = zeros(3,3);   g_xhi = zeros(3,3); 
h_xhi = zeros(2,3); h_q = zeros(2,3);

%% Computation of the Sensitivity
% We do there the integration with the use of integralSens function
% presented in my_function folder 
sens_k = zeros(12,1); sens_int= zeros(12, Nstep); 
for k=1:Nstep
    % Creation of the vectors of the partial derivaatives needed in the integration of the Sensitivity
    f_p = ff_p(q_history(:,k),u_history(:,k),nominal_params);
    f_q = ff_q(q_history(:,k),u_history(:,k),nominal_params);
    f_u = ff_u(q_history(:,k),nominal_params);
    h_q = fh_q(q_history(:,k),xhi_history(:,k),p(:,k),dp(:,k),ddp(:,k),nominal_params);
    h_xhi = fh_xhi(q_history(:,k),xhi_history(:,k),p(:,k),dp(:,k),ddp(:,k),nominal_params);
    g_q = fg_q(q_history(:,k),xhi_history(:,k),p(:,k),dp(:,k),ddp(:,k));
    g_xhi = fg_xhi(q_history(:,k),xhi_history(:,k),p(:,k),dp(:,k),ddp(:,k));

    % Integration through ode45
    [t_s, i] = ode45(@(t,i) sensitivity_dot(t, i, f_q, f_p, f_u, g_q, g_xhi, h_q, h_xhi, k), [0 delta], sens_k);
    sens_k= i(end, :)'; sens_int(:,k) = sens_k;
end

% Reshape sensitivity
%PER VELOCIZZARE RIFARE STO RESHAPE
sensAtLastTimeInstant = [sens_int(1,Nstep) sens_int(2,Nstep);
                         sens_int(3,Nstep) sens_int(4,Nstep);
                         sens_int(5,Nstep) sens_int(6,Nstep)];

end