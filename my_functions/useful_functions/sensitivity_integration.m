function sensAtLastTimeInstant = sensitivity_integration(Nstep,nominal_params,...
                                                         q_history,xhi_history,u_history,...
                                                         p,dp,ddp,...
                                                         delta)
% In this case we are substituting parameters within the functions we created, 
% so we get the elements to create the sensitivity.
f_q = zeros(3,3,Nstep); f_p = zeros(3,2,Nstep); f_u = zeros(3,2,Nstep); 
g_q = zeros(3,3,Nstep); g_xhi = zeros(3,3,Nstep); 
h_xhi = zeros(2,3,Nstep); h_q = zeros(2,3,Nstep);
h_ax1 = zeros(2,1,Nstep); h_ay1 = zeros(2,1,Nstep); 
h_ax2 = zeros(2,1,Nstep); h_ay2= zeros(2,1,Nstep); 
h_ax3 = zeros(2,1,Nstep); h_ay3 = zeros(2,1,Nstep); 
h_ax4 = zeros(2,1,Nstep); h_ay4 = zeros(2,1,Nstep); 
h_ax5 = zeros(2,1,Nstep); h_ay5 = zeros(2,1,Nstep);
g_ax1= zeros(3,1,Nstep); g_ay1 = zeros(3,1,Nstep); 
g_ax2 = zeros(3,1,Nstep); g_ay2 = zeros(3,1,Nstep); 
g_ax3 = zeros(3,1,Nstep); g_ay3 = zeros(3,1,Nstep); 
g_ax4 = zeros(3,1,Nstep); g_ay4 = zeros(3,1,Nstep); 
g_ax5 = zeros(3,1,Nstep); g_ay5 = zeros(3,1,Nstep);

%%% QUESTO PERCHé NON FARLO DIRETTO DENTRO IL CICLO DI INTEGRAZIONE PASSANDOGLI A INTEGRAL SENS IL VALORE CORRETTO SUBITO
%%%% COS^ SI EVITA UN CICLO IN PIU
for k = 1:Nstep
% Creation of the matrix needed in the integration of the Sensitivity
f_p(:,:,k) = ff_p(q_history(:,k),u_history(:,k),nominal_params);
f_q(:,:,k) = ff_q(q_history(:,k),u_history(:,k),nominal_params);
f_u(:,:,k) = ff_u(q_history(:,k),nominal_params);
h_q(:,:,k) = fh_q(q_history(:,k),xhi_history(:,k),p(:,k),dp(:,k),ddp(:,k),nominal_params);
h_xhi(:,:,k) = fh_xhi(q_history(:,k),xhi_history(:,k),p(:,k),dp(:,k),ddp(:,k),nominal_params);
g_q(:,:,k) = fg_q(q_history(:,k),xhi_history(:,k),p(:,k),dp(:,k),ddp(:,k));
g_xhi(:,:,k) = fg_xhi(q_history(:,k),xhi_history(:,k),p(:,k),dp(:,k),ddp(:,k));
end

%% Computation of the Sensitivity
% We do there the integration with the use of integralSens function
% presented in my_function folder 
sens_k = zeros(12,1); sens_int= zeros(12, Nstep); 
for k=1:Nstep
    [t_s, i] = ode45(@(t,i) sensitivity_dot(t, i, f_q, f_p, f_u, g_q, g_xhi, h_q, h_xhi, k), [0 delta], sens_k);
    sens_k= i(end, :)'; sens_int(:,k) = sens_k;
end

% Reshape sensitivity
%PER VELOCIZZARE RIFARE STO RESHAPE
sensAtLastTimeInstant = [sens_int(1,Nstep) sens_int(2,Nstep);
                         sens_int(3,Nstep) sens_int(4,Nstep);
                        sens_int(5,Nstep) sens_int(6,Nstep)];

end