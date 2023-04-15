function sensitivity_ai_integration()
%% Tensor product calculations, one for each parameter
% initialize 3D matrixes, which are overwritten 
% how to read: 
% - dfqqgamma--> tensor product between the derivate of f_q respect to q and gamma
% - dfpqgamma--> tensor product between the derivate of f_p respect to q wand gamma
% - dhxhiqgamma--> tensor product between the derivate of h_xhi respect to q with gamma
% ...
curr_dfqqgamma = zeros(3,3,Nstep); curr_dfquuai = zeros(3,3,Nstep); 
curr_dfpqgamma = zeros(3,2,Nstep); curr_dfpuuai = zeros(3,2,Nstep); 
curr_dfuqgamma = zeros(3,2,Nstep); curr_dfuuuai = zeros(3,2,Nstep); 
curr_dhqqgamma = zeros(2,3,Nstep); curr_dhqxhigammaxhi = zeros(2,3,Nstep); 
curr_dhxhiqgamma = zeros(2,3,Nstep); curr_dhxhixhigammaxhi = zeros(2,3,Nstep); 
curr_dgqqgamma = zeros(3,3,Nstep); curr_dgqxhigammaxhi = zeros(3,3,Nstep); 
curr_dgxhiqgamma = zeros(3,3,Nstep); curr_dgxhixhigammaxhi = zeros(3,3,Nstep);

% Initialize cell array to store all x double derivatives, one for each x
allx_dfqqgamma =cell(1,12); allx_dfquuai = cell(1,12); 
allx_dfpqgamma =cell(1,12); allx_dfpuuai = cell(1,12); 
allx_dfuqgamma = cell(1,12); allx_dfuuuai = cell(1,12); 
allx_dhqqgamma = cell(1,12); allx_dhqxhigammaxhi = cell(1,12); 
allx_dhxhiqgamma = cell(1,12); allx_dhxhixhigammaxhi = cell(1,12); 
allx_dgqqgamma = cell(1,12); allx_dgqxhigammaxhi = cell(1,12); 
allx_dgxhiqgamma = cell(1,12); allx_dgxhixhigammaxhi = cell(1,12);

% We compute this for all the 12 trajectory coefficients x and for the 12
% trajectory coefficients y
% X
for i = 1:12
    % Get name of each gamma integrated variable
    gamma_name = sprintf('gammax%d_int', i); uai_name = sprintf('u_ax%d', i);
    % Get variable
    var = eval(gamma_name); var2 = eval(uai_name);
    % Loop over time steps
    for k = 1:Nstep
        % For the computation we use the Matlab Function created in the
        % file b
        gamma = var(1:3,k); gammaxhi = var(4:6,k); u_ai = var2(:,:,k);
        curr_dfqqgamma(:,:,k) = df_q_q_gamma(gamma, params(1:2),u_history(:,k),q_history(:,k));
        curr_dfquuai(:,:,k) = df_q_u_uai(q_history(:,k),u_ai,params(1:2));
        curr_dfpqgamma(:,:,k) = df_p_q_gamma(gamma, u_history(:,k), q_history(:,k));
        curr_dfpuuai(:,:,k) = df_p_u_uai(u_ai, q_history(:,k), params(1:2));
        curr_dfuqgamma(:,:,k) = df_u_q_gamma(gamma, params(1:2), q_history(:,k));
        curr_dfuuuai(:,:,k) = df_u_u_uai();
        curr_dhqqgamma(:,:,k) = dh_q_q_gamma(gamma,q_history(:,k),xhi_history(:,k),desired_traj(:,k),dp(:,k),ddp(:,k), params(1:2));
        curr_dhqxhigammaxhi(:,:,k) =dh_q_xhi_gammaxhi(gammaxhi,q_history(:,k),xhi_history(:,k),desired_traj(:,k),dp(:,k),ddp(:,k), params(1:2));
        curr_dhxhiqgamma(:,:,k) =dh_xhi_q_gamma(gamma,q_history(:,k),xhi_history(:,k),desired_traj(:,k),dp(:,k),ddp(:,k), params(1:2));
        curr_dhxhixhigammaxhi(:,:,k) = dh_xhi_xhi_gammaxhi(gammaxhi,q_history(:,k),xhi_history(:,k),desired_traj(:,k),dp(:,k),ddp(:,k), params(1:2));
        curr_dgqqgamma(:,:,k) = dg_q_q_gamma(gamma,q_history(:,k),xhi_history(:,k),desired_traj(:,k),dp(:,k),ddp(:,k));
        curr_dgqxhigammaxhi(:, :, k) = dg_q_xhi_gammaxhi(gammaxhi,q_history(:,k));
        curr_dgxhiqgamma(:,:,k) = dg_xhi_q_gamma(gamma,q_history(:,k));
        curr_dgxhixhigammaxhi(:,:,k) = dg_xhi_xhi_gammaxhi(); 
    end
    % Store 3D matrix in cell array
    allx_dfqqgamma{i} = curr_dfqqgamma; allx_dfquuai{i} =  curr_dfquuai; 
    allx_dfpqgamma{i} = curr_dfpqgamma; allx_dfpuuai{i} =  curr_dfpuuai;
    allx_dfuqgamma{i} = curr_dfuqgamma; allx_dfuuuai{i} =  curr_dfuuuai; 
    allx_dhqqgamma{i} =  curr_dhqqgamma; allx_dhqxhigammaxhi{i} =  curr_dhqxhigammaxhi;
    allx_dhxhiqgamma{i} =  curr_dhxhiqgamma; allx_dhxhixhigammaxhi{i} =  curr_dhxhixhigammaxhi; 
    allx_dgqqgamma{i} =  curr_dgqqgamma; allx_dgqxhigammaxhi{i} = curr_dgqxhigammaxhi;
    allx_dgxhiqgamma{i} =  curr_dgxhiqgamma; allx_dgxhixhigammaxhi{i} = curr_dgxhixhigammaxhi;
end

% Initialize cell array to store all y double derivatives, one for each y
ally_dfqqgamma =cell(1,12);  ally_dfquuai = cell(1,12); 
ally_dfpqgamma =cell(1,12);  ally_dfpuuai = cell(1,12);  
ally_dfuqgamma = cell(1,12);  ally_dfuuuai = cell(1,12); 
ally_dhqqgamma = cell(1,12); ally_dhqxhigammaxhi = cell(1,12); 
ally_dhxhiqgamma = cell(1,12); ally_dhxhixhigammaxhi = cell(1,12); 
ally_dgqqgamma = cell(1,12);  ally_dgqxhigammaxhi = cell(1,12); 
ally_dgxhiqgamma = cell(1,12); ally_dgxhixhigammaxhi = cell(1,12);
% Y
for i = 1:12
    % Get name of each gamma integrated variable
    gamma_name = sprintf('gammay%d_int', i); uai_name = sprintf('u_ay%d', i);
    % Get variable
    var = eval(gamma_name); var2 = eval(uai_name);
    % Loop over time steps
    for k = 1:Nstep
        % For the computation we use the Matlab Function created in the
        % file b
        gamma = var(1:3,k); gammaxhi = var(4:6,k); u_ai = var2(:,:,k);
        curr_dfqqgamma(:,:,k) = df_q_q_gamma(gamma, params(1:2),u_history(:,k),q_history(:,k));
        curr_dfquuai(:,:,k) = df_q_u_uai(q_history(:,k),u_ai,params(1:2));
        curr_dfpqgamma(:,:,k) = df_p_q_gamma(gamma, u_history(:,k), q_history(:,k));
        curr_dfpuuai(:,:,k) = df_p_u_uai(u_ai, q_history(:,k), params(1:2));
        curr_dfuqgamma(:,:,k) = df_u_q_gamma(gamma, params(1:2), q_history(:,k));
        curr_dfuuuai(:,:,k) = df_u_u_uai();
        curr_dhqqgamma(:,:,k) = dh_q_q_gamma(gamma,q_history(:,k),xhi_history(:,k),desired_traj(:,k),dp(:,k),ddp(:,k), params(1:2));
        curr_dhqxhigammaxhi(:,:,k) =dh_q_xhi_gammaxhi(gammaxhi,q_history(:,k),xhi_history(:,k),desired_traj(:,k),dp(:,k),ddp(:,k), params(1:2));
        curr_dhxhiqgamma(:,:,k) =dh_xhi_q_gamma(gamma,q_history(:,k),xhi_history(:,k),desired_traj(:,k),dp(:,k),ddp(:,k), params(1:2));
        curr_dhxhixhigammaxhi(:,:,k) = dh_xhi_xhi_gammaxhi(gammaxhi,q_history(:,k),xhi_history(:,k),desired_traj(:,k),dp(:,k),ddp(:,k), params(1:2));
        curr_dgqqgamma(:,:,k) = dg_q_q_gamma(gamma,q_history(:,k),xhi_history(:,k),desired_traj(:,k),dp(:,k),ddp(:,k));
        curr_dgqxhigammaxhi(:, :, k) = dg_q_xhi_gammaxhi(gammaxhi,q_history(:,k));
        curr_dgxhiqgamma(:,:,k) = dg_xhi_q_gamma(gamma,q_history(:,k));
        curr_dgxhixhigammaxhi(:,:,k) = dg_xhi_xhi_gammaxhi(); 
    end
    % Store 3D matrix in cell array
    ally_dfqqgamma{i} = curr_dfqqgamma; ally_dfquuai{i} =  curr_dfquuai; 
    ally_dfpqgamma{i} = curr_dfpqgamma; ally_dfpuuai{i} =  curr_dfpuuai;
    ally_dfuqgamma{i} = curr_dfuqgamma; ally_dfuuuai{i} =  curr_dfuuuai; 
    ally_dhqqgamma{i} =  curr_dhqqgamma; ally_dhqxhigammaxhi{i} =  curr_dhqxhigammaxhi;
    ally_dhxhiqgamma{i} =  curr_dhxhiqgamma; ally_dhxhixhigammaxhi{i} =  curr_dhxhixhigammaxhi; 
    ally_dgqqgamma{i} =  curr_dgqqgamma; ally_dgqxhigammaxhi{i} = curr_dgqxhigammaxhi;
    ally_dgxhiqgamma{i} =  curr_dgxhiqgamma; ally_dgxhixhigammaxhi{i} = curr_dgxhixhigammaxhi;
end

%% Creation of derivate of h_q, h_xhi, g_q, g_xhi respect to coefficents [a1_x, a1_y, a2_x, ...]
% These are necessary to make the integration of Sensitivity_ai

% Double derivatives respect to the parameters 
dhqa1x = zeros(2,3,Nstep); dhqa1y = zeros(2,3,Nstep); dhqa2x = zeros(2,3,Nstep); dhqa2y = zeros(2,3,Nstep); dhqa3x = zeros(2,3,Nstep); dhqa3y = zeros(2,3,Nstep); dhqa4x = zeros(2,3,Nstep); dhqa4y = zeros(2,3,Nstep); dhqa5x = zeros(2,3,Nstep); dhqa5y = zeros(2,3,Nstep);
dhxhia1x = zeros(2,3,Nstep); dhxhia1y = zeros(2,3,Nstep); dhxhia2x = zeros(2,3,Nstep); dhxhia2y = zeros(2,3,Nstep); dhxhia3x = zeros(2,3,Nstep); dhxhia3y = zeros(2,3,Nstep); dhxhia4x = zeros(2,3,Nstep); dhxhia4y = zeros(2,3,Nstep); dhxhia5x = zeros(2,3,Nstep); dhxhia5y = zeros(2,3,Nstep); 
dgqa1x = zeros(3,3,Nstep); dgqa1y = zeros(3,3,Nstep); dgqa2x = zeros(3,3,Nstep); dgqa2y = zeros(3,3,Nstep); dgqa3x = zeros(3,3,Nstep); dgqa3y = zeros(3,3,Nstep); dgqa4x = zeros(3,3,Nstep); dgqa4y = zeros(3,3,Nstep); dgqa5x = zeros(3,3,Nstep); dgqa5y = zeros(3,3,Nstep); 
dgxhia1x = zeros(3,3,Nstep); dgxhia1y = zeros(3,3,Nstep); dgxhia2x = zeros(3,3,Nstep); dgxhia2y = zeros(3,3,Nstep); dgxhia3x = zeros(3,3,Nstep); dgxhia3y = zeros(3,3,Nstep); dgxhia4x = zeros(3,3,Nstep); dgxhia4y = zeros(3,3,Nstep); dgxhia5x = zeros(3,3,Nstep); dgxhia5y = zeros(3,3,Nstep);
for k=1:Nstep
    dhqa1x(:,:,k) = dhq_a1x(params(2), params(1), timeVec(k), q_history(3,k), xhi_history(1,k));
    dhqa2x(:,:,k) = dhq_a2x(params(2), params(1), timeVec(k), q_history(3,k), xhi_history(1,k));
    dhqa3x(:,:,k) = dhq_a3x(params(2), params(1), timeVec(k), q_history(3,k), xhi_history(1,k));
    dhqa4x(:,:,k) = dhq_a4x(params(2), params(1), timeVec(k), q_history(3,k), xhi_history(1,k));
    dhqa5x(:,:,k) = dhq_a5x(params(2), params(1), q_history(3,k), xhi_history(1,k));
    
    dhqa1y(:,:,k) = dhq_a1y(params(2), params(1), timeVec(k), q_history(3,k), xhi_history(1,k));
    dhqa2y(:,:,k) = dhq_a2y(params(2), params(1), timeVec(k), q_history(3,k), xhi_history(1,k));
    dhqa3y(:,:,k) = dhq_a3y(params(2), params(1), timeVec(k), q_history(3,k), xhi_history(1,k));
    dhqa4y(:,:,k) = dhq_a4y(params(2), params(1), timeVec(k), q_history(3,k), xhi_history(1,k));
    dhqa5y(:,:,k) = dhq_a5y(params(2), params(1), q_history(3,k), xhi_history(1,k));

    dhxhia1x(:,:,k) = dhxhi_a1x(params(2), params(1), timeVec(k), q_history(3,k), xhi_history(1,k));
    dhxhia2x(:,:,k) = dhxhi_a2x(params(2), params(1), timeVec(k), q_history(3,k), xhi_history(1,k));
    dhxhia3x(:,:,k) = dhxhi_a3x(params(2), params(1), timeVec(k), q_history(3,k), xhi_history(1,k));
    dhxhia4x(:,:,k) = dhxhi_a4x(params(2), params(1), timeVec(k), q_history(3,k), xhi_history(1,k));
    dhxhia5x(:,:,k) = dhxhi_a5x(params(2), params(1), q_history(3,k), xhi_history(1,k));

    dhxhia1y(:,:,k) = dhxhi_a1y(params(2), params(1), timeVec(k), q_history(3,k), xhi_history(1,k));
    dhxhia2y(:,:,k) = dhxhi_a2y(params(2), params(1), timeVec(k), q_history(3,k), xhi_history(1,k));
    dhxhia3y(:,:,k) = dhxhi_a3y(params(2), params(1), timeVec(k), q_history(3,k), xhi_history(1,k));
    dhxhia4y(:,:,k) = dhxhi_a4y(params(2), params(1), timeVec(k), q_history(3,k), xhi_history(1,k));
    dhxhia5y(:,:,k) = dhxhi_a5y(params(2), params(1), q_history(3,k), xhi_history(1,k));

    dgqa1x(:,:,k) = dgq_a1x(timeVec(k), q_history(3,k));
    dgqa2x(:,:,k) = dgq_a2x(timeVec(k), q_history(3,k));
    dgqa3x(:,:,k) = dgq_a3x(timeVec(k), q_history(3,k));
    dgqa4x(:,:,k) = dgq_a4x(timeVec(k), q_history(3,k));
    dgqa5x(:,:,k) = dgq_a5x(q_history(3,k));
    dgqa1y(:,:,k) = dgq_a1y(timeVec(k), q_history(3,k));
    dgqa2y(:,:,k) = dgq_a2y(timeVec(k), q_history(3,k));
    dgqa3y(:,:,k) = dgq_a3y(timeVec(k), q_history(3,k));
    dgqa4y(:,:,k) = dgq_a4y(timeVec(k), q_history(3,k));
    dgqa5y(:,:,k) = dgq_a5y(q_history(3,k));

    dgxhia1x(:,:,k)=dgxhi_a1x();
    dgxhia2x(:,:,k)=dgxhi_a2x();
    dgxhia3x(:,:,k)=dgxhi_a3x();
    dgxhia4x(:,:,k)=dgxhi_a4x();
    dgxhia5x(:,:,k)=dgxhi_a5x();
    dgxhia1y(:,:,k)=dgxhi_a1y();
    dgxhia2y(:,:,k)=dgxhi_a2y();
    dgxhia3y(:,:,k)=dgxhi_a3y();
    dgxhia4y(:,:,k)=dgxhi_a4y();
    dgxhia5y(:,:,k)=dgxhi_a5y();
 end

 % We use now all these matrix to computed the Sensitivity_ai 

%% FINAL SENSITIVITY_Ai CALCULATION THROUGHT INTEGRATION OF ODEs
% For the x parameters
all_sensaix = cell(1,12);
% Repeat the calculations 3 times, one for each trajectory
z=1;
for i=1:12
if z==5
    z=1;
end
sens_k = zeros(12,1); sensai_tot = zeros(12,1,Nstep); 
dgq = eval(sprintf('dgqa%dx', z)); dgxhi = eval(sprintf('dgxhia%dx', z)); dhq = eval(sprintf('dhqa%dx', z)); dhxhi = eval(sprintf('dhxhia%dx', z));
for k=1:Nstep
[~, j] = ode45(@(t,j) integralSens_ai(t,j, allx_dfqqgamma{i}(:,:,k),allx_dfquuai{i}(:,:,k),sens_int(1:6,k),...
         f_q(:,:,k), allx_dfpqgamma{i}(:,:,k), allx_dfpuuai{i}(:,:,k),allx_dfuqgamma{i}(:,:,k),allx_dfuuuai{i}(:,:,k),...
         h_q(:,:,k), h_xhi(:,:,k),sens_int(7:12,k), f_u(:,:,k), allx_dhqqgamma{i}(:,:,k),allx_dhqxhigammaxhi{i}(:,:,k),...
         dhq(:,:,k),allx_dhxhiqgamma{i}(:,:,k), allx_dhxhixhigammaxhi{i}(:,:,k),dhxhi(:,:,k),allx_dgqqgamma{i}(:,:,k),allx_dgqxhigammaxhi{i}(:,:,k),...
         dgq(:,:,k),allx_dgxhiqgamma{i}(:,:,k),allx_dgxhixhigammaxhi{i}(:,:,k),dgxhi(:,:,k), g_q(:,:,k), g_xhi(:,:,k)), [0 delta],sens_k);
sens_k = j(end, :)'; sensai_tot(:,:,k) = sens_k;
end
z=z+1;
all_sensaix{i} = sensai_tot;
end

% For the y parameters
all_sensaiy = cell(1,12);
% Repeat the calculations 3 times, one for each trajectory
z=1;
for i=1:12
if z==5
    z=1;
end
sens_k = zeros(12,1); sensai_tot = zeros(12,1,Nstep); 
dgq = eval(sprintf('dgqa%dy', z)); dgxhi = eval(sprintf('dgxhia%dy', z)); dhq = eval(sprintf('dhqa%dy', z)); dhxhi = eval(sprintf('dhxhia%dy', z));
for k=1:Nstep
[~, j] = ode45(@(t,j) integralSens_ai(t,j, ally_dfqqgamma{i}(:,:,k),ally_dfquuai{i}(:,:,k),sens_int(1:6,k),...
         f_q(:,:,k), ally_dfpqgamma{i}(:,:,k), ally_dfpuuai{i}(:,:,k),ally_dfuqgamma{i}(:,:,k),ally_dfuuuai{i}(:,:,k),...
         h_q(:,:,k), h_xhi(:,:,k),sens_int(7:12,k), f_u(:,:,k), ally_dhqqgamma{i}(:,:,k),ally_dhqxhigammaxhi{i}(:,:,k),...
         dhq(:,:,k),ally_dhxhiqgamma{i}(:,:,k), ally_dhxhixhigammaxhi{i}(:,:,k),dhxhi(:,:,k),ally_dgqqgamma{i}(:,:,k),ally_dgqxhigammaxhi{i}(:,:,k),...
         dgq(:,:,k),ally_dgxhiqgamma{i}(:,:,k),ally_dgxhixhigammaxhi{i}(:,:,k),dgxhi(:,:,k), g_q(:,:,k), g_xhi(:,:,k)), [0 delta],sens_k);
sens_k = j(end, :)'; sensai_tot(:,:,k) = sens_k;
end
z=z+1;
all_sensaiy{i} = sensai_tot;
end



end