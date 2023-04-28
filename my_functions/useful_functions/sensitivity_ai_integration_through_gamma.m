function allSensitivitiesInOneArray = sensitivity_ai_integration_through_gamma(sens_int,CoeffMatrix,...
                                                  params,timeVec,...
                                                  q_history,xhi_history,u_history,...
                                                  desired_traj,dp,ddp,...
                                                  kv,ki,kp,delta,Nstep)
% Get the number of coefficients
[Ncoeff,ncolumns] = size(CoeffMatrix);

% Derivatives calculation for Gamma
% h Initializations
h_a11 = zeros(2,1,Nstep);  h_a12 = zeros(2,1,Nstep); 
h_a21 = zeros(2,1,Nstep);  h_a22= zeros(2,1,Nstep); 
h_a31 = zeros(2,1,Nstep);  h_a32 = zeros(2,1,Nstep); 
h_a41 = zeros(2,1,Nstep);  h_a42 = zeros(2,1,Nstep); 
h_a51 = zeros(2,1,Nstep);  h_a52 = zeros(2,1,Nstep);
% g Initializations
g_a11= zeros(3,1,Nstep);   g_a12 = zeros(3,1,Nstep); 
g_a21 = zeros(3,1,Nstep);  g_a22 = zeros(3,1,Nstep); 
g_a31 = zeros(3,1,Nstep);  g_a32 = zeros(3,1,Nstep); 
g_a41 = zeros(3,1,Nstep);  g_a42 = zeros(3,1,Nstep); 
g_a51 = zeros(3,1,Nstep);  g_a52 = zeros(3,1,Nstep);

for k = 1:Nstep
% Creation of the matrix needed in the integration of the Gamma
h_a11(:,:,k) = h_a_1x(q_history(:,k),timeVec(k),xhi_history(:,k), params,kv,kp);
h_a12(:,:,k) = h_a_1y(q_history(:,k),timeVec(k),xhi_history(:,k), params, kv,kp);
h_a21(:,:,k) = h_a_2x(q_history(:,k),timeVec(k),xhi_history(:,k), params,kv,kp);
h_a22(:,:,k) = h_a_2y(q_history(:,k),timeVec(k),xhi_history(:,k), params,kv,kp);
h_a31(:,:,k) = h_a_3x(q_history(:,k),timeVec(k),xhi_history(:,k), params,kv,kp);
h_a32(:,:,k) = h_a_3y(q_history(:,k),timeVec(k),xhi_history(:,k), params,kv,kp);
h_a41(:,:,k) = h_a_4x(q_history(:,k),timeVec(k),xhi_history(:,k), params,kv,kp);
h_a42(:,:,k) = h_a_4y(q_history(:,k),timeVec(k),xhi_history(:,k), params,kv,kp);
h_a51(:,:,k) = h_a_5x(q_history(:,k),xhi_history(:,k), params,kp);
h_a52(:,:,k) = h_a_5y(q_history(:,k),xhi_history(:,k), params,kp);

g_a11(:,:,k) = g_a_1x(q_history(:,k),timeVec(k),kv,kp);
g_a12(:,:,k) = g_a_1y(q_history(:,k),timeVec(k),kv,kp);
g_a21(:,:,k) = g_a_2x(q_history(:,k),timeVec(k),kv,kp);
g_a22(:,:,k) = g_a_2y(q_history(:,k),timeVec(k),kv,kp);
g_a31(:,:,k) = g_a_3x(q_history(:,k),timeVec(k),kv,kp);
g_a32(:,:,k) = g_a_3y(q_history(:,k),timeVec(k),kv,kp);
g_a41(:,:,k) = g_a_4x(q_history(:,k),timeVec(k),kv,kp);
g_a42(:,:,k) = g_a_4y(q_history(:,k),timeVec(k),kv,kp);
g_a51(:,:,k) = g_a_5x(q_history(:,k),kp);
g_a52(:,:,k) = g_a_5y(q_history(:,k),kp);
end

%% GAMMA vectors calculation with respect to each a coefficient and TENSOR PRODUCTS.
% Creation of cell arrays to store the integration history of each gamma.
gammaCellArray = cell(Ncoeff,ncolumns);
% La cella contiene 15 matrici per ogni cella in cui, 
% la PRIMA COLONNA presentA la derivata di gamma rispetto ai coefficienti per la X,
% e nella SECONDA COLONNA le derivate per la Y.

%% Tensor product calculations, one for each parameter
% initialize 3D matrixes, which are overwritten 
% how to read: 
% - dfqqgamma--> tensor product between the derivate of f_q respect to q and gamma
% - dfpqgamma--> tensor product between the derivate of f_p respect to q wand gamma
% - dhxhiqgamma--> tensor product between the derivate of h_xhi respect to q with gamma
% Cell Arrays for tensor products of double dervatives with gamma.
dfqqgammaArray = cell(Ncoeff,ncolumns);
dfpqgammaArray  = cell(Ncoeff,ncolumns);
dfuqgammaArray =cell(Ncoeff,ncolumns);
dhqqgammaArray =cell(Ncoeff,ncolumns);
dhqxhigammaxhiArray = cell(Ncoeff,ncolumns);
dhxhiqgammaArray = cell(Ncoeff,ncolumns);
dhxhixhigammaxhiArray =cell(Ncoeff,ncolumns);
dgqqgammaArray  =cell(Ncoeff,ncolumns);
dgqxhigammaxhiArray = cell(Ncoeff,ncolumns);
dgxhiqgammaArray = cell(Ncoeff,ncolumns);
dgxhixhigammaxhiArray = cell(Ncoeff,ncolumns);
dfquuaiArray = cell(Ncoeff,ncolumns);
dfpuuaiArray = cell(Ncoeff,ncolumns);
dfuuuaiArray = cell(Ncoeff,ncolumns);

z=1;
for i = 1:Ncoeff
    % After the 5th coefficient reinitialize the computations.
    if z == Ncoeff/3 +1
        z=1;
    end
    for j = 1:ncolumns
        % Get name of each gamma integrated variable
        %gamma_int_name = sprintf('gamma_int%d%d',i,j);
        h_a_name = sprintf('h_a%d%d',z,j);
        g_a_name = sprintf('g_a%d%d',z,j);
        % Get variable
        %varGammaInt = eval(gamma_int_name);
        varh_a = eval(h_a_name);
        varg_a = eval(g_a_name);
        
        % Initialize gamma_k and the u-ai vec which change at every Nstep.
        gamma_k = zeros(6,1); gamma_int = zeros(6,1,Nstep); 
        %u_ai = zeros(2,1);

        % Initialize tensor products evolution in time
        dfqqgamma_hist = zeros(3,3,Nstep); 
        dfpqgamma_hist = zeros(3,2,Nstep); 
        dfuqgamma_hist = zeros(3,2,Nstep);  
        dhqqgamma_hist = zeros(2,3,Nstep); dhqxhigammaxhi_hist = zeros(2,3,Nstep); 
        dhxhiqgamma_hist = zeros(2,3,Nstep); dhxhixhigammaxhi_hist = zeros(2,3,Nstep); 
        dgqqgamma_hist = zeros(3,3,Nstep); dgqxhigammaxhi_hist = zeros(3,3,Nstep); 
        dgxhiqgamma_hist = zeros(3,3,Nstep); dgxhixhigammaxhi_hist = zeros(3,3,Nstep);
        dfquuai_hist = zeros(3,3,Nstep);
        dfpuuai_hist = zeros(3,2,Nstep);
        dfuuuai_hist = zeros(3,2,Nstep);
        
        for k=1:Nstep
        varh_a_k = varh_a(:,:,k);
        varg_a_k = varg_a(:,:,k);
        % Creation of the vectors of the partial derivaatives 
        f_q = ff_q(q_history(:,k),u_history(:,k),params);
        f_u = ff_u(q_history(:,k),params);
        h_q = fh_q(q_history(:,k),xhi_history(:,k),desired_traj(:,k),dp(:,k),ddp(:,k),params,kv,ki,kp);
        h_xhi = fh_xhi(q_history(:,k),xhi_history(:,k),desired_traj(:,k),dp(:,k),ddp(:,k),params,kv,ki,kp);
        g_q = fg_q(q_history(:,k),xhi_history(:,k),desired_traj(:,k),dp(:,k),ddp(:,k),kv,ki,kp);
        g_xhi = fg_xhi(q_history(:,k),kv,ki,kp);

        %% INTEGRATION OF GAMMA
        [~, gammaint] = ode45(@(t,gammaint) gamma_dot(gammaint, f_q, f_u, g_q, g_xhi, h_q, h_xhi, varh_a_k, varg_a_k), [0 delta], gamma_k);
        gamma_k = gammaint(end, :)'; 
        gamma_int(:,:,k) = gamma_k;
        
        %% U_ai vectots calculation 
        % Calculation of u_ai derivative of the input with respect to each parameter.
        u_ai = h_q*gamma_k(1:3) + h_xhi*gamma_k(4:6) + varh_a_k;

        %% TENSOR PRODUCT CALCULATION
        % For the computation we use the automatically created Matlab Functions 
        gamma = gamma_k(1:3); gammaxhi = gamma_k(4:6);
        % With respect to gamma
        dfqqgamma_hist(:,:,k) = df_q_q_gamma(gamma, params(1:2),u_history(:,k),q_history(:,k));
        dfpqgamma_hist(:,:,k) = df_p_q_gamma(gamma, u_history(:,k), q_history(:,k));
        dfuqgamma_hist(:,:,k) = df_u_q_gamma(gamma, params(1:2), q_history(:,k));
        dhqqgamma_hist(:,:,k) = dh_q_q_gamma(gamma,q_history(:,k),xhi_history(:,k),desired_traj(:,k),dp(:,k),ddp(:,k), params,kv,ki,kp);
        dhqxhigammaxhi_hist(:,:,k) =dh_q_xhi_gammaxhi(gammaxhi,q_history(:,k),xhi_history(:,k),desired_traj(:,k),dp(:,k),ddp(:,k), params,kv,ki,kp);
        dhxhiqgamma_hist(:,:,k) =dh_xhi_q_gamma(gamma,q_history(:,k),xhi_history(:,k),desired_traj(:,k),dp(:,k),ddp(:,k), params,kv,ki,kp);
        dhxhixhigammaxhi_hist(:,:,k) = dh_xhi_xhi_gammaxhi(gammaxhi,q_history(:,k),xhi_history(:,k),desired_traj(:,k),dp(:,k),ddp(:,k), params,kv,ki,kp);
        dgqqgamma_hist(:,:,k) = dg_q_q_gamma(gamma,q_history(:,k),xhi_history(:,k),desired_traj(:,k),dp(:,k),ddp(:,k),kv,ki,kp);
        dgqxhigammaxhi_hist(:, :, k) = dg_q_xhi_gammaxhi(gammaxhi,q_history(:,k),kv,ki,kp);
        dgxhiqgamma_hist(:,:,k) = dg_xhi_q_gamma(gamma,q_history(:,k),kv,ki,kp);
        dgxhixhigammaxhi_hist(:,:,k) = dg_xhi_xhi_gammaxhi();
        % With respect to u
        dfquuai_hist(:, :, k) = df_q_u_uai(q_history(:,k), u_ai, params);
        dfpuuai_hist(:, :, k) = df_p_u_uai(u_ai, q_history(:,k), params);
        dfuuuai_hist(:, :, k) = df_u_u_uai();

        end
        % Store 3D matrixes in cell array
        gammaCellArray{i,j} = gamma_int;

        dfqqgammaArray{i,j} = dfqqgamma_hist;
        dfpqgammaArray{i,j}  = dfpqgamma_hist;
        dfuqgammaArray{i,j} = dfuqgamma_hist;
        dhqqgammaArray{i,j} = dhqqgamma_hist;
        dhqxhigammaxhiArray{i,j} = dhqxhigammaxhi_hist;
        dhxhiqgammaArray{i,j} = dhxhiqgamma_hist;
        dhxhixhigammaxhiArray{i,j} = dhxhixhigammaxhi_hist;
        dgqqgammaArray{i,j}  = dgqqgamma_hist;
        dgqxhigammaxhiArray{i,j} = dgqxhigammaxhi_hist;
        dgxhiqgammaArray{i,j} = dgxhiqgamma_hist;
        dgxhixhigammaxhiArray{i,j} = dgxhixhigammaxhi_hist;
        dfquuaiArray{i,j} = dfquuai_hist;
        dfpuuaiArray{i,j} = dfpuuai_hist;
        dfuuuaiArray{i,j} =dfuuuai_hist;
    end
    z=z+1;
end

%% Creation of derivate of h_q, h_xhi, g_q, g_xhi respect to coefficents [a1_x, a1_y, a2_x, ...]
% These are necessary to make the integration of Sensitivity_ai
% Double derivatives respect to the parameters
dhqa11 = zeros(2,3,Nstep); dhqa12 = zeros(2,3,Nstep); dhqa21 = zeros(2,3,Nstep); dhqa22 = zeros(2,3,Nstep); dhqa31 = zeros(2,3,Nstep); dhqa32 = zeros(2,3,Nstep); dhqa41 = zeros(2,3,Nstep); dhqa42 = zeros(2,3,Nstep); dhqa51 = zeros(2,3,Nstep); dhqa52 = zeros(2,3,Nstep);
dhxhia11 = zeros(2,3,Nstep); dhxhia12 = zeros(2,3,Nstep); dhxhia21 = zeros(2,3,Nstep); dhxhia22 = zeros(2,3,Nstep); dhxhia31 = zeros(2,3,Nstep); dhxhia32 = zeros(2,3,Nstep); dhxhia41 = zeros(2,3,Nstep); dhxhia42 = zeros(2,3,Nstep); dhxhia51 = zeros(2,3,Nstep); dhxhia52 = zeros(2,3,Nstep); 
dgqa11 = zeros(3,3,Nstep); dgqa12 = zeros(3,3,Nstep); dgqa21 = zeros(3,3,Nstep); dgqa22 = zeros(3,3,Nstep); dgqa31 = zeros(3,3,Nstep); dgqa32 = zeros(3,3,Nstep); dgqa41 = zeros(3,3,Nstep); dgqa42 = zeros(3,3,Nstep); dgqa51 = zeros(3,3,Nstep); dgqa52 = zeros(3,3,Nstep); 
dgxhia11 = zeros(3,3,Nstep); dgxhia12 = zeros(3,3,Nstep); dgxhia21 = zeros(3,3,Nstep); dgxhia22 = zeros(3,3,Nstep); dgxhia31 = zeros(3,3,Nstep); dgxhia32 = zeros(3,3,Nstep); dgxhia41 = zeros(3,3,Nstep); dgxhia42 = zeros(3,3,Nstep); dgxhia51 = zeros(3,3,Nstep); dgxhia52 = zeros(3,3,Nstep);

for k=1:Nstep
dhqa11(:,:,k) = dhq_a1x(params(2), kp, kv, params(1), timeVec(k), q_history(3,k), xhi_history(1,k));
dhqa21(:,:,k) = dhq_a2x(params(2), kp, kv, params(1), timeVec(k), q_history(3,k), xhi_history(1,k));
dhqa31(:,:,k) = dhq_a3x(params(2), kp, kv, params(1), timeVec(k), q_history(3,k), xhi_history(1,k));
dhqa41(:,:,k) = dhq_a4x(params(2), kp, kv, params(1), timeVec(k), q_history(3,k), xhi_history(1,k));
dhqa51(:,:,k) = dhq_a5x(params(2), kp, params(1), q_history(3,k), xhi_history(1,k));

dhqa12(:,:,k) = dhq_a1y(params(2), kp, kv, params(1), timeVec(k), q_history(3,k), xhi_history(1,k));
dhqa22(:,:,k) = dhq_a2y(params(2), kp, kv, params(1), timeVec(k), q_history(3,k), xhi_history(1,k));
dhqa32(:,:,k) = dhq_a3y(params(2), kp, kv, params(1), timeVec(k), q_history(3,k), xhi_history(1,k));
dhqa42(:,:,k) = dhq_a4y(params(2), kp, kv, params(1), timeVec(k), q_history(3,k), xhi_history(1,k));
dhqa52(:,:,k) = dhq_a5y(params(2), kp, params(1), q_history(3,k), xhi_history(1,k));

dhxhia11(:,:,k) = dhxhi_a1x(params(2), kp, kv, params(1), timeVec(k), q_history(3,k), xhi_history(1,k));
dhxhia21(:,:,k) = dhxhi_a2x(params(2), kp, kv, params(1), timeVec(k), q_history(3,k), xhi_history(1,k));
dhxhia31(:,:,k) = dhxhi_a3x(params(2), kp, kv, params(1), timeVec(k), q_history(3,k), xhi_history(1,k));
dhxhia41(:,:,k) = dhxhi_a4x(params(2), kp, kv, params(1), timeVec(k), q_history(3,k), xhi_history(1,k));
dhxhia51(:,:,k) = dhxhi_a5x(params(2), kp, params(1), q_history(3,k), xhi_history(1,k));

dhxhia12(:,:,k) = dhxhi_a1y(params(2), kp, kv, params(1), timeVec(k), q_history(3,k), xhi_history(1,k));
dhxhia22(:,:,k) = dhxhi_a2y(params(2), kp, kv, params(1), timeVec(k), q_history(3,k), xhi_history(1,k));
dhxhia32(:,:,k) = dhxhi_a3y(params(2), kp, kv, params(1), timeVec(k), q_history(3,k), xhi_history(1,k));
dhxhia42(:,:,k) = dhxhi_a4y(params(2), kp, kv, params(1), timeVec(k), q_history(3,k), xhi_history(1,k));
dhxhia52(:,:,k) = dhxhi_a5y(params(2), kp, params(1), q_history(3,k), xhi_history(1,k));

dgqa11(:,:,k) = dgq_a1x(kp, kv, timeVec(k), q_history(3,k));
dgqa21(:,:,k) = dgq_a2x(kp, kv, timeVec(k), q_history(3,k));
dgqa31(:,:,k) = dgq_a3x(kp, kv, timeVec(k), q_history(3,k));
dgqa41(:,:,k) = dgq_a4x(kp, kv, timeVec(k), q_history(3,k));
dgqa51(:,:,k) = dgq_a5x(kp, q_history(3,k));
dgqa12(:,:,k) = dgq_a1y(kp, kv, timeVec(k), q_history(3,k));
dgqa22(:,:,k) = dgq_a2y(kp, kv, timeVec(k), q_history(3,k));
dgqa32(:,:,k) = dgq_a3y(kp, kv, timeVec(k), q_history(3,k));
dgqa42(:,:,k) = dgq_a4y(kp, kv, timeVec(k), q_history(3,k));
dgqa52(:,:,k) = dgq_a5y(kp, q_history(3,k));

dgxhia11(:,:,k)=dgxhi_a1x();
dgxhia21(:,:,k)=dgxhi_a2x();
dgxhia31(:,:,k)=dgxhi_a3x();
dgxhia41(:,:,k)=dgxhi_a4x();
dgxhia51(:,:,k)=dgxhi_a5x();
dgxhia12(:,:,k)=dgxhi_a1y();
dgxhia22(:,:,k)=dgxhi_a2y();
dgxhia32(:,:,k)=dgxhi_a3y();
dgxhia42(:,:,k)=dgxhi_a4y();
dgxhia52(:,:,k)=dgxhi_a5y();
end

%% FINAL SENSITIVITY_Ai CALCULATION THROUGHT INTEGRATION OF ODEs
sensaiCellArray = cell(Ncoeff,ncolumns);
% Repeat the calculations 3 times, one for each trajectory
z=1;
for i = 1:Ncoeff
    % After the 5th coefficient reinitialize the computations.
    if z ==Ncoeff/3 +1
        z=1;
    end
    for j = 1:ncolumns
        sensai_int = zeros(12,1,Nstep); 
        dgq = eval(sprintf('dgqa%d%d', z,j)); 
        dgxhi = eval(sprintf('dgxhia%d%d', z,j)); 
        dhq = eval(sprintf('dhqa%d%d', z,j)); 
        dhxhi = eval(sprintf('dhxhia%d%d', z,j));

        for k=1:Nstep-1

            % The first derivatives needed here as well
            f_q = ff_q(q_history(:,k),u_history(:,k),params);
            f_u = ff_u(q_history(:,k),params);
            h_q = fh_q(q_history(:,k),xhi_history(:,k),desired_traj(:,k),dp(:,k),ddp(:,k),params,kv,ki,kp);
            h_xhi = fh_xhi(q_history(:,k),xhi_history(:,k),desired_traj(:,k),dp(:,k),ddp(:,k),params,kv,ki,kp);
            g_q = fg_q(q_history(:,k),xhi_history(:,k),desired_traj(:,k),dp(:,k),ddp(:,k),kv,ki,kp);
            g_xhi = fg_xhi(q_history(:,k),kv,ki,kp);

            % Sensitivity_ai integration
            sensai_int(:,:,k+1) = sensai_int(:,:,k) + delta*sensitivity_ai_dot(sensai_int(:,:,k),dfqqgammaArray{i,j}(:,:,k),dfquuaiArray{i,j}(:,:,k),sens_int(1:6,k),...
            f_q ,dfpqgammaArray{i,j}(:,:,k),dfpuuaiArray{i,j}(:,:,k),dfuqgammaArray{i,j}(:,:,k),dfuuuaiArray{i,j}(:,:,k),...
            h_q,h_xhi, sens_int(7:12,k), f_u,dhqqgammaArray{i,j}(:,:,k),dhqxhigammaxhiArray{i,j}(:,:,k),...
            dhq(:,:,k),dhxhiqgammaArray{i,j}(:,:,k), dhxhixhigammaxhiArray{i,j}(:,:,k),dhxhi(:,:,k),dgqqgammaArray{i}(:,:,k),...
            dgqxhigammaxhiArray{i,j}(:,:,k),dgq(:,:,k),dgxhiqgammaArray{i,j}(:,:,k),...
            dgxhixhigammaxhiArray{i,j}(:,:,k),dgxhi(:,:,k), g_q, g_xhi);
        end
        sensaiCellArray{i,j} = sensai_int;
    end
    z=z+1;
end

allSensitivitiesInOneArray = sensaiCellArray;

end 

