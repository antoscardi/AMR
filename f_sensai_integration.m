close all; clc;
%% Load data
% Iterate in the data folder to extract ALL data in the folder
folderpath = '../AMR/data';
% Get a list of all files in the folder
filelist = dir(folderpath);
% Iterate through each file in the folder
for i = 1:length(filelist)
    filename = filelist(i).name;
    % Skip directories
    if filelist(i).isdir
        continue;
    end
    % Load the file and get a struct
    data = load(filename);
    % Find all variables in the struct 
    varNames = fieldnames(data);
    % Extract all variables
    for j = 1:length(varNames)
        eval(sprintf('%s = data.%s;', varNames{j}, varNames{j}));
    end
end

%% Tensor product calculations, one for each parameter
% initialize 3D matrixes, which are overwritten  
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
for i = 1:12
    % Get name of each gamma integrated variable
    gamma_name = sprintf('gammax%d_int', i); uai_name = sprintf('u_ax%d', i);
    % Get variable
    var = eval(gamma_name); var2 = eval(uai_name);
    % Loop over time steps
    for k = 1:Nstep
        gamma = var(1:3,k); gammaxhi = var(4:6,k); u_ai = var2(:,:,k);
        curr_dfqqgamma(:,:,k) = df_q_q_gamma(gamma, params(:,k),u_history(:,k),q_history(:,k));
        curr_dfquuai(:,:,k) = df_q_u_uai(q_history(:,k),u_ai,params(:,k));
        curr_dfpqgamma(:,:,k) = df_p_q_gamma(gamma, u_history(:,k), q_history(:,k));
        curr_dfpuuai(:,:,k) = df_p_u_uai(u_ai, q_history(:,k), params(:,k));
        curr_dfuqgamma(:,:,k) = df_u_q_gamma(gamma, params(:,k), q_history(:,k));
        curr_dfuuuai(:,:,k) = df_u_u_uai();
        curr_dhqqgamma(:,:,k) = dh_q_q_gamma(gamma,q_history(:,k),xhi_history(:,k),p(:,k),dp(:,k),ddp(:,k));
        curr_dhqxhigammaxhi(:,:,k) =dh_q_xhi_gammaxhi(gammaxhi,q_history(:,k),xhi_history(:,k),p(:,k),dp(:,k),ddp(:,k));
        curr_dhxhiqgamma(:,:,k) =dh_xhi_q_gamma(gamma,q_history(:,k),xhi_history(:,k),p(:,k),dp(:,k),ddp(:,k));
        curr_dhxhixhigammaxhi(:,:,k) = dh_xhi_xhi_gammaxhi(gammaxhi,q_history(:,k),xhi_history(:,k),p(:,k),dp(:,k),ddp(:,k));
        curr_dgqqgamma(:,:,k) = dg_q_q_gamma(gamma,q_history(:,k),xhi_history(:,k),p(:,k),dp(:,k),ddp(:,k));
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
for i = 1:12
    % Get name of each gamma integrated variable
    gamma_name = sprintf('gammay%d_int', i); uai_name = sprintf('u_ay%d', i);
    % Get variable
    var = eval(gamma_name); var2 = eval(uai_name);
    % Loop over time steps
    for k = 1:Nstep
        gamma = var(1:3,k); gammaxhi = var(4:6,k); u_ai = var2(:,:,k);
        curr_dfqqgamma(:,:,k) = df_q_q_gamma(gamma, params(:,k),u_history(:,k),q_history(:,k));
        curr_dfquuai(:,:,k) = df_q_u_uai(q_history(:,k),u_ai,params(:,k));
        curr_dfpqgamma(:,:,k) = df_p_q_gamma(gamma, u_history(:,k), q_history(:,k));
        curr_dfpuuai(:,:,k) = df_p_u_uai(u_ai, q_history(:,k), params(:,k));
        curr_dfuqgamma(:,:,k) = df_u_q_gamma(gamma, params(:,k), q_history(:,k));
        curr_dfuuuai(:,:,k) = df_u_u_uai();
        curr_dhqqgamma(:,:,k) = dh_q_q_gamma(gamma,q_history(:,k),xhi_history(:,k),p(:,k),dp(:,k),ddp(:,k));
        curr_dhqxhigammaxhi(:,:,k) =dh_q_xhi_gammaxhi(gammaxhi,q_history(:,k),xhi_history(:,k),p(:,k),dp(:,k),ddp(:,k));
        curr_dhxhiqgamma(:,:,k) =dh_xhi_q_gamma(gamma,q_history(:,k),xhi_history(:,k),p(:,k),dp(:,k),ddp(:,k));
        curr_dhxhixhigammaxhi(:,:,k) = dh_xhi_xhi_gammaxhi(gammaxhi,q_history(:,k),xhi_history(:,k),p(:,k),dp(:,k),ddp(:,k));
        curr_dgqqgamma(:,:,k) = dg_q_q_gamma(gamma,q_history(:,k),xhi_history(:,k),p(:,k),dp(:,k),ddp(:,k));
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

%% FINAL SENSITIVITY CALCULATION THROUGHT INTEGRATION OF ODEs
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
         h_q(:,:,k), h_xhi(:,:,k),sens_int(6:12,k), f_u(:,:,k), allx_dhqqgamma{i}(:,:,k),allx_dhqxhigammaxhi{i}(:,:,k),...
         dhq(:,:,k),allx_dhxhiqgamma{i}(:,:,k), allx_dhxhixhigammaxhi{i}(:,:,k),dhxhi(:,:,k),allx_dgqqgamma{i}(:,:,k),allx_dgqxhigammaxhi{i}(:,:,k),...
         dgq(:,:,k),allx_dgxhiqgamma{i}(:,:,k),allx_dgxhixhigammaxhi{i}(:,:,k),dgxhi(:,:,k), g_q(:,:,k), g_xhi(:,:,k)), [0 delta],sens_k);
sens_k = j(end, :)'; sensai_tot(:,:,k) = sens_k;
end
z=z+1;
all_sensaix{i} = sensai_tot;
end

% For the y parameters %%%% VANNO SOSTITUTE LE Y 
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
         h_q(:,:,k), h_xhi(:,:,k),sens_int(6:12,k), f_u(:,:,k), ally_dhqqgamma{i}(:,:,k),ally_dhqxhigammaxhi{i}(:,:,k),...
         dhq(:,:,k),ally_dhxhiqgamma{i}(:,:,k), ally_dhxhixhigammaxhi{i}(:,:,k),dhxhi(:,:,k),ally_dgqqgamma{i}(:,:,k),ally_dgqxhigammaxhi{i}(:,:,k),...
         dgq(:,:,k),ally_dgxhiqgamma{i}(:,:,k),ally_dgxhixhigammaxhi{i}(:,:,k),dgxhi(:,:,k), g_q(:,:,k), g_xhi(:,:,k)), [0 delta],sens_k);
sens_k = j(end, :)'; sensai_tot(:,:,k) = sens_k;
end
z=z+1;
all_sensaiy{i} = sensai_tot;
end

save('data/sensitivity_ai','all_sensaix','all_sensaiy')

%% Integration function Ode
function djdt = integralSens_ai(t, j, dfqqgamma, dfquuai, sens, f_q, dfpqgamma, dfpuuai,dfuqgamma,dfuuuai,h_q, h_xhi, sens_xhi, f_u, dhqqgamma,dhqxhigammaxhi,hq_ai,dhxhiqgamma, dhxhixhigammaxhi, h_xhi_ai, dgqqgamma,dgqxhigammaxhi,g_q_ai,dgxhiqgamma,dgxhixhigammaxhi,g_xhi_ai, g_q, g_xhi)
    % Create matrixes from column vector
    sensai = [j(1) j(2);
            j(3) j(4);
            j(5) j(6)];

    sensxhiai = [j(7) j(8);
               j(9) j(10);
               j(11) j(12)];

    sens_mat = [sens(1) sens(2);
               sens(3) sens(4);
               sens(5) sens(6)];
    
    sensxhi_mat = [sens_xhi(1) sens_xhi(2);
                   sens_xhi(3) sens_xhi(4);
                   sens_xhi(5) sens_xhi(6)];
    
    % Sensitivity component of the ODEs  
    dsensai = (dfqqgamma + dfquuai)*sens_mat + f_q*sensai + dfpqgamma + dfpuuai + (dfuqgamma + dfuuuai)*(h_q*sens_mat + h_xhi*sensxhi_mat) + f_u*((dhqqgamma + dhqxhigammaxhi + hq_ai)*sens_mat + (dhxhiqgamma + dhxhixhigammaxhi + h_xhi_ai)*sensxhi_mat + h_q*sensai + h_xhi*sensxhiai);
    % sensitivity_xhi component of the ODEs
    dsensxhiai = (dgqqgamma + dgqxhigammaxhi + g_q_ai)*sens_mat + (dgxhiqgamma + dgxhixhigammaxhi + g_xhi_ai)*sensxhi_mat + g_q*sensai + g_xhi*sensxhiai; 
    % Output
    djdt = [dsensai(1,1); dsensai(1,2); dsensai(2,1);dsensai(2,2);dsensai(3,1); dsensai(3,2);dsensxhiai(1,1); dsensxhiai(1,2); dsensxhiai(2,1);dsensxhiai(2,2);dsensxhiai(3,1);dsensxhiai(3,2)];
end 


