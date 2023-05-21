close all; clc; 
%% INIZIALIZE RANDOM SEED FOR REPRODUCTION
rng('default');
rng(42);

fileCoeff = 'data/coeff_a_star';
dataCoefficients = load(fileCoeff, 'ax_star', 'ay_star');
ax = dataCoefficients.ax_star;
ay = dataCoefficients.ay_star;

%% SETTING THE NUMBER OF PERTURBATIONS OF THE PARAMETERS
numberOfPerturbations = 20;

%% GENERATION OF THE COEFFICIENT RELATED TO OPTIMAL TRAJECTORY
optimizMatrix = [ax, ay];

%% GENERATION OF THE COEFFICIENT RELATED TO NON-OPTIMIZED TRAJECTORY
[coeffs, ~] = coeff_generation(totalTime, dx, dy);

%% GENERATE OPTIMIZED TRAJECTORY
[posOpt, velOpt, accOpt, thetaOpt] = trajectory_generation(optimizMatrix, timeVec, totalTime, ...
    linewidth, colors, false);

%% GENERATION OF OPTIMIZED TRAJECTORY WITH NOMINAL PARAMETERS
[q_OPT_NOM, ~, ~, e_OPT_NOM] = simulation_loop(initialPositionVec, initialVelocityVec, ...
    delta, ...
    nominal_params, perturbed_params, false, ...
    posOpt, velOpt, accOpt, thetaOpt, kv,ki,kp);
% Generate NON-OPTIMAL trajectory
[posNonOpt, velNonOpt, accNonOpt, thetaNonOpt] = trajectory_generation(coeffs, timeVec, totalTime, ...
    linewidth, colors, false);

%% GENERATION OF NOT-OPTIMIZED TRAJECTORY WITH NOMINAL PARAMETERS
[q_NOPT_NOM, ~, ~, e_NOPT_NOM] = simulation_loop(initialPositionVec, initialVelocityVec, ...
    delta, ...
    nominal_params, perturbed_params, false, ...
    posNonOpt, velNonOpt, accNonOpt, thetaNonOpt, kv,ki,kp);

hold off
%% GENERATION OF OPTIMAL STATES ROBOT
fig1 = figure(120); hold on
set(gcf, 'Position', get(0, 'Screensize'));
vectorForOptStateLegend = zeros(numberOfPerturbations+1,1);
counterTrajForLegend = 1;
colorsOfDifferentTrajectories = linspecer(numberOfPerturbations+10,'sequential');
vectorForOptStateLegend(counterTrajForLegend) = plot( posOpt(1, :), posOpt(2, :), 'Color', colorsOfDifferentTrajectories(counterTrajForLegend, :), 'LineWidth', 4.5, 'DisplayName', sprintf('Trajectory Optimal'));
xlabel("x [m]"), ylabel('y [m]','Rotation',0), grid on
title('Unicycle State Optimal Trajectory Tracking variation w.r.t. parameters:')
subtitle('parameters sampled randomly from uniform distribution: $p \sim \mathcal{U}_{(0.8p,1.2p)}$')
fontsize(fontSize, "points")
legend('show');
drawnow;
legend(vectorForOptStateLegend(1:counterTrajForLegend))
per_params = zeros(2,numberOfPerturbations);
err_vec = zeros(4, numberOfPerturbations);

e_nopt=zeros(3,numberOfPerturbations); e_opt=zeros(3,numberOfPerturbations);
for i=1:numberOfPerturbations
    set(0,'CurrentFigure',fig1);
    counterTrajForLegend = counterTrajForLegend + 1;
    var_r = randi([80,120])/100;
    var_b = randi([80,120])/100;
    per_params(:,i) = [var_r*wheelRadius;
                       var_b*wheelDistance];
    var_total = [var_r;
                 var_b]

    %% GENERATION OF OPTIMIZED TRAJECTORY WITH PERTURBED PARAMETERS
    [q_OPT_PERT, ~, ~, e_OPT_PERT] = simulation_loop(initialPositionVec, initialVelocityVec, ...
    delta, ...
    nominal_params, per_params(:,i), true, ...
    posOpt, velOpt, accOpt, thetaOpt, kv, ki, kp);
    
    err_vec(1,i)=e_OPT_PERT(end,3);
    err_vec(2,i)=e_OPT_PERT(end,4);
    e_opt(:,i)=abs(q_OPT_NOM(:,end)-q_OPT_PERT(:,end));

% Plot the trajectories with different lines and different colors
    if mod(i, 2) == 0
        vectorForOptStateLegend(counterTrajForLegend) = plot(q_OPT_PERT(1, :), q_OPT_PERT(2, :), 'Color', colorsOfDifferentTrajectories(counterTrajForLegend, :), 'LineWidth', linewidth, 'LineStyle', '-.', 'DisplayName', sprintf('State variation n: %d', i));
        legend('show');
        drawnow;
        legend(vectorForOptStateLegend(1:counterTrajForLegend),'FontSize',15)
    end
    if mod(i, 2) ~= 0
        vectorForOptStateLegend(counterTrajForLegend) = plot(q_OPT_PERT(1, :), q_OPT_PERT((2), :), 'Color', colorsOfDifferentTrajectories(counterTrajForLegend, :), 'LineWidth', linewidth, 'DisplayName', sprintf('State variation n: %d', i));
        legend('show');
        drawnow;
        legend(vectorForOptStateLegend(1:counterTrajForLegend),'FontSize',15)
    end
    
end
hold off

%% GENERATION OF NON-OPTIMAL STATES ROBOT
fig2 = figure(100); hold 
set(gcf, 'Position', get(0, 'Screensize'));
vectorForNotOptimalStateLegend = zeros(numberOfPerturbations+1,1);
counterTrajForLegendNotOpt = 1;
colorsOfDifferentTrajectoriesNotOpt = linspecer(numberOfPerturbations+10,'sequential');
vectorForNotOptimalStateLegend(counterTrajForLegendNotOpt) = plot(posNonOpt(1, :), posNonOpt(2, :), 'Color', colorsOfDifferentTrajectoriesNotOpt(counterTrajForLegendNotOpt, :), 'LineWidth', 4.5, 'DisplayName', sprintf('Trajectory not Optimal'));
title('Unicycle State Non-Optimal Trajectory Tracking variation w.r.t. parameters:')
subtitle('parameters sampled randomly from uniform distribution: $p \sim \mathcal{U}_{(0.8p,1.2p)}$')
xlabel("x [m]"), ylabel('y [m]','Rotation',0), grid on
fontsize(fontSize, "points")
legend('show');
drawnow;
legend(vectorForNotOptimalStateLegend(1:counterTrajForLegendNotOpt))

for j=1:numberOfPerturbations
   set(0,'CurrentFigure',fig2);
   counterTrajForLegendNotOpt = counterTrajForLegendNotOpt + 1;
   %% GENERATION OF NOT-OPTIMIZED TRAJECTORY WITH PERTURBED PARAMETERS
   [q_NOPT_PERT, ~, ~, e_NOPT_PERT] = simulation_loop(initialPositionVec, initialVelocityVec, ...
    delta, ...
    nominal_params, per_params(:,j), true, ...
    posNonOpt, velNonOpt, accNonOpt, thetaNonOpt, kv,ki,kp);

   err_vec(3,j)= e_NOPT_PERT(end,4);
   err_vec(4,j)= e_NOPT_PERT(end,3);
   e_nopt(:,j)=abs(q_NOPT_NOM(:,end)-q_NOPT_PERT(:,end));

   % Plot the trajectories with different lines and different colors
    if mod(j, 2) == 0
        vectorForNotOptimalStateLegend(counterTrajForLegendNotOpt) = plot(q_NOPT_PERT(1, :), q_NOPT_PERT(2, :), 'Color', colorsOfDifferentTrajectoriesNotOpt(counterTrajForLegendNotOpt, :), 'LineWidth', linewidth, 'LineStyle', '-.', 'DisplayName', sprintf('State variation n: %d', j));
        legend('show');
        drawnow;
        legend(vectorForNotOptimalStateLegend(1:counterTrajForLegendNotOpt),'FontSize',15)
    end
    if mod(j, 2) ~= 0
        vectorForNotOptimalStateLegend(counterTrajForLegendNotOpt) = plot(q_NOPT_PERT(1, :), q_NOPT_PERT((2), :), 'Color', colorsOfDifferentTrajectoriesNotOpt(counterTrajForLegendNotOpt, :), 'LineWidth', linewidth, 'DisplayName', sprintf('State variation n: %d', j));
        legend('show');
        drawnow;
        legend(vectorForNotOptimalStateLegend(1:counterTrajForLegendNotOpt),'FontSize',15)
    end
end

all_our_errors_opt_tot = zeros(1,numberOfPerturbations); all_our_errors_nopt_tot = zeros(1,numberOfPerturbations); all_our_errors_theta_opt = zeros(1,numberOfPerturbations); all_our_errors_theta_nopt = zeros(1,numberOfPerturbations);
all_paper_errors_opt = zeros(1,numberOfPerturbations); all_paper_errors_nopt = zeros(1,numberOfPerturbations); 
for z = 1:numberOfPerturbations
    all_our_errors_opt_tot(z) = sqrt_of_quadratics(e_opt(:,z));
    all_our_errors_nopt_tot(z) = sqrt_of_quadratics(e_nopt(:,z));
    all_our_errors_theta_opt(z) = e_opt(3,z);
    all_our_errors_theta_nopt(z) = e_nopt(3,z);
    all_paper_errors_opt(z) = sqrt(e_opt(1,z)^2 + e_opt(2,z)^2+e_opt(3,z)^2);
    all_paper_errors_nopt(z) = sqrt(e_nopt(1,z)^2 + e_nopt(2,z)^2+e_nopt(3,z)^2);
end
our_std_opt = std(all_our_errors_opt_tot);
our_std_nopt = std(all_our_errors_nopt_tot);
our_std_theta_opt = std(all_our_errors_theta_opt);
our_std_theta_nopt = std(all_our_errors_theta_nopt);
paper_std_errors_opt = std(all_paper_errors_opt);
paper_std_errors_nopt = std(all_paper_errors_nopt);

our_mean_opt_tot = mean(all_our_errors_opt_tot);
our_mean_nopt_tot = mean(all_our_errors_nopt_tot);
our_mean_opt_theta = mean(all_our_errors_theta_opt);
our_mean_nopt_theta = mean(all_our_errors_theta_nopt);
paper_mean_opt = mean(all_paper_errors_opt);
paper_mean_nopt = mean(all_paper_errors_nopt);

strg = ['Our mean_tot in the opt case is:', sprintf('%1.5f',(our_mean_opt_tot))]; 
disp(strg)
strg = ['Our mean_tot in the Nopt case is:', sprintf('%1.5f',(our_mean_nopt_tot))]; 
disp(strg)
strg = ['Our mean of theta in the opt case is:', sprintf('%1.5f',(our_mean_opt_theta))]; 
disp(strg)
strg = ['Our mean of theta in the Nopt case is:', sprintf('%1.5f',(our_mean_nopt_theta))]; 
disp(strg)
strg = ['Paper mean in the Nopt case is:', sprintf('%1.5f',(paper_mean_nopt))]; 
disp(strg)
strg = ['Paper mean in the opt case is:', sprintf('%1.5f',(paper_mean_opt))]; 
disp(strg)

strg = ['Our standard deviation tot in the opt case is:', sprintf('%1.5f',(our_std_opt))]; 
disp(strg)
strg = ['Our standard deviation tot in the Nopt case is:', sprintf('%1.5f',(our_std_nopt))]; 
disp(strg)
strg = ['Our standard deviation of theta in the opt case is:', sprintf('%1.5f',(our_std_theta_opt))]; 
disp(strg)
strg = ['Our standard deviation of theta in the Nopt case is:', sprintf('%1.5f',(our_std_theta_nopt))]; 
disp(strg)
strg = ['Paper standard deviation in the opt case is:', sprintf('%1.5f',(paper_std_errors_opt))]; 
disp(strg)
strg = ['Paper standard deviation in the Nopt case is:', sprintf('%1.5f',(paper_std_errors_nopt))]; 
disp(strg)
