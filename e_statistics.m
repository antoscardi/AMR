close all; clc; 

fileCoeff = 'data/coeff_a_star';
dataCoefficients = load(fileCoeff, 'ax_star', 'ay_star');
ax = dataCoefficients.ax_star;
ay = dataCoefficients.ay_star;


%% GENERATION OF THE COEFFICIENT RELATED TO OPTIMAL TRAJECTORY
optimizMatrix = [ax, ay];

%% GENERATION OF THE COEFFICIENT RELATED TO NON-OPTIMIZED TRAJECTORY
[coeffs, ~] = coeff_generation(totalTime, dx, dy);

%% GENERATE OPTIMIZED TRAJECTORY
[posOpt, velOpt, accOpt, thetaOpt] = trajectory_generation(optimizMatrix, timeVec, totalTime, ...
    linewidth, colors, true);

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
figure(70); hold on
vectorForOptStateLegend = zeros(22,1);
counterTrajForLegend = 1;
colorsOfDifferentTrajectories = linspecer(25,'sequential');
vectorForOptStateLegend(counterTrajForLegend) = plot(posOpt(1, :), posOpt(2, :), 'Color', colorsOfDifferentTrajectories(counterTrajForLegend, :), 'LineWidth', 4.5, 'DisplayName', sprintf('Trajectory Optimal'));
xlabel("x[m]"), ylabel('y[m]'), grid minor
title('State Variation'), fontsize(fontSize, "points")
legend('show');
drawnow;
legend(vectorForOptStateLegend(1:counterTrajForLegend))
per_params = zeros(2,20);
err_vec = zeros(4, 20);

e_nopt=zeros(3,21); e_opt=zeros(3,21);
for i=1:21
    counterTrajForLegend = counterTrajForLegend + 1;
    var_r = randi([80,120])/100;
    var_b = randi([80,120])/100;
    per_params(:,i) = [var_r*wheelRadius;
                       var_b*wheelDistance];

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
        vectorForOptStateLegend(counterTrajForLegend) = plot(q_OPT_PERT(1, :), q_OPT_PERT(2, :), 'Color', colorsOfDifferentTrajectories(counterTrajForLegend, :), 'LineWidth', linewidth, 'LineStyle', '-.', 'DisplayName', sprintf('State variation n: %d', counterTrajForLegend));
        xlabel("x[m]"), ylabel('y[m]'), grid on
        title('State variation:'), fontsize(fontSize, "points")
        legend('show');
        drawnow;
        legend(vectorForOptStateLegend(1:counterTrajForLegend))
    end
    if mod(i, 2) ~= 0
        vectorForOptStateLegend(counterTrajForLegend) = plot(q_OPT_PERT(1, :), q_OPT_PERT((2), :), 'Color', colorsOfDifferentTrajectories(counterTrajForLegend, :), 'LineWidth', linewidth, 'DisplayName', sprintf('State variation n: %d', counterTrajForLegend));
        xlabel("x[m]"), ylabel('y[m]'), grid on
        title('State variation:'), fontsize(fontSize, "points")
        legend('show');
        drawnow;
        legend(vectorForOptStateLegend(1:counterTrajForLegend))
    end
end
hold off

%% GENERATION OF NON-OPTIMAL STATES ROBOT
figure(65); hold on
vectorForNotOptimalStateLegend = zeros(22);
counterTrajForLegendNotOpt = 1;
colorsOfDifferentTrajectoriesNotOpt = linspecer(25,'sequential');
vectorForNotOptimalStateLegend(counterTrajForLegendNotOpt) = plot(posNonOpt(1, :), posNonOpt(2, :), 'Color', colorsOfDifferentTrajectoriesNotOpt(counterTrajForLegendNotOpt, :), 'LineWidth', 4.5, 'DisplayName', sprintf('Trajectory not Optimal'));
xlabel("x[m]"), ylabel('y[m]'), grid minor
title('State variation'), fontsize(fontSize, "points")
legend('show');
drawnow;
legend(vectorForNotOptimalStateLegend(1:counterTrajForLegendNotOpt))

for j=1:21
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
        vectorForNotOptimalStateLegend(counterTrajForLegendNotOpt) = plot(q_NOPT_PERT(1, :), q_NOPT_PERT(2, :), 'Color', colorsOfDifferentTrajectoriesNotOpt(counterTrajForLegendNotOpt, :), 'LineWidth', linewidth, 'LineStyle', '-.', 'DisplayName', sprintf('State variation n: %d', counterTrajForLegendNotOpt));
        xlabel("x[m]"), ylabel('y[m]'), grid on
        title('State Variation'), fontsize(fontSize, "points")
        legend('show');
        drawnow;
        legend(vectorForNotOptimalStateLegend(1:counterTrajForLegendNotOpt))
    end
    if mod(j, 2) ~= 0
        vectorForNotOptimalStateLegend(counterTrajForLegendNotOpt) = plot(q_NOPT_PERT(1, :), q_NOPT_PERT((2), :), 'Color', colorsOfDifferentTrajectoriesNotOpt(counterTrajForLegendNotOpt, :), 'LineWidth', linewidth, 'DisplayName', sprintf('State variation n: %d', counterTrajForLegendNotOpt));
        xlabel("x[m]"), ylabel('y[m]'), grid on
        title('State Variation'), fontsize(fontSize, "points")
        legend('show');
        drawnow;
        legend(vectorForNotOptimalStateLegend(1:counterTrajForLegendNotOpt))
    end
end

our_mean_opt_tot=0; our_mean_nopt_tot=0; our_mean_opt_theta=0; our_mean_nopt_theta=0;
paper_mean_opt=0; paper_mean_nopt=0;
for c=1:21
    %Our statistic
    our_mean_opt_tot = our_mean_opt_tot + sqrt_of_quadratics(e_opt(:,c));
    our_mean_nopt_tot = our_mean_nopt_tot + sqrt_of_quadratics(e_nopt(:,c));
    our_mean_opt_theta = our_mean_opt_theta + e_opt(3,c);
    our_mean_nopt_theta = our_mean_nopt_theta + e_nopt(3,c);

    %Paper statistic
    paper_mean_opt= paper_mean_opt + sqrt(e_opt(1,c)^2 + e_opt(2,c)^2+e_opt(3,c)^2);
    paper_mean_nopt= paper_mean_nopt + sqrt(e_nopt(1,c)^2 + e_nopt(2,c)^2+e_nopt(3,c)^2);
end
strg = ['Our mean_tot in the opt case is:', sprintf('%1.5f',(our_mean_opt_tot/21))]; 
disp(strg)
strg = ['Our mean_tot in the Nopt case is:', sprintf('%1.5f',(our_mean_nopt_tot/21))]; 
disp(strg)
strg = ['Our mean of theta in the opt case is:', sprintf('%1.5f',(our_mean_opt_theta/21))]; 
disp(strg)
strg = ['Our mean of theta in the Nopt case is:', sprintf('%1.5f',(our_mean_nopt_theta/21))]; 
disp(strg)
strg = ['Paper mean in the Nopt case is:', sprintf('%1.5f',(paper_mean_nopt/21))]; 
disp(strg)
strg = ['Paper mean in the opt case is:', sprintf('%1.5f',(paper_mean_opt/21))]; 
disp(strg)

%disp(err_vec)
% disp(e_opt)
% strg = ['La media del errore nel caso ottimo è :', sprintf('%1.5f',mean(e_opt,'all'))];
% disp(strg);
% strg = ['La media del errore nel caso NON ottimo è:', sprintf('%1.5f',mean(e_nopt,'all'))];
% disp(strg );
