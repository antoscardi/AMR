%% GENERATION OF OPTIMAL TRAJECTORY VARYING THE PERTURBED PARAMETERS
b = zeros(21);
counterTrajForLegend = 1;
figure(70); hold on
colorsOfDifferentTrajectories = linspecer(21,'sequential');
b(counterTrajForLegend) = plot(posOpt(1, :), posOpt(2, :), 'Color', colorsOfDifferentTrajectories(counterTrajForLegend, :), 'LineWidth', 4.5, 'DisplayName', sprintf('Trajectory Optimal'));
xlabel("x[m]"), ylabel('y[m]'), grid minor
title('Trajectory Variation for each epoch'), fontsize(fontSize, "points")
legend('show');
drawnow;
legend(b(1:counterTrajForLegend))
per_params = zeros(2,20);
err_vec = zeros(4, 20);

e_nopt=zeros(3,20); e_opt=zeros(3,20);
for i=1:20
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
        b(counterTrajForLegend) = plot(q_OPT_PERT(1, :), q_OPT_PERT(2, :), 'Color', colorsOfDifferentTrajectories(counterTrajForLegend, :), 'LineWidth', linewidth, 'LineStyle', '-.', 'DisplayName', sprintf('Trajectory n: %d', counterTrajForLegend));
        xlabel("x[m]"), ylabel('y[m]'), grid minor
        title('Trajectory Variation for each epoch'), fontsize(fontSize, "points")
        legend('show');
        drawnow;
        %legend(b(1:counterTrajForLegend))
    end
    if mod(i, 2) ~= 0
        b(counterTrajForLegend) = plot(q_OPT_PERT(1, :), q_OPT_PERT((2), :), 'Color', colorsOfDifferentTrajectories(counterTrajForLegend, :), 'LineWidth', linewidth, 'DisplayName', sprintf('Trajectory n: %d', counterTrajForLegend));
        xlabel("x[m]"), ylabel('y[m]'), grid minor
        title('Trajectory Variation for each epoch'), fontsize(fontSize, "points")
        legend('show');
        drawnow;
        %legend(b(1:counterTrajForLegend))
    end
end
hold off

%% GENERATION OF NON - OPTIMAL TRAJECTORY VARYING THE PERTURBED PARAMETERS
bk = zeros(21);
counterTrajForLegend = 1;
figure(65); hold on
colorsOfDifferentTrajectories = linspecer(22,'sequential');
bk(counterTrajForLegend) = plot(posNonOpt(1, :), posNonOpt(2, :), 'Color', colorsOfDifferentTrajectories(counterTrajForLegend, :), 'LineWidth', 4.5, 'DisplayName', sprintf('Trajectory not Optimal'));
xlabel("x[m]"), ylabel('y[m]'), grid minor
title('Trajectory Variation for each epoch'), fontsize(fontSize, "points")
legend('show');
drawnow;
legend(bk(1:counterTrajForLegend))

for j=1:20
   counterTrajForLegend = counterTrajForLegend + 1;

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
        bk(counterTrajForLegend) = plot(q_NOPT_PERT(1, :), q_NOPT_PERT(2, :), 'Color', colorsOfDifferentTrajectories(counterTrajForLegend, :), 'LineWidth', linewidth, 'LineStyle', '-.', 'DisplayName', sprintf('Trajectory n: %d', counterTrajForLegend));
        xlabel("x[m]"), ylabel('y[m]'), grid minor
        title('Trajectory Variation for each epoch'), fontsize(fontSize, "points")
        legend('show');
        drawnow;
        legend(bk(1:counterTrajForLegend))
    end
    if mod(j, 2) ~= 0
        bk(counterTrajForLegend) = plot(q_NOPT_PERT(1, :), q_NOPT_PERT((2), :), 'Color', colorsOfDifferentTrajectories(counterTrajForLegend, :), 'LineWidth', linewidth, 'DisplayName', sprintf('Trajectory n: %d', counterTrajForLegend));
        xlabel("x[m]"), ylabel('y[m]'), grid minor
        title('Trajectory Variation for each epoch'), fontsize(fontSize, "points")
        legend('show');
        drawnow;
        legend(bk(1:counterTrajForLegend))
    end
end
hold off

disp(err_vec)
disp(e_opt)
strg = ['La media del errore nel caso ottimo è :', sprintf('%1.5f',mean(e_opt,'all'))];
disp(strg);
strg = ['La media del errore nel caso NON ottimo è:', sprintf('%1.5f',mean(e_nopt,'all'))];
disp(strg );

%% Videos
b_n = perturbed_params(2);
%video(q_history,r_d,b_n,timeVec,linewidth,delta, 'Non Optimized Trajectory Following')
%% Create and display video animation and plots for the Optimal trajectory.
%video(q_history,opt_traj,b_n,timeVec,linewidth,delta, 'Optimal Trajectory Following')