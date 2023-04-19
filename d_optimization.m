close all; clc;
%% In this file we computed all the thing necessary for the optimization
% At first we computed all the matrix necessary for the computation of the
% two sensitivity, with the use of the MatlabFunction created in the file b.
% And then we make the integration of the two sensitivity that we used
% to compute the new coefficients of the optimal trajectory at the end of the file.
tic
%% OPTIMIZATION CYCLE
% Hyperparameters, chosen in this way, to make a scaling to the size we are interested in
%   k1 = 0.1; k2 = 2.5; epochs = 16; % Loss aumenta ma E_OPT = 0.020
%   k1 = 0.1; k2 = 3; epochs = 8/10; Funziona
%   k1 = 0.5; k2 = 5; epochs = 3; % e' uguale alla situazione precedente
%   k1 = 0.1; k2 = 3; epochs = 9; % Tutto okay

k1 = 0.5; k2 = 5; epochs = 2;
% Initialize loss function
Loss = zeros(1,epochs);

% Generate initial trajectory
[aMatrix, M] = coeff_generation(totalTime,dx,dy);
initial_ax = aMatrix(:,1);
initial_ay = aMatrix(:,2);

% Get the number of rows 
[grado, ~] = size(aMatrix);

% Initialize a vector of the evolution of the parameters
ax_evolution = zeros(grado,epochs); ax_evolution(:,1)= initial_ax;
ay_evolution = zeros(grado,epochs); ay_evolution(:,1)= initial_ay;

colorsOfDifferentTrajectories = linspecer(epochs,"qualitative");
counterColorTrajectory = 1;
b=zeros(epochs);
figure(15); hold on
sensitivityArrayEpochs = cell(epochs,1);
for n = 1:epochs
ax_old = ax_evolution(:,n); ay_old = ay_evolution(:,n);
newCoeffMatrix = [ax_old,ay_old];
% Generate NEW trajectory
[r_d,dr_d,ddr_d] = trajectory_generation(newCoeffMatrix, timeVec, totalTime,...
                                         linewidth, colors, false);

% Running the simulation loop for every new trajectory, ALWAYS NOMINAL CASE
[q_history,u_history,xhi_history,] = simulation_loop(initialPositionVec,initialVelocityVec,...
                                                     delta,...
                                                     nominal_params, perturbed_params,false,...
                                                     r_d,dr_d,ddr_d);

% Sensitivity calculation 
[sens_last, sens_hist] = sensitivity_integration(Nstep,nominal_params,...
                                    q_history,xhi_history,u_history,...
                                    r_d,dr_d,ddr_d,...
                                    delta);
% Saving the sensivity for the plot
sensitivityArrayEpochs{n} = sens_hist;

% Sensitivity_ai calculation, by calling the function gamma_integration
sens_ai_Array = sensitivity_ai_integration_through_gamma(sens_hist,newCoeffMatrix,...
                                                        nominal_params,timeVec,...
                                                        q_history,xhi_history,u_history,...
                                                        r_d,dr_d,ddr_d,...
                                                        delta,Nstep);
%% Calculate vi for each x and y trajectory's coefficient which is the negative gradient of the cost function
vx = zeros(grado,1);
for i= 1:grado
    sensai_last = reshape(sens_ai_Array{i,1}(1:6,Nstep),2,[])';
    vx(i) = -trace(sens_last'*sensai_last);
end

vy = zeros(grado,1);
for i= 1:grado
    sensai_last = reshape(sens_ai_Array{i,2}(1:6,Nstep),2,[])';
    vy(i) = -trace(sens_last'*sensai_last);
end

% Calculate the loss function: as norm the trace of the sensitivity.
Loss(n) = 0.5*trace(sens_last'*sens_last);

% Update law of the optimization
I = eye(grado);
ax_new = ax_old + delta*(k1*pinv(M)*(dx-M*ax_evolution(:,n)) + k2*(I - pinv(M)*M)*vx);
ay_new = ay_old + delta*(k1*pinv(M)*(dy-M*ay_evolution(:,n)) + k2*(I - pinv(M)*M)*vy);

ax_evolution(:,n+1) = ax_new; ay_evolution(:,n+1) = ay_new;

if counterColorTrajectory <= epochs && mod(n,2) == 0
    b(counterColorTrajectory) = plot(r_d(1,:),r_d((2),:),'Color',colorsOfDifferentTrajectories(counterColorTrajectory,:),'LineWidth',linewidth, 'LineStyle', '-.', 'DisplayName', sprintf('Trajectory n: %d',counterColorTrajectory));
    xlabel("x[m]"), ylabel('y[m]'), grid minor
    title('Trajectory Variation for each epoch'),fontsize(fontSize,"points")
    %legend('-DynamicLegend');
    legend('show');
    drawnow;
    legend(b(1:counterColorTrajectory))
end
if counterColorTrajectory <= epochs && mod(n,2) ~= 0
    b(counterColorTrajectory) = plot(r_d(1,:),r_d((2),:),'Color',colorsOfDifferentTrajectories(counterColorTrajectory,:),'LineWidth',linewidth, 'DisplayName',sprintf('Trajectory n: %d',counterColorTrajectory));
    xlabel("x[m]"), ylabel('y[m]'), grid minor
    title('Trajectory Variation for each epoch'),fontsize(fontSize,"points")
    %legend('-DynamicLegend');
    legend('show');
    drawnow;
    legend(b(1:counterColorTrajectory))
end
counterColorTrajectory = counterColorTrajectory + 1;
end 
hold off
%% Take the optimized trajectory as the last obtained in the optimization epochs:
% One could also take the one that minimizes the loss (if it's not the last)
ax_star = ax_evolution(:,epochs);
ay_star = ay_evolution(:,epochs);
optimizedCoeffMatrix = [ax_star,ay_star];

%% Visualize optimized trajectory
[opt_traj,opt_vel,opt_acc] = trajectory_generation(optimizedCoeffMatrix, timeVec, totalTime,...
                                                   linewidth, colors, true);

% Plot Loss function
figure(5); hold on
plot(1:epochs,Loss)
title('Loss Function of a')
xlabel('epochs'); ylabel("Norm of sens at tf");fontsize(fontSize,"points")
hold off

% Plot Sensitivity
figure(17); hold on
c=zeros(12,epochs);
counterColorSens = 1;
colorsOfDifferentSensitivities= linspecer(epochs,"qualitative");
for i=1:epochs
    %plot(timeVec,sensitivityArrayEpochs{i}')
    %hold on
    if counterColorSens <= epochs && i == 1
        c(:,i) = plot(timeVec,sensitivityArrayEpochs{i}','Color',colorsOfDifferentSensitivities(counterColorSens,:),'LineWidth',linewidth, 'LineStyle', '-', 'DisplayName', sprintf('Sensivity n: %d',counterColorSens));
        %xlabel("x[m]"), ylabel('y[m]'), grid minor
        title('Sensitivity evolution over time and over epochs')
        xlabel('epochs'); ylabel("Sensitivity over time");fontsize(fontSize,"points")
        %legend('-DynamicLegend');
        legend('show');
        drawnow;
        legend(c(:,counterColorSens))
    end
    if counterColorSens <= epochs && i == 2
        c(:,i) = plot(timeVec,sensitivityArrayEpochs{i}','Color',colorsOfDifferentSensitivities(counterColorSens,:),'LineWidth',linewidth, 'LineStyle', '-.', 'DisplayName', sprintf('Sensivity n: %d',counterColorSens));
        %plot(timeVec,sensitivityArrayEpochs{i}','Color',colorsOfDifferentSensitivities(counterColorSens,:),'LineWidth',linewidth, 'LineStyle', '-.', 'DisplayName', sprintf('Sensivity n: %d',counterColorSens));
        %xlabel("x[m]"), ylabel('y[m]'), grid minor
        title('Sensitivity evolution over time and over epochs')
        xlabel('epochs'); ylabel("Sensitivity over time");fontsize(fontSize,"points")
        %legend('-DynamicLegend');
        legend('show');
        drawnow;
        legend(c(:,counterColorSens))
    end
    if counterColorSens <= epochs && i == 3
        c(:,i) = plot(timeVec,sensitivityArrayEpochs{i}','Color',colorsOfDifferentSensitivities(counterColorSens,:),'LineWidth',linewidth, 'LineStyle', ':', 'DisplayName', sprintf('Sensivity n: %d',counterColorSens));
        %plot(timeVec,sensitivityArrayEpochs{i}','Color',colorsOfDifferentSensitivities(counterColorSens,:),'LineWidth',linewidth, 'LineStyle', ':', 'DisplayName', sprintf('Sensivity n: %d',counterColorSens));
        %xlabel("x[m]"), ylabel('y[m]'), grid minor
        title('Sensitivity evolution over time and over epochs')
        xlabel('epochs'); ylabel("Sensitivity over time");fontsize(fontSize,"points")
        %legend('-DynamicLegend');
        legend('show');
        drawnow;
        legend(c(:,counterColorSens))
    end
    if counterColorSens <= epochs && i == 4
        c(:,i) = plot(timeVec,sensitivityArrayEpochs{i}','Color',colorsOfDifferentSensitivities(counterColorSens,:),'LineWidth',linewidth, 'LineStyle', '--', 'DisplayName', sprintf('Sensivity n: %d',counterColorSens));
        %plot(timeVec,sensitivityArrayEpochs{i}','Color',colorsOfDifferentSensitivities(counterColorSens,:),'LineWidth',linewidth, 'LineStyle', '--', 'DisplayName', sprintf('Sensivity n: %d',counterColorSens));
        %xlabel("x[m]"), ylabel('y[m]'), grid minor
        title('Sensitivity evolution over time and over epochs')
        xlabel('epochs'); ylabel("Sensitivity over time");fontsize(fontSize,"points")
        %legend('-DynamicLegend');
        legend('show');
        drawnow;
        legend(c(:,counterColorSens))
    end
    counterColorSens = counterColorSens + 1;
end
% title('Sensitivity evolution over time and over epochs')
% xlabel('epochs'); ylabel("Sensitivity over time");fontsize(fontSize,"points")

%% Save optimized coefficients and new trajectory
save('data/coeff_a_star',"ax_star","ay_star")
save('data/optimized_traj','opt_traj','opt_vel','opt_acc')
toc

