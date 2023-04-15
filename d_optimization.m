close all; clc;
%% In this file we computed all the thing necessary for the optimization
% At first we computed all the matrix necessary for the computation of the
% two sensitivity, with the use of the MatlabFunction created in the file b.
% And then we make the integration of the two sensitivity that we used
% to compute the new coefficients of the optimal trajectory at the end of the file.
tic
%% OPTIMIZATION CYCLE
% Hyperparameters, chosen in this way, to make a scaling to the size we are interested in
k1 = 0.1; k2 = 1; epochs = 10;

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

for n = 2:epochs+1
ax_old = ax_evolution(:,n-1); ay_old = ay_evolution(:,n-1);
newCoeffMAtrix = [ax_old,ay_old];
% Generate NEW trajectory
[r_d,dr_d,ddr_d] = trajectory_generation(newCoeffMAtrix, timeVec, totalTime,...
                                         linewidth, colors, false);

% Running the simulation loop for every new trajectory, ALWAYS NOMINAL CASE
[q_history,u_history,xhi_history,] = simulation_loop(initialPositionVec,initialVelocityVec,...
                                                     totalTime, delta, timeVec,...
                                                     nominal_params, perturbed_params,false,...
                                                     newCoeffMAtrix,r_d,dr_d,ddr_d);

% Sensitivity calculation 
sens_last = sensitivity_integration(Nstep,nominal_params,...
                                    q_history,xhi_history,u_history,...
                                    r_d,dr_d,ddr_d,...
                                    delta);

% Sensitivity_ai calculation, by calling the function


%% Calculate vi for each x and y trajectory's coefficient which is the negative gradient of the cost function
%%%PER ORA LA CALCOLO SOLO CON LA SENSITIVITY RICORDARSI DI CAMBIARLO
%%CAPITOOOO
%%%%CAPITOOOOOOOOOOO
vx = zeros(grado,1);
for i= 1:grado
    %sensai_last = reshape(all_sensaix{i}(1:6,Nstep),2,[])';
    vx(i) = -trace(sens_last'*sens_last);
end

vy = zeros(grado,1);
for i= 1:grado
    %sensai_last = reshape(all_sensaiy{i}(1:6,Nstep),2,[])';
    vy(i) = -trace(sens_last'*sens_last);
end

% Calculate the loss function: as norm the trace of the sensitivity.
Loss(n) = 0.5*trace(sens_last'*sens_last);

% Update law of the optimization
I = eye(grado);
ax_new = ax_old + delta*(k1*pinv(M)*(dx-M*ax_evolution(:,n-1)) + k2*(I - pinv(M)*M)*vx);
ay_new = ay_old + delta*(k1*pinv(M)*(dy-M*ay_evolution(:,n-1)) + k2*(I - pinv(M)*M)*vy);

ax_evolution(:,n) = ax_new; ay_evolution(:,n) = ay_new;

end 

%% Take the optimized trajectory as the last obtained in the optimization epochs:
% One could also take the one that minimizes the loss (if it's not the last)
ax_star = ax_evolution(:,epochs);
ay_star = ay_evolution(:,epochs);
optimizedCoeffMatrix = [ax_star,ay_star];

%% Visualize optimized trajectory
[opt_traj,opt_vel,opt_acc] = trajectory_generation(optimizedCoeffMatrix, timeVec, totalTime,...
                                                   linewidth, colors, true);

% Plot Loss function
figure(5)
plot(1:epochs+1,Loss)
title('Loss Function of a')
xlabel('epochs'); ylabel("Norm of sens at tf");fontsize(fontSize,"points")

%% Save optimized coefficients and new trajectory
save('data/coeff_a_star',"ay_star","ay_star")
save('data/optimized_traj','opt_traj','opt_vel','opt_acc')
toc

%%% QUESTO DA RIFARE DOMANI O LASCIARE COSI
%% FUNZIONE CHE NE FA X E Y INSIEME E LA CHIAMI 15 VOLTE
%% u_ai vectors creation
% which is used to differentiate the whole system with respect to the coefficients a_i
% We use these vectors in the integration of the Sensitivity_ai

% %for coefficients x
% u_ax1 = pagemtimes(h_q,gammax1_int(1:3,:,:)) + pagemtimes(h_xhi,gammax1_int(4:6,:,:)) + h_ax1;
% u_ax2 = pagemtimes(h_q,gammax2_int(1:3,:,:)) + pagemtimes(h_xhi,gammax2_int(4:6,:,:)) + h_ax2;
% u_ax3 = pagemtimes(h_q,gammax3_int(1:3,:,:)) + pagemtimes(h_xhi,gammax3_int(4:6,:,:)) + h_ax3;
% u_ax4 = pagemtimes(h_q,gammax4_int(1:3,:,:)) + pagemtimes(h_xhi,gammax4_int(4:6,:,:)) + h_ax4;
% u_ax5 = pagemtimes(h_q,gammax5_int(1:3,:,:)) + pagemtimes(h_xhi,gammax5_int(4:6,:,:)) + h_ax5;
% 
% u_ax6 = pagemtimes(h_q,gammax6_int(1:3,:,:)) + pagemtimes(h_xhi,gammax6_int(4:6,:,:)) + h_ax1;
% u_ax7 = pagemtimes(h_q,gammax7_int(1:3,:,:)) + pagemtimes(h_xhi,gammax7_int(4:6,:,:)) + h_ax2;
% u_ax8 = pagemtimes(h_q,gammax8_int(1:3,:,:)) + pagemtimes(h_xhi,gammax8_int(4:6,:,:)) + h_ax3;
% u_ax9 = pagemtimes(h_q,gammax9_int(1:3,:,:)) + pagemtimes(h_xhi,gammax9_int(4:6,:,:)) + h_ax4;
% u_ax10 = pagemtimes(h_q,gammax10_int(1:3,:,:)) + pagemtimes(h_xhi,gammax10_int(4:6,:,:)) + h_ax5;
% 
% u_ax11 = pagemtimes(h_q,gammax11_int(1:3,:,:)) + pagemtimes(h_xhi,gammax11_int(4:6,:,:)) + h_ax1;
% u_ax12 = pagemtimes(h_q,gammax12_int(1:3,:,:)) + pagemtimes(h_xhi,gammax12_int(4:6,:,:)) + h_ax2;
% u_ax13 = pagemtimes(h_q,gammax13_int(1:3,:,:)) + pagemtimes(h_xhi,gammax13_int(4:6,:,:)) + h_ax3;
% u_ax14 = pagemtimes(h_q,gammax14_int(1:3,:,:)) + pagemtimes(h_xhi,gammax14_int(4:6,:,:)) + h_ax4;
% u_ax15 = pagemtimes(h_q,gammax15_int(1:3,:,:)) + pagemtimes(h_xhi,gammax15_int(4:6,:,:)) + h_ax5;
% 
% % for coefficients y
% u_ay1 = pagemtimes(h_q,gammay1_int(1:3,:,:)) + pagemtimes(h_xhi,gammay1_int(4:6,:,:)) + h_ay1;
% u_ay2 = pagemtimes(h_q,gammay2_int(1:3,:,:)) + pagemtimes(h_xhi,gammay2_int(4:6,:,:)) + h_ay2;
% u_ay3 = pagemtimes(h_q,gammay3_int(1:3,:,:)) + pagemtimes(h_xhi,gammay3_int(4:6,:,:)) + h_ay3;
% u_ay4 = pagemtimes(h_q,gammay4_int(1:3,:,:)) + pagemtimes(h_xhi,gammay4_int(4:6,:,:)) + h_ay4;
% u_ay5 = pagemtimes(h_q,gammay5_int(1:3,:,:)) + pagemtimes(h_xhi,gammay5_int(4:6,:,:)) + h_ay5;
% 
% u_ay6 = pagemtimes(h_q,gammay6_int(1:3,:,:)) + pagemtimes(h_xhi,gammay6_int(4:6,:,:)) + h_ay1;
% u_ay7 = pagemtimes(h_q,gammay7_int(1:3,:,:)) + pagemtimes(h_xhi,gammay7_int(4:6,:,:)) + h_ay2;
% u_ay8 = pagemtimes(h_q,gammay8_int(1:3,:,:)) + pagemtimes(h_xhi,gammay8_int(4:6,:,:)) + h_ay3;
% u_ay9 = pagemtimes(h_q,gammay9_int(1:3,:,:)) + pagemtimes(h_xhi,gammay9_int(4:6,:,:)) + h_ay4;
% u_ay10 = pagemtimes(h_q,gammay10_int(1:3,:,:)) + pagemtimes(h_xhi,gammay10_int(4:6,:,:)) + h_ay5;
% 
% u_ay11 = pagemtimes(h_q,gammay11_int(1:3,:,:)) + pagemtimes(h_xhi,gammay11_int(4:6,:,:)) + h_ay1;
% u_ay12 = pagemtimes(h_q,gammay12_int(1:3,:,:)) + pagemtimes(h_xhi,gammay12_int(4:6,:,:)) + h_ay2;
% u_ay13 = pagemtimes(h_q,gammay13_int(1:3,:,:)) + pagemtimes(h_xhi,gammay13_int(4:6,:,:)) + h_ay3;
% u_ay14 = pagemtimes(h_q,gammay14_int(1:3,:,:)) + pagemtimes(h_xhi,gammay14_int(4:6,:,:)) + h_ay4;
% u_ay15 = pagemtimes(h_q,gammay15_int(1:3,:,:)) + pagemtimes(h_xhi,gammay15_int(4:6,:,:)) + h_ay5;
