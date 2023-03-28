close all; clear; clc;
%% HYPERPARAMETERS:
% Nominal parameters of the robot 
% Radius of the wheels [m].
setGlobal(0.3); r_n = getGlobal;
% Distance between the wheels [m].
setGlobal(1); b_n = getGlobal;
% Simulation time [s]
setGlobal(20); tsim = getGlobal;
% Control frequency [Hz]
setGlobal(100); f = getGlobal;
% Time step [s]
setGlobal(1/f); delta = getGlobal;
% Total number of steps
setGlobal(tsim*f+1); Nstep = getGlobal;
% Time
global time, time = 0:delta:tsim;
% Controller gains
setGlobal(3); kv = getGlobal;
setGlobal(2); kp = getGlobal; 
setGlobal(1); ki = getGlobal;

% Initial position and velocity
global p_0, p_0 = [2 4];
global v_0, v_0 = [0.1 0.1];
% Position of the two break points
global p_1, p_1 = [5 10];
global p_2, p_2 = [7 15];
% Final position and velocity.
global p_f, p_f = [10 20];
global v_f, v_f = [0.1 0.1];

% Generate perturbed values of the parameters, components of vector p
percentage = 0.2;
params = zeros(2,Nstep);
for k=1:Nstep
    params(:,k) = [unifrnd(r_n - r_n*percentage,r_n + r_n*percentage);
                   unifrnd(b_n - b_n*percentage,b_n + b_n*percentage)];
end

% Create data folder
if ~exist('../AMR/data', 'dir')
       mkdir ../AMR data
       % Add paths to the matlab search path
       addpath ../AMR/data
end

% Create functions folder
if ~exist('../AMR/auto_functions', 'dir')
    mkdir ../AMR auto_functions
    addpath ../AMR/auto_functions
end

% Add the folder with our functions to the Path
addpath ../AMR/my_functions

% Save params 
save('data/params','params');

% Set settings for all plots
set(groot,'defaultLineLineWidth',3)
global linewidth, linewidth = 3;
set(groot,'defaultAxesFontWeight','bold')
set(groot,'defaultLegendFontWeight','bold')
set(groot,'defaultAxesSubtitleFontWeight','bold')
set(groot,'defaultAxesTitleFontWeight','bold')
set(groot,'defaultAxesTitleFontSizeMultiplier',1.3)

% Generate colors
colors = linspecer(12,'sequential');
contrast_colors = linspecer(2,'qualitative');
set(groot,'DefaultAxesColorOrder',contrast_colors)

% Latex default for all text
list_factory = fieldnames(get(groot,'factory'));
index_interpreter = find(contains(list_factory,'Interpreter'));
for index = 1:length(index_interpreter)
    set(groot, strrep(list_factory{index_interpreter(index)},'factory','default') ,'Latex');
end

%% Helper functions
function var = getGlobal
global x; var = x;
end

function setGlobal(valore)
global x; x = valore;
end
