close all; clear; clc;
% Nominal parameters of the robot 
% Radius of the wheels [m].
setGlobal(0.0993); r_n = getGlobal;
% Distance between the wheels [m].
setGlobal(0.29); b_n = getGlobal;
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

% Generate perturbed values of the parameters, components of vector p
percentage = 0.3;
min = r_n - r_n*percentage; max = r_n + r_n*percentage;
a1 = b_n - b_n*percentage; b1 = b_n + b_n*percentage;
params = zeros(2,Nstep);
for k=1:Nstep
    params(:,k) = [unifrnd(min,max);
                   unifrnd(a1,b1)];
end

% Create data folder
if ~exist('../AMR/data', 'dir')
       mkdir ../AMR data
end

% Add paths to the matlab search path
addpath '../AMR/data', addpath '../AMR/functions'

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
contrast_colors = linspecer(7,'qualitative');
set(groot,'DefaultAxesColorOrder',contrast_colors)

% Latex default for all text
list_factory = fieldnames(get(groot,'factory'));
index_interpreter = find(contains(list_factory,'Interpreter'));
for i = 1:length(index_interpreter)
    default_name = strrep(list_factory{index_interpreter(i)},'factory','default');
    set(groot, default_name,'Latex');
end

%% Helper functions
function var = getGlobal
global x; var = x;
end

function setGlobal(valore)
global x; x = valore;
end
