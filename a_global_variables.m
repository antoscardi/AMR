close all; clear; clc;
%% HYPERPARAMETERS:
% Nominal parameters of the robot 
% Radius of the wheels [m].
setGlobal(0.3); wheelRadious = getGlobal;
% Distance between the wheels [m].
setGlobal(1); wheelDistance = getGlobal;
% Simulation time [s]
setGlobal(20); totalTime = getGlobal;
% Control frequency [Hz]
setGlobal(100); f = getGlobal;
% Time step [s]
setGlobal(1/f); delta = getGlobal;
% Total number of steps
setGlobal(totalTime*f+1); Nstep = getGlobal;
% Time
global timeVector, timeVector = 0:delta:totalTime;
% Controller gains
setGlobal(3); kv = getGlobal;
setGlobal(2); kp = getGlobal; 
setGlobal(1); ki = getGlobal;

% Initial position and velocity
global initialPosition, initialPosition = [2 4];
global initialVelocity, initialVelocity = [0.1 0.1];
% Position of the two break points
global firstBreak, firstBreak = [5 10];
global secondBreak, secondBreak = [7 15];
% Final position and velocity.
global finalPosition, finalPosition = [10 20];
global finalVelocity, finalVelocity = [0.1 0.1];


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
