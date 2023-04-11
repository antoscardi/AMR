close all; clear; clc;
%% HYPERPARAMETERS:
% Nominal parameters of the robot 
% Radius of the wheels [m].
% Distance between the wheels [m].
% setGlobal(0.2); wheelRadius = getGlobal;
% setGlobal(0.8); wheelDistance = getGlobal;

setGlobal(0.5); wheelRadius = getGlobal;
setGlobal(1.2); wheelDistance = getGlobal;

% Simulation time [s]
setGlobal(20); totalTime = getGlobal;
% Control frequency [Hz]
setGlobal(100); f = getGlobal;
% Time step [s]
setGlobal(1/f); delta = getGlobal;
% Total number of steps
setGlobal(totalTime*f+1); Nstep = getGlobal;
% Time
global timeVec, timeVec = 0:delta:totalTime;

% Controller gains
setGlobal(3); kv = getGlobal;
setGlobal(28); kp = getGlobal; 
setGlobal(3); ki = getGlobal;

% Initial position and velocity
global initialPositionVec, initialPositionVec = [5 5];
global initialVelocityVec, initialVelocityVec = [1 1];
% Position of the two break points
global firstBreak, firstBreak = [18 23];
global secondBreak, secondBreak = [40 45];

% Final position and velocity.
global finalPositionVec, finalPositionVec = [62 66];
global finalVelocityVec, finalVelocityVec = [2.5 2.5];

% Create data folder
if ~exist('../AMR/data', 'dir')
       mkdir ../AMR data
end

% Create functions folder
if ~exist('../AMR/auto_functions', 'dir')
    mkdir ../AMR auto_functions
end

% Add paths to the matlab search path
addpath ../AMR/data
addpath ../AMR/auto_functions
addpath ../AMR/my_functions

% Set settings for all plots
set(groot,'defaultLineLineWidth',5)
global linewidth, linewidth = 5;
set(groot,'defaultAxesTitleFontSizeMultiplier',1.8)
set(groot,'defaultfigureposition',[100 100 540 400])
set(groot,'defaultLegendAutoUpdate','off')

% Clear the persisten variable in plot_function
clear plot_function

% Generate colors
colors = linspecer(12,'sequential');
contrast_colors = linspecer(2,'qualitative');
set(groot,'DefaultAxesColorOrder',contrast_colors)

% Counter to count how many times the function is called in order to change colors, initialize to 1
setGlobal(1); counter = getGlobal;

% Latex default for all text
list_factory = fieldnames(get(groot,'factory'));
index_interpreter = find(contains(list_factory,'Interpreter'));
for index = 1:length(index_interpreter)
    set(groot, strrep(list_factory{index_interpreter(index)},'factory','default') ,'Latex');
end

% % LINE COLORS
% N=5;
% X = linspace(0,pi*3,1000);
% Y = bsxfun(@(x,n)sin(x+2*n*pi/N), X.', 1:N);
% C = linspecer(N,'qualitative');
% axes('NextPlot','replacechildren', 'ColorOrder',C);
% plot(X,Y,'linewidth',5)
% ylim([-1.1 1.1]);

%% Helper functions
function var = getGlobal
global x; var = x;
end

function setGlobal(valore)
global x; x = valore;
end
