close all; clear; clc;
%% HYPERPARAMETERS:
% Nominal parameters of the robot 
% Radius of the wheels [m].
% Distance between the wheels [m].
wheelRadius = 1.2;
wheelDistance = 2.2;

% Simulation time [s]
totalTime = 20;
% Control frequency [Hz]
f = 100;
% Time step [s]
delta = 1/f;
% Total number of steps
Nstep = totalTime*f+1;
% Time
timeVec = 0:delta:totalTime;

% Controller gains
kv = 3;
kp = 10; 
ki = 0;

% Initial position and velocity
initialPositionVec = [5 5];
initialVelocityVec = [1 1];
% Position of the two break points
firstBreak = [18 20];
secondBreak = [40 42];

% Final position and velocity.
finalPositionVec = [62 63];
finalVelocityVec = [2.5 2.5];

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
linewidth = 5;
set(groot,'defaultAxesTitleFontSizeMultiplier',1.8)
set(groot,'defaultfigureposition',[100 100 540 400])
set(groot,'defaultLegendAutoUpdate','off')

% Clear the persistent variable in plot_function
clear plot_function

% Generate colors
colors = linspecer(12,'sequential');
contrast_colors = linspecer(2,'qualitative');
set(groot,'DefaultAxesColorOrder',contrast_colors)

% Counter to count how many times the function is called in order to change colors, initialize to 1
counter = 1;

% Latex default for all text
list_factory = fieldnames(get(groot,'factory'));
index_interpreter = find(contains(list_factory,'Interpreter'));
for index = 1:length(index_interpreter)
    set(groot, strrep(list_factory{index_interpreter(index)},'factory','default') ,'Latex');
end


