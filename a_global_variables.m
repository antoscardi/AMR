close all; clear; clc;
%% HYPERPARAMETERS:
% Nominal parameters of the robot 
% Radius of the wheels [m].
wheelRadius = 1.2;
% Distance between the wheels [m].
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
kp = 28; 
ki = 3;

% Initial position and velocity
initialPositionVec = [5 5];
initialVelocityVec = [1 1];

% Position and velocity of the two break points
firstBreak = [18 20];
secondBreak = [40 42];
velocityFirstBreak = [3 3];
velocitySecondBreak = [3.5 3.5];

% Final position and velocity.
finalPositionVec = [62 63];
finalVelocityVec = [2.5 2.5];

% Clear the persistent variable in plot_function
clear plot_function

% Counter to count how many times the function is called in order to change colors, initialize to 1
counter = 1;

linewidth = 5;
colors = linspecer(12,'sequential');



