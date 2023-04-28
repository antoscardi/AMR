close all; clear; clc;
%% HYPERPARAMETERS:
% Nominal parameters of the robot 
% Radius of the wheels [m].
wheelRadius = 0.1;
% Distance between the wheels [m].
wheelDistance = 0.25;

% NOMINAL parameters
nominal_params = [wheelRadius; 
                  wheelDistance];

% PERTURBED parameters
perturbed_params = [wheelRadius*0.8; 
                    wheelDistance*1.2];

% Simulation time [s]
totalTime = 5;
% Control frequency [Hz]
f = 500;
% Time step [s]
delta = 1/f;
% Total number of steps
Nstep = totalTime*f+1;
%Time
timeVec = 0:delta:totalTime;

% Controller gains
kv = 3;
kp = 30; 
ki = 0;

% Initial position and velocity
initialPositionVec = [2 1];
initialVelocityVec = [1 1];

% Position of the two break points
firstBreak = [7 2];
secondBreak = [9 7];

% Final position and velocity.
finalPositionVec = [10 10];
finalVelocityVec = [1 1];

% Create dx and dy vectors
dx = [initialPositionVec(1);
      initialVelocityVec(1);
           0;
           0;
           0;
     firstBreak(1);
           0;
           0;
     secondBreak(1);
           0;
     finalPositionVec(1);
     finalVelocityVec(1)];

dy = [initialPositionVec(2);
      initialVelocityVec(2);
           0;
           0;
           0;
      firstBreak(2);
           0;
           0;
      secondBreak(2);
           0;
      finalPositionVec(2);
      finalVelocityVec(2)]; 

%% UTILITIES
% Clear the persistent variable in plot_function
clear plot_function

% Counter to count how many times the function is called in order to change colors, initialize to 1
counter = 1;

% Set linewidth and colors.
linewidth = 2.5;
colors = linspecer(8,'qualitative');
fontSize = 16;

% SIMPLER LINE COLOR EXAMPLE
N = 6; X = linspace(0,pi*3,1000);
%hold off;
for ii=1:N
Y = sin(X+2*ii*pi/N);
%plot(X,Y,'color',colors(ii,:),'linewidth',linewidth);
%hold on;
end


