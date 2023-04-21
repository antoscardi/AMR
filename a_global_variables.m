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
totalTime = 20;
% Control frequency [Hz]
f = 100;
% Time step [s]
delta = 1/f;
% Total number of steps
Nstep = totalTime*f+1;
%Time
timeVec = 0:delta:totalTime;

% Controller gains
kv = 3;
kp = 28; 
ki = 0;

% Initial position and velocity
initialPositionVec = [5 5];
initialVelocityVec = [0.1 1];

% Position and velocity of the two break points
firstBreak = [7 10];
secondBreak = [12 14];
velocityFirstBreak = [2 0.3];
velocitySecondBreak = [2.1 0.1];

% Final position and velocity.
finalPositionVec = [16 18];
finalVelocityVec = [1.5 2];

% Create dx and dy vectors
dx = [initialPositionVec(1);
      initialVelocityVec(1);
           0;
           0;
     velocityFirstBreak(1);
     firstBreak(1);
           0;
           0;
     secondBreak(1);
     velocitySecondBreak(1);
     finalPositionVec(1);
     finalVelocityVec(1)];

dy = [initialPositionVec(2);
      initialVelocityVec(2);
           0;
           0;
      velocityFirstBreak(2);
      firstBreak(2);
           0;
           0;
      secondBreak(2);
      velocitySecondBreak(2);
      finalPositionVec(2);
      finalVelocityVec(2)]; 

%% UTILITIES
% Clear the persistent variable in plot_function
clear plot_function

% Counter to count how many times the function is called in order to change colors, initialize to 1
counter = 1;

% Set linewidth and colors.
linewidth = 3;
colors = linspecer(12,'qualitative');
fontSize =16;

% % SIMPLER LINE COLOR EXAMPLE
% N = 20; X = linspace(0,pi*3,1000);
% C = linspecer(N,"qualitative")
% hold off;
% for ii=1:N
% Y = sin(X+2*ii*pi/N);
% plot(X,Y,'color',C(ii,:),'linewidth',3);
% hold on;
% end


