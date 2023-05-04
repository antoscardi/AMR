close all; clear; clc;
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
addpath ../AMR/my_functions/utility_functions
addpath ../AMR/my_functions/useful_functions

% Set settings for all plots
defaultLinewidth = 1.5;
set(groot,'defaultLineLineWidth', defaultLinewidth)
set(groot,'defaultAxesTitleFontSizeMultiplier',1.5)
set(groot,'defaultfigureposition',[100 100 540 400])
set(groot,'defaultLegendAutoUpdate','off')

% Latex default for all text
list_factory = fieldnames(get(groot,'factory'));
index_interpreter = find(contains(list_factory,'Interpreter'));
for index = 1:length(index_interpreter)
    set(groot, strrep(list_factory{index_interpreter(index)},'factory','default') ,'latex');
end

% Generate colors
contrast_colors = linspecer(6,'qualitative');
set(groot,'DefaultAxesColorOrder',contrast_colors)