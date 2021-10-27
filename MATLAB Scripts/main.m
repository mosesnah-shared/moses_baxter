% [Project]        [M3X] Whip Project
% [Title]          Sample script for animations
% [Author]         Moses C. Nah
% [Creation Date]  Monday, April 12th, 2021
%
% [Emails]         Moses C. Nah   : mosesnah@mit.edu
%% (--) INITIALIZATION

clear all; close all; clc; workspace;

%% (--) Read Data
result_dir = "../results/";
file_name  = result_dir + "0p5rest_to_grasp.txt";

rawData = myTxtParse( file_name );