% [Project]        [M3X] Whip Project
% [Title]          Sample script for animations
% [Author]         Moses C. Nah
% [Creation Date]  Monday, April 12th, 2021
%
% [Emails]         Moses C. Nah   : mosesnah@mit.edu
%% (--) INITIALIZATION

clear all; close all; clc;
workspace;
cd( fileparts( matlab.desktop.editor.getActiveFilename ) );                % Setting the current directory as the folder where this "main.m" script is located
    
my_fig_config(  'fontsize',  40, ...
                'lineWidth',   5, ...
               'markerSize',  25 );   
    
%% (--) Read Data

dir_name1 = "../results/";

% Get the subdirectory to read the data
dir_name2 = "2022_08_08/";

% The specific name of the directory to read the data
dir_name3 = "20220808_141406/";

% dir_name = dir_name1 + dir_name2 + dir_name3;
dir_name = "../results/2022_08_08/20220808_154134/";

mat = dir( dir_name + '*.mat'); 

for i = 1:length( mat ) 
    raw_data{ i }= load( dir_name + mat( i ).name ); 
end 

%%
idx = 1;

tmp = raw_data{ idx }.Kq * ( raw_data{ idx }.q0 - raw_data{ idx }.q ) + raw_data{ idx }.Bq * ( raw_data{ idx }.dq0 - raw_data{ idx }.dq )
