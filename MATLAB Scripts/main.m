% [Project]        [M3X] Whip Project
% [Title]          Sample script for animations
% [Author]         Moses C. Nah
% [Creation Date]  Monday, April 12th, 2021
%
% [Emails]         Moses C. Nah   : mosesnah@mit.edu
%% (--) INITIALIZATION

clear all; close all; clc;
workspace;
cd( fileparts( matlab.desktop.editor.getActiveFilename ) );                
    
my_fig_config(  'fontsize',  40, ...
                'lineWidth',   5, ...
               'markerSize',  25 );   
global c                                                                   
c  = my_color(); 
           
           
%% (--) Read Data

dir_name1 = "../results/";

% Get the subdirectory to read the data
dir_name2 = "2022_08_08/";

% The specific name of the directory to read the data
dir_name3 = "20220808_141406/";

% dir_name = dir_name1 + dir_name2 + dir_name3;
dir_name = "../results/2022_08_10/20220810_182601/";

mat = dir( dir_name + '*.mat'); 

for i = 1:length( mat ) 
    raw_data{ i }= load( dir_name + mat( i ).name ); 
end 

%% Analyze the data
idx = 1; % LEFT 
idx = 2; % RIGHT

data = raw_data{ idx };

N = length( data.time );
[n_act, ~] = size( data.qi );  
q0  = zeros( 7, N, n_act );
dq0 = zeros( 7, N, n_act );

for i = 1: length( data.time )
    for j = 1 : n_act
        [tmp_q, tmp_dq ] = submovement( data.time( i ), data.ti( j ), data.qi( j, : ), data.qf( j, : ) , data.D( j ) );
        q0( :, i, j )  = tmp_q;
        dq0( :, i, j ) = tmp_dq; 
    end
end

f = figure( ); a = axes( 'parent', f );
hold on
plot( data.time, squeeze( dq0( :, :, 1 ) )', 'linestyle',  '-', 'linewidth',  3, 'color', c.black )
plot( data.time, squeeze( dq0( :, :, 2 ) )', 'linestyle',  '-', 'linewidth',  3, 'color', c.blue )

%% Plot Optimization

plot( data_arr', 'o', 'markeredgecolor', 'k', 'markersize', 10 )
hold on

% plot( max( data_arr ), 'color', c.blue )
set( gca, 'xlim', [1,50], 'xtick', [1:5:50,50] )

x = [1 50 50 1 1];
y = [90 90 100 100 90];
tmp = fill( x, y, c.blue, 'FaceAlpha',0.3 );

xlabel( 'Iteration (-)' )
ylabel( 'Coverage (\%)' )