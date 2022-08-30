%% Initialization 
clear; close; clc;

workspace;
cd( fileparts( matlab.desktop.editor.getActiveFilename ) );                
    
my_fig_config(  'fontsize',  40, ...
                'lineWidth',   5, ...
               'markerSize',  25 );   
global c                                                                   
c  = my_color(); 

%% Calling the data for left and right 

file_left_name  = "/home/baxterplayground/ros_ws/src/newmanlab_code/moses_baxter/results/2022_08_25/20220825_161957/left_imp2_cartesian_impedance_controller_position.mat"; 
file_right_name = "/home/baxterplayground/ros_ws/src/newmanlab_code/moses_baxter/results/2022_08_25/20220825_161957/right_imp2_cartesian_impedance_controller_position.mat" ;


raw_data_left = load( file_left_name );
raw_data_right = load( file_right_name );

%% 3D Plot 

f = figure( ); a = axes( 'parent', f );


plot3( a, raw_data_right.xp( 1, : ), raw_data_right.xp( 2, : ), raw_data_right.xp( 3, : ), 'linewidth', 3, 'color', c.blue )
hold on
plot3( a, raw_data_left.xp(  1, : ), raw_data_left.xp(  2, : ), raw_data_left.xp(  3, : ), 'linewidth', 3, 'color', c.blue )

plot3( a, raw_data_right.xp0( 1, : ), raw_data_right.xp0( 2, : ), raw_data_right.xp0( 3, : ), 'linewidth', 10, 'linestyle', '--', 'color', c.orange )
plot3( a, raw_data_left.xp0( 1, : ), raw_data_left.xp0( 2, : ), raw_data_left.xp0( 3, : ), 'linewidth', 10, 'linestyle', '--', 'color', c.orange )

xlabel( a, 'x [m]' )
ylabel( a, 'y [m]' )
zlabel( a, 'z [m]' )

w = 1.5;
set( a, 'xlim', [-w, w], 'ylim', [-w, w], 'zlim', [-w, w] )
axis equal 

%% Repeatability of Task

% Right-hand side
f = figure( ); a = axes( 'parent', f );
plot( a, raw_data_right.time, raw_data_right.q' )
set( a, 'xlim', [0, max( raw_data_right.time )] ) 
legend( { 's0', 's1', 'e0', 'e1', 'w0', 'w1', 'w2' } ) 
xlabel( 'Time (sec)' ); ylabel( 'Joint Disp. (rad)' );

% Left-hand side
f = figure( ); a = axes( 'parent', f );
plot( a, raw_data_left.time, raw_data_left.q' )
set( a, 'xlim', [0, max( raw_data_left.time )] ) 
legend( { 's0', 's1', 'e0', 'e1', 'w0', 'w1', 'w2' } ) 
xlabel( 'Time (sec)' ); ylabel( 'Joint Disp. (rad)' );

%% Set desired Orientation 

file_left_name = '/home/baxterplayground/ros_ws/src/newmanlab_code/moses_baxter/results/2022_08_29/20220829_210659/left_imp2_cartesian_impedance_controller_rotation_type1.mat';
file_right_name = '/home/baxterplayground/ros_ws/src/newmanlab_code/moses_baxter/results/2022_08_29/20220829_210659/right_imp2_cartesian_impedance_controller_rotation_type1.mat';

raw_data_left = load( file_left_name );
raw_data_right = load( file_right_name );