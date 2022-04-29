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
result_dir = "../results/2022_04_28/";
file_name  = "20220428_163643.txt";

file_name  = result_dir + file_name;

fid      = fopen( file_name );                                         % Opening the txt file with the name "txtName"
raw_data = struct();

N = 20000; % Defining the size of the array

raw_data.qo_L.time  = nan( 1,  N );
raw_data.qo_L.value = nan( 7,  N );

raw_data.dqo_L.time  = nan( 1,  N );
raw_data.dqo_L.value = nan( 7,  N );

raw_data.qo_R.time  = nan( 1,  N );
raw_data.qo_R.value = nan( 7,  N );

raw_data.dqo_R.time  = nan( 1,  N );
raw_data.dqo_R.value = nan( 7,  N );


raw_data.q_L.time   = nan( 1,  N );
raw_data.q_L.value  = nan( 7,  N );

raw_data.q_R.time   = nan( 1,  N );
raw_data.q_R.value  = nan( 7,  N );

raw_data.dq_L.time   = nan( 1,  N );
raw_data.dq_L.value  = nan( 7,  N );

raw_data.dq_R.time   = nan( 1,  N );
raw_data.dq_R.value  = nan( 7,  N );

raw_data.tau_L.time  = nan( 1,  N );
raw_data.tau_L.value = nan( 7,  N );

raw_data.tau_R.time  = nan( 1,  N );
raw_data.tau_R.value = nan( 7,  N );

str_idx = [ "tau_R", "tau_L", "dq_R", "dq_L", "q_L", "q_R", "qo_R", "qo_L", "dqo_L", "dqo_R" ];
idx = ones( 1, length( str_idx ) );

%%

while( ~feof( fid ) )

    
    % First string is time and second string is the name of the variable    
    tline  = fgetl( fid );                                             % Get the txt file
    names  = regexp( tline , '(?<=\[).+?(?=\])', 'match' );                   % Taking out the string inside the bracket (i.e., without the bracket)
    
    % Getting all the values
    values = regexp( tline , '[+-]?([0-9]*[.])?[0-9]+', 'match' );            % Taking out the string inside the bracket (i.e., without the bracket)
    values = str2double( values );
    
    tmp = find( str_idx == names{ 2 } );
    
    if any( tmp ) % If field exists
        tmpi = idx( tmp ); 
        raw_data.( names{ 2 } ).time( tmpi )  = values( 1 );
        raw_data.( names{ 2 } ).value( :, tmpi ) = values( 2 : 8 )' ;
        idx( tmp ) = tmpi + 1; 
    else   
        error( "no string in match, the name is %s", names{ 2 } )
    end

    
   
end

% Cleaning up all the NaN inside the matrix 
fn = fieldnames( raw_data );

for k = 1: numel( fn )
    raw_data.( fn{ k } ).time  = raw_data.( fn{ k } ).time(  ~isnan( raw_data.( fn{ k } ).time  ) );
end

% Getting the size of the data based on the time

for k = 1: numel( fn )
    N = length( raw_data.( fn{ k } ).time );
    raw_data.( fn{ k } ).value = raw_data.( fn{ k } ).value( :, 1:N );
end
    

 %% Plotting the ZFT and actual
% Index 2, 4 and 6 are s1, e1, w1, right hand
idx  = 4;
tmax = 8;

f = figure; a = axes( 'parent', f );
hold on
% plot(  raw_data.dq_R.time,  raw_data.dq_R.value( idx, : ), 'o', 'markersize', 10, 'parent', a )
% plot( raw_data.dqo_R.time, raw_data.dqo_R.value( idx, : ), 'o', 'markersize', 10,'parent', a )
plot( raw_data.q_R.time, raw_data.q_R.value( idx, : ), 'o', 'markersize', 10,'parent', a )
plot( raw_data.q_L.time, raw_data.q_L.value( idx, : ), 'o', 'markersize', 10,'parent', a )
xlabel( "Time (sec)" )
ylabel( "Ang. Vel. (rad)" )
% legend( 'q', 'q0' )

t = 0 : 0.01 : tmax;
N = length( t );

D1   = 1.7;
D2   = 1.8;
toff = -0.5;
pi   = [ 0.7869321442,  0.4045874328, -0.0149563127, 1.4116458201, -0.0464029188,  0.3879126465, -1.5823011827 ];
pm   = [ 0.7869321442, -0.5419854693, -0.0149563127, 0.3097428536, -0.0464029188, -0.7596604570, -1.5823011827 ];
pf   = [ 0.7869321442,  0.0881893857, -0.0149563127, 0.7309418454, -0.0464029188, -0.2511893540, -1.5823011827 ];

posvec1 = zeros( 1, N );
velvec1 = zeros( 1, N );

posvec2 = zeros( 1, N );
velvec2 = zeros( 1, N );

for i = 1 : N
    [ posvec1( i ), velvec1( i ) ] = submovement( t( i ),         0, pi( idx ), pm( idx ), D1 );
    [ posvec2( i ), velvec2( i ) ] = submovement( t( i ), D1 + toff, 0, pf( idx ) - pm(idx), D2 );
end
% 
% plot( t, velvec1, 'parent', a )
% plot( t, velvec2, 'parent', a )
% plot( t, velvec1 + velvec2, 'parent', a )


plot( t, posvec1 + posvec2, 'parent', a )

set( a, 'xlim', [ 0, 8 ])


%%
% 
% 
% f = figure; a = axes( 'parent', f );
% hold on
% title( "elbow" )
% plot(  raw_data.q_R.time,  raw_data.q_R.value( 4, : ), 'parent', a )
% plot( raw_data.qo_R.time, raw_data.qo_R.value( 4, : ), 'parent', a )


f = figure; a = axes( 'parent', f );
hold on
title( "wrist" ) 
% Ni = 1100;

% for i = 1 : 7

% plot(  raw_data.q_L.time( Ni : end ),  raw_data.dq_L.value( 6, Ni:end ), 'parent', a , 'color', [0.8500 0.3250 0.0980] )
% plot(  raw_data.q_L.time( Ni : end ),  raw_data.tau_L.value( 6, Ni:end ), 'parent', a, 'color', [0.4940 0.1840 0.5560] )

% end
% set( a, 'xlim', [ raw_data.q_L.time( Ni ), max( raw_data.q_L.time ) ] )
set( a, 'fontsize', 40)


legend( "dq", "tau", "location", "northwest", 'fontsize', 50  )
xlabel( "Time (sec)", 'fontsize', 50 ); 
ylabel( "Angle (rad)", 'fontsize', 50 );
% legend( "s0", "s1", "e0", "e1", "w0", "w1", "w2", "location", "northeastoutside"  )


%% (--) Optimization Data analysis

result_dir = "../results/optimization/";

file_name  = "36x42.txt";

file_name  = result_dir + file_name;

fid      = fopen( file_name );                                         % Opening the txt file with the name "txtName"
raw_data = struct();
n = 500;
raw_data.iter   = nan( 1, n );
raw_data.par1   = nan( 1, n );
raw_data.par2   = nan( 1, n );
raw_data.par3   = nan( 1, n );
raw_data.par4   = nan( 1, n );
raw_data.output = nan( 1, n );


i = 1;
while( ~feof( fid ) )

    
    % First string is time and second string is the name of the variable    
    tline  = fgetl( fid );                                             % Get the txt file
    
    % Getting all the values
    values = regexp( tline , '[+-]?([0-9]*[.])?[0-9]+', 'match' );            % Taking out the string inside the bracket (i.e., without the bracket)
    values = str2double( values );
    
    raw_data.iter( i ) = values( 1 ) + 1;
    raw_data.par1( i ) = values( 2 );
    raw_data.par2( i ) = values( 3 );
    raw_data.par3( i ) = values( 4 );
    raw_data.par4( i ) = values( 5 );
    
    raw_data.output( i ) = values( 6 );
    
    i = i + 1;
end

%% (--) Optimization Data analysis 2

result_dir = "../results/2022_03_12_green_opt/";
file_name  = "trial1.txt";


file_name  = result_dir + file_name;

fid      = fopen( file_name );                                         % Opening the txt file with the name "txtName"
raw_data = struct();
n = 500;
raw_data.iter   = nan( 1, n );
raw_data.par1   = nan( 1, n );
raw_data.par2   = nan( 1, n );
raw_data.par3   = nan( 1, n );
raw_data.par4   = nan( 1, n );
raw_data.par5   = nan( 1, n );
raw_data.par6   = nan( 1, n );
raw_data.output = nan( 1, n );


i = 1;
while( ~feof( fid ) )

    
    % First string is time and second string is the name of the variable    
    tline  = fgetl( fid );                                             % Get the txt file
    
    % Getting all the values
    values = regexp( tline , '[+-]?([0-9]*[.])?[0-9]+', 'match' );            % Taking out the string inside the bracket (i.e., without the bracket)
    values = str2double( values );
    
    raw_data.iter( i ) = values( 1 ) + 1;
    raw_data.par1( i ) = values( 2 );
    raw_data.par2( i ) = values( 3 );
    raw_data.par3( i ) = values( 4 );
    raw_data.par4( i ) = values( 5 );
    
    
    raw_data.output( i ) = values( 8 );
    
    i = i + 1;
end


plot( raw_data.iter, raw_data.output, 'linewidth', 10 )
xlabel( 'Iteration (-)', 'fontsize', 35 );
ylabel( 'Coverage (\%)', 'fontsize', 35 )
set( gca, 'xlim', [1, max( raw_data.iter ) ], 'fontsize', 40 )