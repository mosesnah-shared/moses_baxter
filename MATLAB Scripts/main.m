% [Project]        [M3X] Whip Project
% [Title]          Sample script for animations
% [Author]         Moses C. Nah
% [Creation Date]  Monday, April 12th, 2021
%
% [Emails]         Moses C. Nah   : mosesnah@mit.edu
%% (--) INITIALIZATION
%% (--) INITIALIZATION

clear all; close all; clc;
workspace;
cd( fileparts( matlab.desktop.editor.getActiveFilename ) );                % Setting the current directory as the folder where this "main.m" script is located
    
my_fig_config(  'fontsize',  20, ...
                'lineWidth',   5, ...
               'markerSize',  25 );   
    
%% (--) Read Data
result_dir = "../results/";

file_name  = "20220128_120904.txt";
%file_name   = "20220128_114431.txt";
% file_name  = "20220128_114225.txt";
%file_name  = "20220128_113902.txt";
%file_name  = "20220127_164210.txt";

file_name  = result_dir + file_name;

fid      = fopen( file_name );                                         % Opening the txt file with the name "txtName"
raw_data = struct();

N = 5000; % Defining the size of the array

raw_data.qo_L.time  = nan( 1,  N );
raw_data.qo_L.value = nan( 7,  N );

raw_data.qo_R.time  = nan( 1,  N );
raw_data.qo_R.value = nan( 7,  N );

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

idx = ones( 1, 8 );
str_idx = [ "tau_R", "tau_L", "dq_R", "dq_L", "q_L", "q_R", "qo_R", "qo_L" ];

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


%% Plotting the ZFT and actual
% Index 2, 4 and 6 are s1, e1, w1, right hand

f = figure; a = axes( 'parent', f );
hold on
title( "shoulder" )
plot(  raw_data.q_R.time,  raw_data.q_R.value( 2, : ), 'parent', a )
plot( raw_data.qo_R.time, raw_data.qo_R.value( 2, : ), 'parent', a )

plot(  raw_data2.q_R.time,  raw_data2.q_R.value( 2, : ), 'parent', a )
plot( raw_data2.qo_R.time, raw_data2.qo_R.value( 2, : ), 'parent', a )


f = figure; a = axes( 'parent', f );
hold on
title( "elbow" )
plot(  raw_data.q_R.time,  raw_data.q_R.value( 4, : ), 'parent', a )
plot( raw_data.qo_R.time, raw_data.qo_R.value( 4, : ), 'parent', a )

plot(  raw_data2.q_R.time,  raw_data2.q_R.value( 4, : ), 'parent', a )
plot( raw_data2.qo_R.time, raw_data2.qo_R.value( 4, : ), 'parent', a )


f = figure; a = axes( 'parent', f );
hold on
title( "wrist" ) 
plot(  raw_data.q_R.time,  raw_data.q_R.value( 6, : ), 'parent', a )
plot( raw_data.qo_R.time, raw_data.qo_R.value( 6, : ), 'parent', a )

plot(  raw_data2.q_R.time,  raw_data2.q_R.value( 6, : ), 'parent', a )
plot( raw_data2.qo_R.time, raw_data2.qo_R.value( 6, : ), 'parent', a )
