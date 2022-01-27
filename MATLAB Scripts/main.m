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
file_name  = "20220127_164210.txt";
file_name  = result_dir + file_name;

fid      = fopen( file_name );                                         % Opening the txt file with the name "txtName"
raw_data = struct();



while( ~feof( fid ) )

    
    % First string is time and second string is the name of the variable    
    tline  = fgetl( fid );                                             % Get the txt file
    names  = regexp( tline , '(?<=\[).+?(?=\])', 'match' );                   % Taking out the string inside the bracket (i.e., without the bracket)
    
    % Getting all the values
    values = regexp( tline , '[+-]?([0-9]*[.])?[0-9]+', 'match' );            % Taking out the string inside the bracket (i.e., without the bracket)

        
end

%%



fn = fieldnames( raw_data );

f = figure( );
a = axes( 'parent', f );
hold on

for i = 1 : length( fn )
    if contains( fn{ i }, "right" )
       plot( a, raw_data.( fn{ i } ).time, raw_data.( fn{ i } ).val )
    end
end

%%
hold on
% plot( raw_data.right_e0_q.time, raw_data.right_e0_diff.val )
% plot( raw_data.right_e1_q.time, raw_data.right_e1_diff.val )
% plot( raw_data.right_s0_q.time, raw_data.right_s0_diff.val )
% plot( raw_data.right_s1_q.time, raw_data.right_s1_diff.val )

plot( raw_data.right_e0_q.time, raw_data.right_e0_tau.val )
plot( raw_data.right_e1_q.time, raw_data.right_e1_tau.val )
plot( raw_data.right_s0_q.time, raw_data.right_s0_tau.val )
plot( raw_data.right_s1_q.time, raw_data.right_s1_tau.val )
% plot( raw_data.right_e0_q.time, raw_data.right_e0_tau.val )


% plot( raw_data.right_e0_tau.time, raw_data.right_e0_tau.val )
% plot( raw_data.right_e0_q0.time, raw_data.right_e0_q0.val )
% plot( raw_data.right_e0_tau.time, 0.002 * (raw_data.right_e0_q0.val - raw_data.right_e0_q.val ) )
% legend( 'q', 'tau', 'q0', 'calc' )
