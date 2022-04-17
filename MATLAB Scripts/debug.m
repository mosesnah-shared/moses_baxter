% [Project]        [M3X] Whip Project
% [Title]          Debug Code
% [Author]         Moses C. Nah
% [Creation Date]  Monday, April 16th, 2022
%
% [Emails]         Moses C. Nah   : mosesnah@mit.edu
%% (--) INITIALIZATION

clear all; close all; clc;
workspace;
cd( fileparts( matlab.desktop.editor.getActiveFilename ) );                % Setting the current directory as the folder where this "main.m" script is located
    
my_fig_config(  'fontsize',  20, ...
                'lineWidth',   5, ...
               'markerSize',  25 );   
           
%% (--) Read txt file

fid   = fopen( 'tmp.txt' );

rawdata = struct( 'time', []);  

limb_name  = [ "right", "left" ];
joint_name = [ "s0", "s1", "e0", "e1", "w0", "w1", "w2" ];
var_name   = [ "q", "q0", "dq", "dq0", "tau" ];

for i = 1 : length( limb_name )
    for j = 1 : length( joint_name )
        for z = 1 : length( var_name)
            tmpstr = join( [ limb_name( i ), "_", joint_name( j ), "_", var_name( z )  ], '' );
            
            rawdata.( tmpstr ) = [] ;
        
        end
    end
end

         
            
while ~feof(fid)
  line = fgetl( fid ); 
  
  tmp = split( line, ' ' );
  
  if length( tmp ) == 4
      num = str2num( tmp{ end } );
      rawdata.( [ tmp{ 2 }, '_', tmp{ 3 } ] )( end + 1 ) = num;
        
  else
      num = str2num( tmp{ end } );
      rawdata.( tmp{ 1 } )( end + 1 ) = num;
  end
  
  
 end
  

fclose(fid);           

%%

hold on
% plot( rawdata.time, rawdata.right_s0_q0 )
% plot( rawdata.time, rawdata.right_s1_q0 )
% plot( rawdata.time, rawdata.right_e0_q0 )
% plot( rawdata.time, rawdata.right_e1_q0 )
% plot( rawdata.time, rawdata.right_w0_q0 )
% plot( rawdata.time, rawdata.right_w1_q0 )
% plot( rawdata.time, rawdata.right_w2_q0 )

plot( rawdata.time, rawdata.left_s0_tau )
plot( rawdata.time, rawdata.left_s1_tau )
plot( rawdata.time, rawdata.left_e0_tau )
plot( rawdata.time, rawdata.left_e1_tau )
plot( rawdata.time, rawdata.left_w0_tau )
plot( rawdata.time, rawdata.left_w1_tau )
plot( rawdata.time, rawdata.left_w2_tau )

% 
% plot( rawdata.time, rawdata.right_s0_q )
% plot( rawdata.time, rawdata.right_s1_q )
% plot( rawdata.time, rawdata.right_e0_q )
% plot( rawdata.time, rawdata.right_e1_q )
% plot( rawdata.time, rawdata.right_w0_q )
% plot( rawdata.time, rawdata.right_w1_q )
% plot( rawdata.time, rawdata.right_w2_q )
% 

% plot( rawdata.time, rawdata.right_s0_dq )
% plot( rawdata.time, rawdata.right_s1_dq )
% plot( rawdata.time, rawdata.right_e0_dq )
% plot( rawdata.time, rawdata.right_e1_dq )
% plot( rawdata.time, rawdata.right_w0_dq )
% plot( rawdata.time, rawdata.right_w1_dq )
% plot( rawdata.time, rawdata.right_w2_dq )
% 
% plot( rawdata.time, rawdata.left_s0_q )
% plot( rawdata.time, rawdata.left_s1_q )
% plot( rawdata.time, rawdata.left_e0_q )
% plot( rawdata.time, rawdata.left_e1_q )
% plot( rawdata.time, rawdata.left_w0_q )
% plot( rawdata.time, rawdata.left_w1_q )
% plot( rawdata.time, rawdata.left_w2_q )

% plot( rawdata.time, rawdata.left_s0_dq )
% plot( rawdata.time, rawdata.left_s1_dq )
% plot( rawdata.time, rawdata.left_e0_dq )
% plot( rawdata.time, rawdata.left_e1_dq )
% plot( rawdata.time, rawdata.left_w0_dq )
% plot( rawdata.time, rawdata.left_w1_dq )
% plot( rawdata.time, rawdata.left_w2_dq )