function raw_data = my_txt_read( file_name )
% myTxtParse for parsing the txt file named "fileName"
%
% =============================================================== %
% [CREATED BY]: Moses C. Nah
% [   DATE   ]: 28-Oct-2021
% =============================================================== %
%
% =============================================================== %
% [DESCRIPTION]
%    - Function for parsing the txt file.
%    - This Function is specifically for the lab computer
% =============================================================== %
%
% =============================================================== %
% [INPUT] 
%   (1) file_name (string)
%       -  The txt file name aimed to be parsed
%          The syntax of a single line of the txt file should be as following:
%          [INFO] [XX] [name] [XX] [time] [XX] [value] [XX]
%          For the given name, save the time and value function.
% =============================================================== %
% [OUTPUT] 
%   (1) raw_data (structure)
%       -  The data name and its actual data. 
%
% =============================================================== %
%
% [REMARKS]  ADD DETAILS
%
% =============================================================== %
%
% =============================================================== %
% SEE ALSO testHelpFunction 
%
% =============================================================== %
    fid      = fopen( file_name );                                         % Opening the txt file with the name "txtName"
    raw_data = struct();
    
    while( ~feof( fid ) )
        
        tline  = fgetl( fid );                                             % Get the txt file
        names  = regexp( tline , '\[(.*?)\]', 'match' );                   % Taking out the string inside the bracket (i.e., without the bracket)
        values = regexp( tline , '(?<=\[).+?(?=\])', 'match' );            % Taking out the string inside the bracket (i.e., without the bracket)

        
                                                                           % [REF1] https://stackoverflow.com/questions/2973436/regex-lookahead-lookbehind-and-atomic-groups
                                                                           % [REF2] https://stackoverflow.com/questions/2403122/regular-expression-to-extract-text-between-square-brackets
                                                                           % (?<=\[) Look behind \[, e.g., (?<=B)A , Find expression A where expression B precedes
                                                                           % (?=\])  Look ahead  \], e.g., A(?=B)  , Find expression A where expression B follows
                                                                           % .+?, a nongreedy way of reading the line
        if isempty( tmp )
            continue
        end
                                                                           
                                                                           % Within the tmp "cell", find the index of "name" and create the structure.
        idx_name = find( contains( tmp, 'name'  ) );
        idx_time = find( contains( tmp, 'time'  ) );
        idx_val  = find( contains( tmp, 'value' ) );
        
        name = tmp{ idx_name + 1 };
        time = str2double( tmp{ idx_time + 1 } );
        val  = str2double( tmp{ idx_val  + 1 } );
        
        
        % If the name is not yet inside the structure
        %                       Getting the corresponding name
        if ~isfield( raw_data, name )                       % If new field, add to raw_data
             raw_data.( name ).( "time" ) = time;
             raw_data.( name ).( "val"  ) = val;
        else    
             raw_data.( name ).( "time" )( end + 1 ) = time;
             raw_data.( name ).( "val"  )( end + 1 ) = val;            
        end
    end

end

