%*********************************************************************
% 
% This function takes a string and checks if it < 80 characters long.
% If so the string is "padded" with zeros until it is 80 characters
% long.  Useful when reading RINEX files.
% 
%*********************************************************************
function [ current_line ] = check_line_length(current_line)

if length(current_line) < 80    
    add_spaces = 80 - length(current_line);
    
    for j = 1 : add_spaces        
        current_line = [ current_line , '0' ];        
    end    
end

% Check if there are any blanks in the data and put a zero there.
current_line = strrep(current_line,' ', '0');