function varargout = parsef(input, format)
%parsef   parse string value using FORTRAN formatting codes
%   [val1,val2,...valn] = parsef(input, format)
%   input is string input value
%   format is cell array of format codes

global input_
input_ = input;
varargout = getvals(1, format, 1);
clear global input_
return

% this function does the work. you probably don't want to go here
function [output, idx] = getvals(idx, format, reps)
global input_
count = 1;
output = {};
for k = 1:reps
	odx = 1;
	for i = 1:length(format)
		fmt = format{i};
		switch class(fmt)
		case 'double'
			count = fmt;
		case 'char'
			type = fmt(1);
			if type == 'X'
				idx = idx+count;
			else
				[len,cnt] = sscanf(fmt,'%*c%d',1);
				if cnt ~= 1
					error(['Invalid format specifier: ''',fmt,'''']);
				end
				switch type
				case {'I','i'}
					for j = 1:count
						[val,cnt] = sscanf(input_(idx:min(idx+len-1,end)),'%d',1);
						if cnt == 1
							output{odx}(j,k) = val;
						else
							output{odx}(j,k) = NaN;
						end
						idx = idx+len;
					end
				case {'F','f'}
					for j = 1:count
						[val,cnt] = sscanf(input_(idx:min(idx+len-1,end)),'%f',1);
						if cnt == 1
							output{odx}(j,k) = val;
						else
							output{odx}(j,k) = NaN;
						end
						idx = idx+len;
					end
				case {'E','D','G'}
					for j = 1:count
						[val,cnt] = sscanf(input_(idx:min(idx+len-1,end)),'%f%*1[DdEe]%f',2);
						if cnt == 2
							output{odx}(j,k) = val(1) * 10^val(2); %#ok<AGROW>
						elseif cnt == 1
							output{odx}(j,k) = val;
						else
							output{odx}(j,k) = NaN;
						end
						idx = idx+len;
					end
				case 'A'
					for j = 1:count
						output{odx}{j,k} = input_(idx:min(idx+len-1,end));
						idx = idx+len;
					end
				otherwise
					error(['Invalid format specifier: ''',fmt,'''']);
				end
				odx = odx+1;
			end
			count = 1;
		case 'cell'
 			[val, idx] = getvals(idx, fmt, count);
			if length(val) == 1
				output(odx) = val;
			else
				output{odx} = val;
			end
			odx = odx+1;
			count = 1;
		end
	end
end
return

