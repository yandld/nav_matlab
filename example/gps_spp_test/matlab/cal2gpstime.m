%*******************************************************
% DESCRIPTION:
%     This function converts calendar date/time
%     to GPS week/time.
%  
% ARGUMENTS:
%     Either 1) a single n x 6 matrix or
%            2) six separate variables that are
%               equal dimensioned vectors
% 
%     Representing:
%     year - Two digit calendar year representing the 
%            range 1980-2079
%            (e.g., 99 = 1999 and 01 = 2001).
%     month - calendar month, must be in range 1-12.
%     day - calendar day, must be in range 1-31.
%     hour - hour (UTC), must be in range 0-24.
%     min - minutes (UTC), must be in range 0-59.
%     sec - seconds (UTC), must be in range 0-59.
% 
%     [ Note:  all arguments must be integers! ]
% 
% OUTPUT:
%     gps_week - integer GPS week (does not take 
%              "rollover" into account).
%     gps_seconds - integer seconds elapsed in gps_week.
%
% REFERENCE:
% 		ASEN 5090 class notes (Spring 2003).
%
% MODIFICATIONS:    
%       03-14-06  :  Jan Weiss - Original/modified 
%                                from old code. 
% 
% Colorado Center for Astrodynamics Research
% Copyright 2006 University of Colorado, Boulder
%*******************************************************
function [ gps_week, gps_seconds ] = cal2gpstime(varargin)

% Unpack
if nargin == 1
    cal_time = varargin{1};
    year = cal_time(:,1);
    month = cal_time(:,2);
    day = cal_time(:,3);
    hour = cal_time(:,4);
    min = cal_time(:,5);
    sec = cal_time(:,6);  
    clear cal_time
else
    year = varargin{1};
    month = varargin{2};
    day = varargin{3};
    hour = varargin{4};
    min = varargin{5};
    sec = varargin{6};
end

% Seconds in one week
secs_per_week = 604800;

% Converts the two digit year to a four digit year.
% Two digit year represents a year in the range 1980-2079.
if (year >= 80 & year <= 99)
    year = 1900 + year;
end

if (year >= 0 & year <= 79)
    year = 2000 + year;
end

% Calculates the 'm' term used below from the given calendar month.
if (month <= 2)
    y = year - 1;
    m = month + 12;
end

if (month > 2)
    y = year;
    m = month;
end

% Computes the Julian date corresponding to the given calendar date.
JD = floor( (365.25 * y) ) + floor( (30.6001 * (m+1)) ) + ...
    day + ( (hour + min / 60 + sec / 3600) / 24 ) + 1720981.5;

% Computes the GPS week corresponding to the given calendar date.
gps_week = floor( (JD - 2444244.5) / 7 );

% Computes the GPS seconds corresponding to the given calendar date.
gps_seconds=round(((((JD-2444244.5)/7)-gps_week)*secs_per_week)/0.5)*0.5;

