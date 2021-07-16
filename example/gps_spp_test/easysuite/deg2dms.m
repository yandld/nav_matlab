function result = deg2dms(deg)
%DEG2DMS  Conversion of degrees to degrees, minutes, and seconds.

% Written by Kai Borre
% February 7, 2001

neg_arg = 'FALSE';
if deg < 0
    neg_arg = 'TRUE ';
    deg = -deg;
end
int_deg = floor(deg);
decimal = deg-int_deg;
min_part = decimal*60;
min = floor(min_part);
sec_part = min_part-floor(min_part);
sec = sec_part*60;
if sec == 60
    min = min+1;
    sec = 0;
end
if min == 60
    int_deg = int_deg+1;
    min = 0;
end  
if strcmp(neg_arg, 'TRUE ')
    int_deg = -int_deg;
end
result = [int_deg min sec];
%%%%%%%%%%%%%%%%%%% end deg2dms.m %%%%%%%%%%%%%%%%