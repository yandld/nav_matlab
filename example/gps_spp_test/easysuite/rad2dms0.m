function result = rad2dms0(arg)
%RAD2DMS Conversion of radians to degrees, minutes, and seconds

%Kai Borre
%Copyright (c) by Kai Borre
%$Revision 1.1 $  $Date1999/01/12  $

neg_arg = 'FALSE';
if arg < 0
   neg_arg = 'TRUE ';
   arg = -arg;
end

arg = arg*180/pi;
result = zeros(1,3);
result(1) = fix(arg);
if result(1) == 0
   result(2) = fix(arg*60);
else
   result(2) = fix(rem(arg,result(1))*60);
end
result(3) = (arg-result(1)-result(2)/60)*3600;
if strcmp(neg_arg, 'TRUE ')
   result(1) = -result(1);
end
%fprintf('   %3.0f %2.0f %8.6f\n',result(1),result(2),result(3))
%%%%%%%%%%%%%%%%% end rad2dms0.m  %%%%%%%%%%%%%%%%%%%%%%%%%%%
