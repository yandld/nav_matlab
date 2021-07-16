function tt = check_t(t)
% CHECK_T if necessary, moves t to the interval 
%                 -302400 < t < 302400

%  Written by Kai Borre
%  April 1, 1996

 half_week = 302400;
 tt = t;

 if t >  half_week, tt = t-2*half_week; end
 if t < -half_week, tt = t+2*half_week; end
%%%%%%% end check_t.m  %%%%%%%%%%%%%%%%%
