 function [s_acc, s_gyr, s_mag] = imu_data_purify (acc, gyr, mag, acc_thr, gyr_thr)
 
 n = length(acc);
 m = 1;
 k = 1;
 
    % find static point
     for i = 1:n
         if(abs(norm(acc(i, :)) - 1.0) < acc_thr && norm(gyr(i, :)) < gyr_thr)
             s_acc(m, :) = acc(i, :);
             s_gyr(m, :) = gyr(i, :);
             s_mag(m, :) = mag(i, :);
             m = m+1;
         end
     end
     
%      % purify similar point
%      for i = 1 : m-2
%          if(norm(t_acc(i, :) - t_acc(i+1, :)) > 0.2)
%              
%              s_acc(k, :) = t_acc(i, :);
%              s_gyr(k, :) = t_gyr(i, :);
%              s_mag(k, :) = t_mag(i, :);
%              k = k + 1;
%          end
%      end
     
 end
 