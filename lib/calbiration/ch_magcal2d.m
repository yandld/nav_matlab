clear;
clc;
close all;

% %% http://ztrw.mchtr.pw.edu.pl/en/least-squares-circle-fit/
% 
% 
% input = [
%   9.1667   0.5000   1.0000 
%   0.3333   1.8750   1.0000 
%  -7.8083   7.4167   1.0000 
% -10.0167  11.2500   1.0000 
% -15.5583  21.3750   1.0000 
% -16.7500  31.6250   1.0000 
% -13.4333  40.8333   1.0000 
%   4.3917  53.0000   1.0000 
%  15.3500  54.8750   1.0000 
%  21.3083  54.6250   1.0000 
%  32.5417  49.2083   1.0000 
%  33.0417  38.8333   1.0000 
%  32.8750  31.5417   1.0000 
%  34.3083  19.3750   1.0000 
%  25.2917  11.0417   1.0000 
%  16.2500   5.0000   1.0000 
%  11.2083   4.0000   1.0000 
%     ];
% 
% 
% P = input(:,1:2)';
% n = length(P);



clear;
clc;
close all;


P = [1 7; 2 6; 5 8; 7 7; 9 5; 3 7]';
n= length(P);


plot(P(1,:), P(2,:), '*');

%build deisgn matrix
A = [ P(1,:); P(2,:); ones(1,n)]';
b = sum(P.*P, 1)';

% ls solution
a= (A'*A)^(-1)*A'*b;

xc = 0.5*a(1);
yc = 0.5*a(2);
R  =  sqrt((a(1)^2+a(2)^2)/4+a(3));
R

viscircles([xc, yc],R);
axis equal


% max_min_ofs = max(P,[],2) + min(P, [], 2) ;
% max_min_ofs = max_min_ofs / 2;
% max_min_ofs



% %¸ø¶¨³õÖµ
% xc = 5.3794;
% yc = 7.2532;
% r = 3.0370;
% res = [xc; yc; r];
% 
% max_iters = 20;
% 
% max_dif = 10^(-6); % max difference between consecutive results
% for i = 1 : max_iters
%     J = Jacobian(res(1), res(2), P);
%     R = Residual(res(1), res(2), res(3), P);
%     prev = res;
%     res = res - (J'*J)^(-1)*J'*R;
%     dif = abs((prev - res)./res); 
%     if dif < max_dif
%         fprintf('Convergence met after %d iterations\n', i);
%         break;
%     end
% end
% if i == max_iters
%     fprintf('Convergence not reached after %d iterations\n', i);
% end
% 
% xc = res(1);
% yc = res(2);
% r = res(3);
% 
% plot(P(:,1), P(:,2), '*')
%  viscircles([xc, yc],r);
%  axis equal
%  
% function J = Jacobian(xc, yc, P) 
%     len = size(P);
%     r = sqrt((xc - P(:,1)).^2 + (yc - P(:,2)).^2);
%     J = [(xc - P(:,1))./r, (yc - P(:,2))./r, -ones(len(1), 1)];
% end


function R = Residual(xc, yc, r, P)
    R = sqrt((xc - P(:,1)).^2 + (yc - P(:,2)).^2) - r;
end


 
% 
