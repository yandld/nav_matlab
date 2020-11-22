% syms  b1 b2 b3 real
% syms  g1 g2 g3 real
% syms  v1 v2 v3 real
% syms  lambda real
% syms h
% syms  l11 l12 l13 l21 l22 l23 l31 l32 l33 real
% L = [l11 l23 l13; l21 l22 l23; l31 l32 l33];
% B = [b1 b2 b3]';
% V = [v1 v2 v3]';
% G = [g1 g2 g3]';
% 
% f = V'*L'*L*V - 2*V'*L'*L*B + B'*L'*L*B - h^2 
% j = jacobian(f, [l11 l12 l13 l21 l22 l23 l31 l32 l33 b1 b2 b3 lamda]);
% collect(j(1,1), [l11]) 
% 
% alp = L*(V - B);
% beta = V-B;
% 2*alp(1)*beta(1);
% collect(ans,[l11])

 function [mis, bias, lamda, inter, residual] = dip13 (acc, mag, mag_norm, epsilon)
%函数的功能：using DIP(dual inner product mehold to caluate mag)
%函数的描述：  Calibration of tri-axial magnetometer using vector observations and  inner products    or       Calibration and Alignment of Tri-Axial Magnetometers for Attitude Determination
%函数的使用：[mis, bias, lamda, inter, J] = dip (acc, mag, mag_norm, epsilon)
%输入：
%     input1: acc: raw acc
%     input2: mag: raw mag

%输出：
%     mis: misalign matrix
%     bias: bias
%     lamda: |reference_vector| * |mag_vector| * cos(theta)
%     inter: interation
%    J: cost function array

    mis = eye(3);
    bias = zeros(1,3);
    lamda = cos(deg2rad(60));
    
    mu = 0.001; % damp
    inter = 100;
    last_e = 100;
    last_J = 100;
   
    % [l11 l12 l13 l21 l22 l23 l31 l32 l33 b1 b2 b3 lamda], lamba = cos(inclincation)
    x = [1 0 0 0 1 0 0 0 1 0 0 0 1];
    
    % using max-min mehold to get a inital bias
   x(10:12) =  (max(mag) + min(mag)) / 2;
    
     [last_J, ~, ~] = lm_dip(x, mag, acc, mag_norm);
     
    for i = 1:inter
        [residual(i), Jacobi, e] = lm_dip(x, mag, acc, mag_norm);
         x = x - (inv((Jacobi' * Jacobi + mu * eye(length(x)))) * Jacobi' * e)';
        %x = x - mu*(Jacobi' * e)';

        if(residual(i) <= last_J)
            mu = 0.1 * mu;
        else
            mu = 10 * mu;
        end
        
       if((abs(norm(e) - norm(last_e))) < epsilon)
             mis =[x(1:3); x(4:6); x(7:9)];
             bias = x(10:12)';
             lamda = x(13);
             inter = i;
           break;
       end
       
       last_e  = e;
       last_J = residual(i);
    end
    
end
 
%% Function
% Jval:  error
% gradient :  gradient of cost func
% e:  Fx

function[residual, gradient, e]=lm_dip(x, mB, gB, h)
n = length(mB);
e = zeros(n*2, 1);
gradient = zeros(n*2, 13);
 L =[x(1:3); x(4:6); x(7:9)];
 b = x(10:12)';
    for i = 1:n

        v = mB(i, :)';
        g = gB(i , :)';

        % caluate e 
        e(i , :) = v'*L'*L*v -2*v'*L'*L*b + b'*L'*L*b - h^2;
     %   e(i , :) = (L*(v - b))' * (L*(v - b)) - h^2;
      %  e(n+ i, :) = g'*L*v - g'*L*b - h*norm(g)*x(13);
        e(n+ i, :) = g'*L*(v-b) -  h*norm(g)*x(13);

        alpa = L * (v - b);
        beta = v - b;

        % caluate J  1- r
        gradient(i, 1) = 2 * alpa(1) * beta(1);
        gradient(i, 2) = 2 * alpa(1) * beta(2);
        gradient(i, 3) = 2 * alpa(1) * beta(3);
        gradient(i, 4) = 2 * alpa(2) * beta(1);
        gradient(i, 5) = 2 * alpa(2) * beta(2);
        gradient(i, 6) = 2 * alpa(2) * beta(3);
        gradient(i, 7) = 2 * alpa(3) * beta(1);
        gradient(i, 8) = 2 * alpa(3) * beta(2);
        gradient(i, 9) = 2 * alpa(3) * beta(3);

        gradient(i, 10) = -2 * (alpa(1) * L(1,1) + alpa(2) * L(2,1) + alpa(3) * L(3,1)); 
        gradient(i, 11) = -2 * (alpa(1) * L(1,2) + alpa(2) * L(2,2) + alpa(3) * L(3,2)); 
        gradient(i, 12) = -2 * (alpa(1) * L(1,3) + alpa(2) * L(2,3) + alpa(3) * L(3,3)); 
        gradient(i, 13) = 0;

        % caluate J  r- 2r
        gradient(n + i, 1) =  g(1) * beta(1);
        gradient(n + i, 2) =  g(1) * beta(2);
        gradient(n + i, 3) =  g(1) * beta(3);
        gradient(n + i, 4) =  g(2) * beta(1);
        gradient(n + i, 5) =  g(2) * beta(2);
        gradient(n + i, 6) =  g(2) * beta(3);
        gradient(n + i, 7) =  g(3) * beta(1);
        gradient(n + i, 8) =  g(3) * beta(2);
        gradient(n + i, 9) =  g(3) * beta(3);
        gradient(n + i, 10) = - (g(1) * L(1,1) + g(2) * L(2,1) + g(3) * L(3,1)); 
        gradient(n + i, 11) = - (g(1) * L(1,2) + g(2) * L(2,2) + g(3) * L(3,2)); 
        gradient(n + i, 12) = - (g(1) * L(1,3) + g(2) * L(2,3) + g(3) * L(3,3)); 
        gradient(n + i, 13) = -norm(v) * norm(g);
    end 

    residual = e' * e;
end

 