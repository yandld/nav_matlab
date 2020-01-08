% using DIP mehold, only compute bias, norm_h, lamda(cos(inclination)),
% modifed from DIP13

% %% moudle DIP5
% syms  b1 b2 b3 real
% syms  g1 g2 g3 real
% syms  v1 v2 v3 real
% syms  lambda real
% syms norm_h norm_g real
% B = [b1 b2 b3]';
% V = [v1 v2 v3]';
% G = [g1 g2 g3]';
% % %G'*(V - B) - CONST
% 
%  f1= (V - B)' * (V - B) - norm_h^2;
%  j1 = jacobian(f1, [B' norm_h lambda]);
%  collect(j1, B) ;
%  
%  f2 = G' * (V - B) - norm_g * norm_h * lambda;
%  
%  f = f1 + f2;
%  j2 = jacobian(f2, [B' norm_h lambda]);
%  collect(j2, B);
% 
% 
% f1
% j1
% 
% f2 
% j2

 function [mis, bias, norm_h, lamda, inter, J] = dip5 (acc, mag, x, epsilon)
    % initalize value
    mis = eye(3);
    bias = zeros(1,3);
    lamda = cos(deg2rad(60));
    
    mu = 0.000; % damp
    inter = 100;
    last_e = 100;
 
    for i = 1:inter
        [J(i), Jacobi, e] = compute_cost_and_jacob (x, mag, acc);
         x = x - (inv((Jacobi' * Jacobi + mu * eye(length(x)))) * Jacobi' * e)';
        
       if((abs(norm(e) - norm(last_e))) < epsilon)
             bias = x(1:3)';
             lamda = x(5);
             inter = i;
             norm_h = x(4);
           break;
       end
       last_e  = e;
    end
    
end
 
%% Function
% Jval:  error
% gradient :  gradient of cost func
% e:  bx by bz lamada
% X: [Bx By Bz norm_h lambada]

function[Jval, gradient, e]=compute_cost_and_jacob(x, mB, gB)
    n = length(mB);
    e = zeros(n * 2, 1);
    gradient = zeros(n * 2, 5);
    b = x(1:3)';
    norm_h = x(4);
    lamda = x(5);
    
    for i = 1:n
        v = mB(i, :)';
        g = gB(i , :)';
        
        % caluate e
        e(i , :) = (v - b)' * (v - b) - norm_h^2;
         e(n+ i, :) = g' * (v - b) - (norm_h * norm(g) * lamda);

        % caluate J  1- r
        gradient(i, 1) = 2 * (b(1) - v(1));
        gradient(i, 2) = 2 * (b(2) - v(2));
        gradient(i, 3) = 2 * (b(3) - v(3));
        gradient(i, 4) = -2 * norm_h;
        gradient(i, 5) = 0;

        % caluate J  r- 2r
        gradient(n + i, 1) =  -g(1);
        gradient(n + i, 2) =  -g(2);
        gradient(n + i, 3) =  -g(3); 
        gradient(n + i, 4) =  -lamda * norm(g);
        gradient(n + i, 5) =  -norm_h * norm(g);
    end 

    Jval = e' * e;
end

 