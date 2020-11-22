%% A new calibration method for tri-axial field sensors in strap-down navigation systems   点积不变法
% Accelerometer: acc is sensor frame, n= number of obs,   m = 3(x y z)
% Magnetometer: raw mag in sensor frame
% A: mag misalignment matrix
% b: mag bias

 function [mis bias] = dpi (acc, mag, dot_product)

    nbos = length(acc);
    
    % x = l11 l12 l13 l21 l22 l23 l31 l32 l33 b1 b2 b3
    A(:,1) = acc(:,1).* mag(:,1);
    A(:,2) = acc(:,1).* mag(:,2);
    A(:,3) = acc(:,1).* mag(:,3);
    A(:,4) = acc(:,2).* mag(:,1);
    A(:,5) = acc(:,2).* mag(:,2);
    A(:,6) = acc(:,2).* mag(:,3);
    A(:,7) = acc(:,3).* mag(:,1);
    A(:,8) = acc(:,3).* mag(:,2);
    A(:,9) = acc(:,3).* mag(:,3);
    A(:,10) = -acc(:,1);
    A(:,11) = -acc(:,2);
    A(:,12) = -acc(:,3);

    %Y = ones(length(gB),1) * dot(gmOb, gReference);
    Y = ones(nbos,1) * dot_product;
    B = inv(A'*A)*A'*Y; 
    % or you can use lsqlin(A,Y)
    
    mis = [B(1:3)'; B(4:6)'; B(7:9)'];
    bias = [B(10) B(11) B(12)];
 end
 
%% DPI module
% syms  l11 l12 l13 l21 l22 l23 l31 l32 l33 real
% syms  b1 b2 b3 real
% syms  g1 g2 g3 real
% syms  v1 v2 v3 real
% syms  CONST real

% b: bias
% l11 - l33: mis align
% v magnormetor reading
% g graivity

% model
% L = sym('l%d%d', [3 3]);
% B = [b1 b2 b3]';
% V = [v1 v2 v3]';
% G = [g1 g2 g3]';
% G'*(L*V - B) - CONST
% %G'*(V - B) - CONST
% 
% collect(ans, [L B]) 



