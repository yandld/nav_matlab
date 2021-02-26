
syms x y theta xl d yl real
m1 = sqrt((xl - x - d*cos(theta))^(2) + (yl - y - d*sin(theta))^(2));
m2 = atan2(yl - y - d*sin(theta), xl - x - d*cos(theta)) - theta;

J = jacobian([m1], [x;y;theta]);
J


