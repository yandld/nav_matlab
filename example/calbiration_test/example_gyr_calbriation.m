%% Example: 最小二乘法校准陀螺仪
clc;
clear;
close all;
format short
%% 数据集1
input = [
    44.4694  -0.6640  -0.0020 
   0.4183  43.4974   0.2307 
   -0.0600  -0.2073  43.5827
  -42.8855   0.3022   0.1131 
    -0.5930 -44.6703  -0.1181
   -0.1035  -0.1213 -44.5605
    ];

 theory = [
     44.471893   0 0
     0 43.636364   0
     0 0 43.347378 
     -42.729969  0 0
     0 -44.471893 0
     0 0  -44.444443 ];


%% 数据集2
% input = [
%    100  0       5
%    0    100     0
%    0    0     100
% -100  0        -5
%   0  -100      0
%   0   0     -100
%     ];

% B =  [1 ,2 ,3];
% input = input + B;

% theory = [
%     50   0    0
%     0   50    0
%     0    0    50
%  -50   0     0
%    0   -50   0
%   0     0    -50
%     ];



%% 校准并打印结果
[C, B]= gyr_calibration(input, theory);

fprintf('校准矩阵:');
C
fprintf('零偏:');
B

%% 计算校准后的数据

output(1,:) = C*(input(1,:)') - B;
output(2,:) = C*(input(2,:)') - B;
output(3,:) = C*(input(3,:)') - B;

output(4,:) = C*(input(4,:)') - B;
output(5,:) = C*(input(5,:)') - B;
output(6,:) = C*(input(6,:)') - B;

%% 计算校准前后误差
X =  input - theory;
error_input =  sum(sum(abs(X).^2, 2).^(1/2));

X =  output - theory;
error_ouput =  sum(sum(abs(X).^2, 2).^(1/2));
fprintf('校准前误差: %f    校准后误差: %f\n', error_input, error_ouput);


%% plot
grid on;
plot3(input(:,1), input(:,2), input(:,3), 'or');
hold on;
plot3(output(:,1), output(:,2), output(:,3), '*b');
axis equal

legend('输入', '校准后');





