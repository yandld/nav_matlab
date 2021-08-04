clear;
clc;
close all;

% https://github.com/libing64/learning_rtklib
% 
% C : pseudorange 伪距
% L : carrier phase 载波相位
% D : doppler 多普勒
% S : signal strengh 信号强度
% 每条观测数据有指明观测类型和频段，主要以下几种
% 
% C1 : C/A码在L1频段的伪距测量
% L1 L2 : 在L1/L2频段的相位测量
% P1 P2 : P码在L1/L2频段的伪距测量
% D1 D2 : 在L1/L2频段的多普勒频率
% T1 T2 : 在150MHz(T1)和400MHz(T2)的传输集成多普勒

% 卫星obs 观测文件
[all_obs, rec_xyz]  = read_rinex_obs("SAVE2021_7_20_22-32-23.obs");

all_obs.col

PRN = 10; %需要计算的卫星数
N = length(all_obs.data);

% 选出需要的卫星(PRN相同)
 index = find(all_obs.data(:,all_obs.col.PRN) == PRN);
 obs = all_obs.data(index, :);



plot(obs(:,2), obs(:,4:6), '.-');
legend("L1(载波)", "L2(载波)", "C1(伪距)");






