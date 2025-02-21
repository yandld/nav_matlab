clear; clc; close all;

%% Test Case 1: 标准四元数
fprintf('Case 1: 标准四元数\n');
Qb2n = [0.9313, -0.0476, 0.3049, -0.1935]';

% 期望值
exp_312 = [-11.93, 34.17, -19.80];  % deg
exp_321 = [-14.32, 33.33, -27.78];  % deg
exp_dcm = [
    0.7392    0.3314    0.5863
   -0.3894    0.9206   -0.0293
   -0.5495   -0.2067    0.8095
];

% 计算结果
[eul_312, eul_321] = ch_q2eul(Qb2n);
dcm = ch_q2m(Qb2n);

% 验证结果
tol = 1e-2;
err_312 = max(abs(rad2deg(eul_312)' - exp_312));
err_321 = max(abs(rad2deg(eul_321)' - exp_321));
err_dcm = max(max(abs(dcm - exp_dcm)));

fprintf('312欧拉角: %s (err=%.2f)\n', iif(err_312<tol,'P','F'), err_312);
fprintf('321欧拉角: %s (err=%.2f)\n', iif(err_321<tol,'P','F'), err_321);
fprintf('DCM矩阵:   %s (err=%.2e)\n', iif(err_dcm<tol,'P','F'), err_dcm);

function r = iif(cond,t,f)
    if cond, r=t; else, r=f; end
end
