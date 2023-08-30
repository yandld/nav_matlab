
close all;
clear;
clc;

fullfilename  = "2023年08月30日18时20分13秒_RAW.csv";


% 切换到当前工作目录
scriptPath = mfilename('fullpath');
scriptFolder = fileparts(scriptPath);
cd(scriptFolder);

[pathstr, file_name, ext] = fileparts(fullfilename);

fprintf("正在读取%s\r\n", fullfilename);
T = readtable(fullfilename, 'HeaderLines', 3); % 读取 CSV 文件
fprintf("读取完成\r\n");
frame_name = table2cell(T(:,2));

% 提取并剪裁
RAWIMUXB = T(strcmp(frame_name, 'RAWIMUXB'), :);
RAWIMUXB = table2array(RAWIMUXB(:, 3:10));

GNSSRCV  = T(strcmp(frame_name, 'GNSSRCV'), :);
GNSSRCV = table2array(GNSSRCV(:, 3:34));

INSPVAXB = T(strcmp(frame_name, 'INSPVAXB'), :);
INSPVAXB = table2array(INSPVAXB(:, 3:25));


fprintf("%-20s: %d帧\n", "INSPVAXB", length(INSPVAXB));
fprintf("%-20s: %d帧\n", "RAWIMUXB",length(RAWIMUXB));
fprintf("%-20s: %d帧\n", "GNSSRCV",length(GNSSRCV));


% 结构化
data.imu.time = RAWIMUXB(: ,1:2);
data.imu.gyr = RAWIMUXB(: ,3:5);
data.imu.acc = RAWIMUXB(: ,6:8);

data.ins.time =INSPVAXB(: ,1:2);
data.ins.att =INSPVAXB(: ,3:5);
data.ins.vel =INSPVAXB(: ,6:8);
data.ins.lla =INSPVAXB(: ,9:11);
data.ins.att_std =INSPVAXB(: ,12:14);
data.ins.vel_std =INSPVAXB(: ,15:17);
data.ins.lla_std =INSPVAXB(: ,18:20);
data.ins.insStatus =INSPVAXB(: ,21);
data.ins.posType =INSPVAXB(: ,22);
data.ins.evtBit =INSPVAXB(: ,23);

data.gnss.time =GNSSRCV(: ,1:2);
data.gnss.lla =GNSSRCV(: ,3:5);
data.gnss.vel =GNSSRCV(: ,6:8);
data.gnss.att =GNSSRCV(: ,9:11);
data.gnss.lla_std =GNSSRCV(: ,12:14);
data.gnss.vel_std =GNSSRCV(: ,15:17);
data.gnss.att_std =GNSSRCV(: ,18:20);
data.gnss.od =GNSSRCV(: ,21);
data.gnss.nv =GNSSRCV(: ,22);
data.gnss.nv_heading =GNSSRCV(: ,23);
data.gnss.solq =GNSSRCV(: ,24);
data.gnss.solq_heading =GNSSRCV(: ,25);
data.gnss.diff_age =GNSSRCV(: ,26);
data.gnss.wb =GNSSRCV(: ,27:29);
data.gnss.gb =GNSSRCV(: ,30:32);


% % 画帧间隔
subplot(1,3,1);
plot(diff(INSPVAXB(:, 2)), '.-');
title("INSPVAXB 帧时间差分");
xlabel("帧数");
ylabel("s");

subplot(1,3,2);
plot(diff(RAWIMUXB(:, 2)), '.-');
title("RAWIMUXB 帧时间差分");
xlabel("帧数");
ylabel("s");

subplot(1,3,3);
plot(diff(unique(GNSSRCV(:, 2))), '.-');
title("GNSSRCV 帧时间差分");
xlabel("帧数");
ylabel("s");


%保存数据
fprintf("保存数据...\r\n");
fprintf("保存位置%s/%s\r\n", scriptFolder, fullfile(file_name + ".mat"));
save(fullfile(file_name + ".mat"), 'data');
fprintf("保存完成\r\n");


