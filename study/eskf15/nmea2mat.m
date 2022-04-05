close all;
clear;
clc;

file_name = 'data20220405_RTK';

data = textread(strcat(file_name, '.txt'),'%s');
data_length = length(data);

% data_length = 142058;    %data20220405_RTK后半部分数据有问题，截取前半部分

imu_length = 0;
gnss_length = 0;
ins_length = 0;
span_length = 0;
for i = 1:data_length
    str = char(data(i));

    if (strcmp(str(1:4),'$IMU') || strcmp(str(1:6),'$GNIMU'))
        if imu_length == 0
            sstr = string(str);
            sstr = sstr.split(',');
            imu_n = length(sstr) - 1;
        end
        imu_length = imu_length + 1;
    end

    if (strcmp(str(1:4),'$PVT') || strcmp(str(1:6),'$GNPVT'))
        if gnss_length == 0
            sstr = string(str);
            sstr = sstr.split(',');
            gnss_n = length(sstr) - 1;
        end
        gnss_length = gnss_length + 1;
    end

    if (strcmp(str(1:4),'$INS') || strcmp(str(1:6),'$GNINS'))
        if ins_length == 0
            sstr = string(str);
            sstr = sstr.split(',');
            ins_n = length(sstr) - 1;
        end
        ins_length = ins_length + 1;
    end

    if (strcmp(str(1:9),'#INSPVAXA'))
        span_length = span_length + 1;
    end
end

if imu_length>0
    imu_data = zeros(imu_length, imu_n);
else
    imu_data = [];
end

if gnss_length>0
    gnss_data = zeros(gnss_length, gnss_n);
else
    gnss_data = [];
end

if ins_length>0
    ins_data = zeros(ins_length, ins_n);
else
    ins_data = [];
end

if span_length>0
    span.lat = zeros(span_length, 1);
    span.lon = zeros(span_length, 1);
    span.alt = zeros(span_length, 1);
    span.vel = zeros(span_length, 3);
    span.att = zeros(span_length, 3);
    span.pos_std = zeros(span_length, 3);
    span.vel_std = zeros(span_length, 3);
    span.att_std = zeros(span_length, 3);
    span.time = zeros(span_length, 1);
    span.ins_status = string(zeros(span_length, 1));
    span.pos_type = string(zeros(span_length, 1));
else
    span_data = [];
end

tic;
count_sec = 1;
imu_p = 1;
gnss_p = 1;
ins_p = 1;
span_p = 1;
for i = 1:data_length
    current_time = toc;
    if current_time>count_sec
        percent = (i)/(data_length);
        clc;
        fprintf('已处理%.3f%%, 用时%.3f秒, 预计还需%.3f秒\n', (i)/(data_length)*100, current_time, current_time/percent*(1-percent));
        count_sec = count_sec + 1;
    end

    str = char(data(i));

    if (strcmp(str(1:4),'$IMU') || strcmp(str(1:6),'$GNIMU'))
        sstr = string(str);
        sstr = sstr.split(',');
        imu_data(imu_p,:) = double(string(sstr(2:end)))';
        imu_p = imu_p + 1;
    end

    if (strcmp(str(1:4),'$PVT') || strcmp(str(1:6),'$GNPVT'))
        sstr = string(str);
        sstr = sstr.split(',');
        gnss_data(gnss_p,:) = double(string(sstr(2:end)))';
        gnss_p = gnss_p + 1;
    end

    if (strcmp(str(1:4),'$INS') || strcmp(str(1:6),'$GNINS'))
        sstr = string(str);
        sstr = sstr.split(',');
        ins_data(ins_p,:) = double(string(sstr(2:end)))';
        ins_p = ins_p + 1;
    end

    if (strcmp(str(1:9),'#INSPVAXA'))
        sstr = string(str);
        sstr = sstr.split(';');

        sstr1 = sstr(1);
        sstr1 = sstr1.split(',');
        
        sstr2 = sstr(2);
        sstr2 = sstr2.split(',');

        if length(sstr2)>=21
            span.time(span_p) = str2double(sstr1(7));
            span.lat(span_p) = str2double(sstr2(3));
            span.lon(span_p) = str2double(sstr2(4));
            span.alt(span_p) = str2double(sstr2(5));
            span.vel(span_p,:) = str2double(sstr2([8,7,9]))';
            span.att(span_p,:) = str2double(sstr2([11,10,12]))';
            span.pos_std(span_p,:) = str2double(sstr2(13:15))';
            span.vel_std(span_p,:) = str2double(sstr2([17,16,18]))';
            span.att_std(span_p,:) = str2double(sstr2([20,19,21]))';
            span.ins_status(span_p) = sstr2(1);
            span.pos_type(span_p) = sstr2(2);
            span_p = span_p + 1;
        end
    end
end

clc;
fprintf('已处理完毕，用时%.3f秒\n', toc);

save(file_name, 'imu_data', 'gnss_data', 'ins_data', 'span');