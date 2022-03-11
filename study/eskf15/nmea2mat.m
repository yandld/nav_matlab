close all;
clear;
clc;

file_name = 'data20220311';

data = textread(strcat(file_name,'.txt'),'%s');
data_length = length(data);

imu_data = [];
gnss_data = [];
ins_data = [];

tic;
count_sec = 1;
for i = 1:data_length
    current_time = toc;
    if current_time>count_sec
        percent = (i)/(data_length);
        clc;
        fprintf('已处理%.3f%%, 用时%.3f秒, 预计还需%.3f秒\n', (i)/(data_length)*100, current_time, current_time/percent*(1-percent));
        count_sec = count_sec + 1;
    end

    str = char(data(i));

    if (strcmp(str(1:6),'$GNIMU'))
        sstr = string(str);
        sstr = sstr.split(',');
        imu_data = [imu_data; double(string(sstr(2:end)))'];
    end

    if (strcmp(str(1:6),'$GNPVT'))
        sstr = string(str);
        sstr = sstr.split(',');
        gnss_data = [gnss_data; double(string(sstr(2:end)))'];
    end

    if (strcmp(str(1:6),'$GNINS'))
        sstr = string(str);
        sstr = sstr.split(',');
        ins_data = [ins_data; double(string(sstr(2:end)))'];
    end
end

clc;
fprintf('已处理完毕，用时%.3f秒\n', toc);

save(file_name, 'imu_data', 'gnss_data', 'ins_data');