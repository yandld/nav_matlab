%% 超核 IMU matlab接收程序
clear;
clc;
close all;
format short;

%% 默认配置
DEFALUT_BAUD = 115200;
PORT = 'COM4';

%% 串口选择
if length(serialportlist) >=1 %发现多个串口
    fprintf("可用串口:%s\n", serialportlist);
end

if length(serialportlist) == 1 %只有一个串口
    PORT = serialportlist;
end

if isempty(serialportlist) == true %没有串口
    fprintf("无可用串口\n");
end

%% 提示
fprintf('请使用matlab 2020b 及以上版本!!!\n');
fprintf('输入 clear s  或者 CTRL+C 可以终止串口传输\n');
x = input("按回车键续...\n");

%% 打开串口
s = serialport(PORT, DEFALUT_BAUD); %创建串口
%configureCallback(s,"byte",100,@callbackFcn)  %串口事件回调设置

while true
    len = s.NumBytesAvailable;
    if len > 0
        data = read(s, s.NumBytesAvailable,"uint8"); %读取还串口数据
      [imu_data, new_data_rdy] = parse_fame(data); %解析串口数据
        if new_data_rdy == 1
            fprintf("加速度:%.3f %.3f %.3f\n", imu_data.acc);
            fprintf("角速度:%.3f %.3f %.3f\n", imu_data.gyr);
            fprintf("欧拉角: Roll:%.2f Pitch:%.2f Yaw:%.2f\n", imu_data.roll, imu_data.pitch, imu_data.yaw);
        end
    end
    
    pause(0.02);
end


% 解析帧中数据域
function imu_data = parse_data(data)
     len = length(data); %数据域长度
     
    offset = 1;
    while offset < len
        byte = data(offset);
        switch byte
            case 0x90 % ID标签
                imu_data.id = data(offset+1);
                offset = offset + 2;
            case 0xA0 %加速度
                tmp = typecast(uint8(data(offset+1:offset+6)), 'int16');
                imu_data.acc = double(tmp) / 1000;
                offset = offset + 7;
            case 0xB0 %角速度
                tmp = typecast(uint8(data(offset+1:offset+6)), 'int16');
                imu_data.gyr = double(tmp) / 10;
                offset = offset + 7;
            case 0xC0 %地磁
                tmp = typecast(uint8(data(offset+1:offset+6)), 'int16');
                imu_data.mag = double(tmp) / 10;
                offset = offset + 7;
            case 0xD0 %欧拉角
                tmp = typecast(uint8(data(offset+1:offset+6)), 'int16');
                imu_data.pitch = double(tmp(1)) / 100;
                imu_data.roll = double(tmp(2)) / 100;
                imu_data.yaw = double(tmp(3)) / 10;
                offset = offset + 7;
            case 0xF0 % 气压
                offset = offset + 5;
            case 0x91 % 0x91数据包
                 imu_data.id = data(offset+1);
                 imu_data.acc =double(typecast(uint8(data(offset+12:offset+23)), 'single'));
                 imu_data.gyr =double(typecast(uint8(data(offset+24:offset+35)), 'single'));
                 imu_data.mag =double(typecast(uint8(data(offset+36:offset+47)), 'single'));
                 imu_data.roll = double(typecast(uint8(data(offset+48:offset+51)), 'single'));
                 imu_data.pitch = double(typecast(uint8(data(offset+52:offset+55)), 'single'));
                 imu_data.yaw = double(typecast(uint8(data(offset+56:offset+59)), 'single'));
                 imu_data.quat = double(typecast(uint8(data(offset+60:offset+75)), 'single'));
                offset = offset + 76;
            otherwise
               % offset = offset + 1;
        end
    end
    
end



% 拆包一帧，并校验CRC
function [imu_data, new_data_rdy] = parse_fame(data)    
     imu_data = 0;
new_data_rdy = 0;

persistent current_state; %状态机
if isempty(current_state)
    current_state=0;
end

persistent frame_len;   % 帧中数据域长度
persistent frame_dat;  %一帧数据内容
persistent frame_dat_cnt; %帧中数据域计数器

len = length(data);
if len > 0
    %data = read(src,src.NumBytesAvailable,"uint8");
    len = length(data);
    
    for i = 1:len
        byte = data(i);
        switch(current_state)
            case 0 %帧头 0x5A
                if(byte == 0x5A)
                    frame_dat_cnt = 1;
                    current_state = 1;
                end
            case 1 %帧头0xA5
                if(byte == 0xA5)
                    current_state = 2;
                end
            case 2 %长度低字节
                frame_len = byte;
                current_state = 3;
            case 3 %长度高字节
                frame_len = frame_len + byte*256;  % 长度字段
                current_state = 4;
            case 4 % CRC字段低
                current_state = 5;
            case 5 % CRC字段高
                current_state = 6;
            case 6 % 帧中数据段
                if(frame_dat_cnt >= frame_len+6)
                    crc1 = frame_dat(5) + frame_dat(6)*256;

                    % 去除CRC校验字段
                     crc_text = frame_dat;
                     crc_text(5:6) = [];
                    
                     %计算CRC
                    crc2 = crc16(double(crc_text));
                    if crc1 == crc2
                        imu_data = parse_data(frame_dat(7:end));
                        new_data_rdy = 1;
                    end
                    current_state = 0;
                end
        end
        frame_dat(frame_dat_cnt) = byte;
        frame_dat_cnt = frame_dat_cnt+1;
    end
end

end


% data = "5A A5 4C 00 6C 51 91 00 A0 3B 01 A8 02 97 BD BB 04 00 9C A0 65 3E A2 26 45 3F 5C E7 30 3F E2 D4 5A C2 E5 9D A0 C1 EB 23 EE C2 78 77 99 41 AB AA D1 C1 AB 2A 0A C2 8D E1 42 42 8F 1D A8 C1 1E 0C 36 C2 E6 E5 5A 3F C1 94 9E 3E B8 C0 9E BE BE DF 8D BE";
% data = sscanf(data,'%2x');
% parse_fame(data);




