function [lat, lon, alt, vel, att, pos_std, vel_std, att_std, ins_status, pos_type, time] = inspvaxa2pvax(file_name)
%     data = importdata(file_name);
    fid = fopen(file_name, 'rt');
    
    lat = [];
    lon = [];
    alt = [];
    vel = [];
    att = [];
    pos_std = [];
    vel_std = [];
    att_std = [];
    ins_status = [];
    pos_type = [];
    time = [];

%     for i = 1:length(data)
%         str = char(data(i));
    while ~feof(fid)
        str = fgetl(fid);
        if length(str)>10 && (strcmp(str(1:9),'#INSPVAXA'))
            sstr = string(str);
            sstr = sstr.split(';');
            
            sstr1 = sstr(1);
            sstr1 = sstr1.split(',');
            gps_week_time = mod(str2double(sstr1(7)), (3600*24)) - 18;
            time = [time; gps_week_time];
            
            sstr2 = sstr(2);
            sstr2 = sstr2.split(',');

            lat = [lat; str2double(sstr2(3))];
            lon = [lon; str2double(sstr2(4))];
            alt = [alt; str2double(sstr2(5))];
            vel = [vel; str2double(sstr2([8,7,9]))'];
            att = [att; str2double(sstr2([11,10,12]))'];
            pos_std = [pos_std; str2double(sstr2(13:15))'];
            vel_std = [vel_std; str2double(sstr2([17,16,18]))'];
            att_std = [att_std; str2double(sstr2([20,19,21]))'];
            
            ins_status = [ins_status; sstr2(1)];
            pos_type = [pos_type; sstr2(2)];
        end
    end

    fclose(fid);
end

