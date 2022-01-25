function [lat, lon, alt, vel, att, pos_std, vel_std, att_std, ins_status, pos_type] = inspvaxa2pvax(file_name)
    data = importdata(file_name);
    
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
    for i = 1:length(data)
        str = char(data(i));
        if length(str)>10 && (strcmp(str(1:9),'#INSPVAXA'))
            sstr = string(str);
            sstr = sstr.split(';');
            sstr = sstr(2);
            sstr = sstr.split(',');

            lat = [lat; str2double(sstr(3))];
            lon = [lon; str2double(sstr(4))];
            alt = [alt; str2double(sstr(5))];
            vel = [vel; str2double(sstr([8,7,9]))'];
            att = [att; str2double(sstr([11,10,12]))'];
            pos_std = [pos_std; str2double(sstr(13:15))'];
            vel_std = [vel_std; str2double(sstr([17,16,18]))'];
            att_std = [att_std; str2double(sstr([20,19,21]))'];
            
            ins_status = [ins_status; sstr(1)];
            pos_type = [pos_type; sstr(2)];
        end
    end
end

