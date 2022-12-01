function [lat_save, lon_save, utc_time] = gga2pos(file_name)
    data = importdata(file_name);
    
    lat_save = [];
    lon_save = [];
    utc_time = [];
    for i = 1:length(data)
        str = char(data(i));
        if length(str)>20 && (strcmp(str(1:6),'$GNRMC') || strcmp(str(1:6),'$GPRMC'))
            sstr = string(str);
            sstr = sstr.split(',');
            
            if strlength(sstr(4))>4
                lat = sstr(4);
                lat = lat.split('.');
                lat1 = char(lat(1));
                lat_deg = str2double(lat1(1:end-2));
                lat_min = str2double(lat1(end-1:end)) + str2double(strcat('0.',lat(2)));
                lat = lat_deg + lat_min/60;
                lat_save = [lat_save; lat];
            else
                lat_save = [lat_save; NaN];
            end
            
            if strlength(sstr(6))>4
                lon = sstr(6);
                lon = lon.split('.');
                lon1 = char(lon(1));
                lon_deg = str2double(lon1(1:end-2));
                lon_min = str2double(lon1(end-1:end)) + str2double(strcat('0.',lon(2)));
                lon = lon_deg + lon_min/60;
                lon_save = [lon_save; lon];
            else
                lon_save = [lon_save; NaN];
            end
                        
            utc_str = char(sstr(2));
            utc = str2double(utc_str(1:2))*3600 + ...
                  str2double(utc_str(3:4))*60 + ...
                  str2double(utc_str(5:end));
            utc_time = [utc_time; utc];
        end
    end
end

