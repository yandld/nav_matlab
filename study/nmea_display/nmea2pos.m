function [lat_save, lon_save, alt_save, pos_status] = nmea2pos(file_name)
    data = importdata(file_name);
    
    lat_save = [];
    lon_save = [];
    alt_save = [];
    pos_status = [];
    for i = 1:length(data)
        str = char(data(i));
        if length(str)>6 && (strcmp(str(1:6),'$GNGGA') || strcmp(str(1:6),'$GPGGA'))
            sstr = string(str);
            sstr = sstr.split(',');

            lat = sstr(3);
            lat = lat.split('.');
            lat1 = char(lat(1));
            lat_deg = str2double(lat1(1:end-2));
            lat_min = str2double(lat1(end-1:end)) + str2double(strcat('0.',lat(2)));
            lat = lat_deg + lat_min/60;

            lat_save = [lat_save; lat];

            lon = sstr(5);
            lon = lon.split('.');
            lon1 = char(lon(1));
            lon_deg = str2double(lon1(1:end-2));
            lon_min = str2double(lon1(end-1:end)) + str2double(strcat('0.',lon(2)));
            lon = lon_deg + lon_min/60;

            lon_save = [lon_save; lon];

            alt = str2double(sstr(10));
            alt_save = [alt_save; alt];

            rtk = str2double(sstr(7));
            pos_status = [pos_status; rtk];
        end
    end
end

