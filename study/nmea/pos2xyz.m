function [neu] = pos2xyz(lat, lon, alt)
    rad = pi/180;
    deg = 180/pi;
    g = 9.8;
    Re = 6378137;
    Earth_e = 0.00335281066474748;
    
    lat0 = lat(1)*rad;
    h0 = alt(1);
    
    Rm = Re * (1 - 2*Earth_e + 3*Earth_e*sin(lat0)*sin(lat0));
    Rn = Re * (1 + Earth_e*sin(lat0)*sin(lat0));
    Rmh = Rm + h0;
    Rnh = Rn + h0;
    
    neu = [];
    for i = 1:length(lat)
        neu(i, 3) = alt(i) - alt(1);
        neu(i, 2) = (lat(i) - lat(1)) * rad * (Rmh);
        neu(i, 1) = (lon(i) - lon(1)) * rad * (Rnh) * cos(lat0);
    end
end

