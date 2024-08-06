function generateKmlFiles(file_name, latitude, longitude, N, rgbColor)
    % 每隔N个取出一个经纬度
    lat_sampled = latitude(1:N:end);
    lon_sampled = longitude(1:N:end);

    % 将 RGB 转换为 KML 颜色格式（默认不透明）
    color = sprintf('ff%02x%02x%02x', rgbColor(3), rgbColor(2), rgbColor(1));

    % 打开文件
    fileID = fopen(file_name, 'w');

    % 写入KML文件头
    fprintf(fileID, '<?xml version="1.0" encoding="UTF-8"?>\n');
    fprintf(fileID, '<kml xmlns="http://www.opengis.net/kml/2.2">\n');
    fprintf(fileID, '<Document>\n');
    
    % 写入样式
    fprintf(fileID, '<Style id="pointStyle">\n');
    fprintf(fileID, '<IconStyle>\n');
    fprintf(fileID, '<color>%s</color>\n', color);
    fprintf(fileID, '<scale>0.5</scale>\n'); % 调整点的大小
    fprintf(fileID, '</IconStyle>\n');
    fprintf(fileID, '</Style>\n');

    % 写入每个点
    for i = 1:length(lat_sampled)
        fprintf(fileID, '<Placemark>\n');
        fprintf(fileID, '<styleUrl>#pointStyle</styleUrl>\n');
        fprintf(fileID, '<Point>\n');
        fprintf(fileID, '<coordinates>%f,%f</coordinates>\n', lon_sampled(i), lat_sampled(i));
        fprintf(fileID, '</Point>\n');
        fprintf(fileID, '</Placemark>\n');
    end

    % 写入KML文件尾
    fprintf(fileID, '</Document>\n');
    fprintf(fileID, '</kml>\n');

    % 关闭文件
    fclose(fileID);
end
