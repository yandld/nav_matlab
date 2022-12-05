function plot_google_map(gnss, matlab, mcu)
%将轨迹显示在谷歌地图上
    wm = webmap('World Imagery');
    s = geoshape(gnss(:,1), gnss(:,2));
    
    for i=1:nargin
        if i==1
            wmline(s, 'Color', 'red', 'Width', 1, 'OverlayName', 'GNSS');
        elseif i==2
            wmline(matlab(:,1), matlab(:,2), 'Color', 'blue', 'Width', 1, 'OverlayName', 'MATLAB');
        elseif i==3
            wmline(mcu(:,1), mcu(:,2), 'Color', 'magenta', 'Width', 2, 'OverlayName', 'MCU');
        end
    end
    wmmarker(gnss(1,1), gnss(1,1), 'Color', 'g');
    wmmarker(gnss(end,1), gnss(end,1), 'Color', 'r');
    % 线性可选颜色：
    % 'red', 'green', 'blue', 'white', 'cyan', 'magenta', 'yellow', 'black'
end

