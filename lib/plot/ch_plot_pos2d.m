%%  显示2D轨迹
% example ch_plot_pos2d('p1', out_data.uwb.fusion_pos(1:2,:)', 'p2', out_data.uwb.pos(1:2,:)',  'title', '融合轨迹', 'legend', ["UWB_IMU融合轨迹", "纯UWB定位", "基站"]);

function ch_plot_pos2d(varargin)

param= inputParser;
param.addOptional('p1', []);
param.addOptional('p2', []);
param.addOptional('p3', []);
param.addOptional('title', []);
param.addOptional('legend', []);

param.parse(varargin{:});
r = param.Results;

figure;
if(~isempty(r.p1))
    plot(r.p1(:,1), r.p1(:,2), '.r');
    hold on;
end

if(~isempty(r.p2))
    plot(r.p2(:,1), r.p2(:,2), '.g');
    hold on;
end

if(~isempty(r.p3))
    plot(r.p3(:,1), r.p3(:,2), '.b');
    hold on;
end

title(r.title);
legend(r.legend);

axis equal
xlabel('X(m)');  ylabel('Y(m)');


end


