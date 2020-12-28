%%  显示3D轨迹
% example: ch_plot_pos3d('p1', out_data.uwb.fusion_pos(1:2,:)', 'p2', out_data.uwb.pos(1:2,:)',  'title', '融合轨迹', 'legend', ["UWB_IMU融合轨迹", "纯UWB定位", "基站"]);

function ch_plot_pos3d(varargin)

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
plot3(r.p1(:,1), r.p1(:,2), r.p1(:,3), '.r');
hold on;
end

if(~isempty(r.p2))
plot3(r.p2(:,1), r.p2(:,2), r.p2(:,3), '.g');
hold on;
end

if(~isempty(r.p3))
plot3(r.p3(:,1), r.p3(:,2), r.p3(:,3), '.b');
hold on;
end

title(r.title);
legend(r.legend);

axis equal
xlabel('X(m)');  ylabel('Y(m)');   zlabel('Z(m)'); 
end



