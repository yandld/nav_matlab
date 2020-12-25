%%  plot UWB data

function ch_plot_pos3d(varargin)

param= inputParser;
param.addOptional('p1', []);
param.addOptional('p2', []);
param.addOptional('p3', []);
param.addOptional('title', []);
param.addOptional('legend', []);

%然后将输入的参数进行处理，如果有不同于默认值的那就覆盖掉
param.parse(varargin{:});
r = param.Results;

if size(r.p1, 2) == 3
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

% figure;
% if(~isempty(r.p1))
% plot(r.p1(:,1), r.p1(:,2), '.b');
% hold on;
% plot(r.p1(1,1), r.p1(1,2), '-ks');
% end
% axis equal
% title('2D Position');
% xlabel('X(m)');  ylabel('Y(m)');
end


