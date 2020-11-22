%%  plot UWB data

function ch_plot_uwb(data, varargin)

if nargin == 2
    dim =  cell2mat(varargin(1));
else
    dim = size(data.pos, 1);
end


%% plot Trajectory
figure;
if dim == 2
    x = data.pos(1,:);
    y = data.pos(2,:);
    plot(x,y,'.');
if( isfield(data, 'fusion_pos'))
    hold on;
    x = data.fusion_pos(1,:);
    y = data.fusion_pos(2,:);
    plot(x,y);
end
elseif dim == 3
    x = data.pos(1,:);
    y = data.pos(2,:);
    z = data.pos(3,:);
    plot3(x,y,z,'.');
end

xlim([-20 20])
ylim([-20 20])
title('Trajectory');xlabel('x:m');ylabel('y:m');grid on;

hold on;
plotanchors(data.anchor);
axis equal

%% plot TOF
figure;
plot(data.tof');
legend();
end




function plotanchors(anch)
hold all;
scatter(anch(1, :),anch(2, :),'k');
for i=1:size(anch,2)
    text(anch(1, i),anch(2, i),"A"+(i-1))
end
end


