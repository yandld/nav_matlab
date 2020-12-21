%%  plot UWB data

function ch_plot_uwb(anchor_pos, tag_pos, varargin)

if nargin == 2
    dim =  cell2mat(varargin(1));
else
    dim = size(tag_pos, 1);
end


%% plot Trajectory
figure;
if dim == 2
    x = tag_pos(1,:);
    y = tag_pos(2,:);
    plot(x,y,'.');

elseif dim == 3
    x = tag_pos(1,:);
    y = tag_pos(2,:);
    z = tag_pos(3,:);
    plot3(x,y,z,'.');
end

xlim([-20 20])
ylim([-20 20])
title('Trajectory');xlabel('x:m');ylabel('y:m');grid on;

hold on;
plotanchors(anchor_pos);
axis equal

end




function plotanchors(anch)
hold all;
scatter(anch(1, :),anch(2, :),'k');
for i=1:size(anch,2)
    text(anch(1, i),anch(2, i),"A"+(i-1))
end
end


