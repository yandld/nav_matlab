function demo_plot(dataset, out_data)

figure('NumberTitle', 'off', 'Name', '原始数据');
subplot(2, 2, 1);
plot(dataset.imu.acc');
legend("X", "Y", "Z");
title("加速度测量值");
subplot(2, 2, 2);
plot(dataset.imu.gyr');
legend("X", "Y", "Z");
title("陀螺测量值");
subplot(2, 2, 3);
plot(dataset.uwb.tof');
title("UWB测量值(伪距)");
subplot(2,2,4);
plot(diff(dataset.uwb.tof'));
title("伪距的差分(diff(tof))");

figure('NumberTitle', 'off', 'Name', '滤波器估计零偏');
subplot(2,1,1);
plot(out_data.delta_u(:,1:3));
legend("X", "Y", "Z");
title("加速度零偏");
subplot(2,1,2);
plot(rad2deg(out_data.delta_u(:,4:6)));
legend("X", "Y", "Z");
title("陀螺仪零偏");

figure('NumberTitle', 'off', 'Name', '量测滤波信息');
subplot(2,1,1);
plot(out_data.L);
title("量测置信度S");

if(isfield(out_data, "good_anchor_cnt"))
    subplot(2,1,2);
    plot(out_data.good_anchor_cnt, '.-');
    title("量测更新时使用的基站数量");
end


figure('NumberTitle', 'off', 'Name', 'PVT');
subplot(2,2,1);
plot(out_data.x(:,1:3));
legend("X", "Y", "Z");
title("位置");
subplot(2,2,2);
plot(out_data.x(:,4:6));
legend("X", "Y", "Z");
title("速度");
subplot(2,2,3);
plot(out_data.eul);
legend("X", "Y", "Z");
title("欧拉角(姿态)");


figure('NumberTitle', 'off', 'Name', '硬件给出的UWB解算位置');
subplot(1,2,1);
plot3(out_data.uwb.pos(1,:), out_data.uwb.pos(2,:), out_data.uwb.pos(3,:), '.');
axis equal
title("UWB 伪距解算位置");

% if(isfield(dataset, "pos"))
%     subplot(1,2,2);
%     plot3(dataset.pos(1,:), dataset.pos(2,:), dataset.pos(3,:), '.');
%     hold on;
%     plot3(out_data.x(:,1), out_data.x(:,2), out_data.x(:,3), '.-');
%     axis equal
%     title("硬件给出轨迹");
% end

figure('NumberTitle', 'off', 'Name', '纯UWB伪距解算的位置和融合轨迹');
plot(out_data.uwb.pos(1,:), out_data.uwb.pos(2,:), '.');
hold on;
plot(out_data.uwb.fusion_pos(1,:), out_data.uwb.fusion_pos(2,:), '.-');

anch = out_data.uwb.anchor;
hold all;
scatter(anch(1, :),anch(2, :),'k');
for i=1:size(anch,2)
    text(anch(1, i),anch(2, i),"A"+(i-1))
end
hold off;


legend("伪距解算UWB轨迹", "融合轨迹");

% 
% if(isfield(dataset, "pos"))
%      figure('NumberTitle', 'off', 'Name', '硬件给出的位置和融合轨迹');
%     plot(dataset.pos(1,:), dataset.pos(2,:), '.');
%     hold on;
%     plot(out_data.uwb.fusion_pos(1,:), out_data.uwb.fusion_pos(2,:), '.-');
%     legend("硬件给出轨迹", "融合轨迹");
%     anch = out_data.uwb.anchor;
%     hold all;
%     scatter(anch(1, :),anch(2, :),'k');
%     for i=1:size(anch,2)
%         text(anch(1, i),anch(2, i),"A"+(i-1))
%     end
%     hold off;
% end





