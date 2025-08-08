function dynamicDisplayGUI(log, data)
%DYNAMICDISPLAYGUI 卡尔曼滤波结果的GUI动态显示器
%   dynamicDisplayGUI(log, data) 创建一个带播放控制的GUI界面
%
% 输入参数:
%   log  - 包含滤波结果的结构体
%   data - 包含原始数据的结构体
%
% GUI功能:
%   - 播放/暂停控制
%   - 播放速度调节 (1x, 2x, 5x, 10x, 20x)
%   - 快进/快退 (跳转10秒)
%   - 进度条拖拽跳转
%   - 时间显示和跳转输入框
%   - 实时轨迹和参数显示
%   - 清除当前轨迹功能
%   - 图形缩放、平移和坐标轴调整

  % 全局变量
  gui_data = struct();
  gui_data.log = log;
  gui_data.data = data;
  gui_data.current_idx = 1;
  gui_data.is_playing = false;
  gui_data.play_speed = 1;
  gui_data.timer_obj = [];
  gui_data.data_len = size(log.pos, 1);
  gui_data.time_data = data.imu.tow;
  
  % 常量
  R2D = 180/pi;
  GRAVITY = 9.8;
  
  %% 创建主窗口 (增加高度以容纳新图形)
  fig_main = figure('Name', '卡尔曼滤波动态显示器', ...
                    'Position', [50, 50, 1900, 1200], ...
                    'MenuBar', 'figure', ...
                    'ToolBar', 'figure', ...
                    'CloseRequestFcn', @closeGUI);
  
  %% 创建控制面板
  control_panel = uipanel('Parent', fig_main, ...
                         'Position', [0.01, 0.94, 0.98, 0.05], ...
                         'Title', '播放控制', ...
                         'FontSize', 12, ...
                         'FontWeight', 'bold');
  
  % 播放/暂停按钮
  btn_play = uicontrol('Parent', control_panel, ...
                      'Style', 'pushbutton', ...
                      'String', '播放', ...
                      'Position', [20, 5, 80, 25], ...
                      'FontSize', 11, ...
                      'Callback', @togglePlay);
  
  % 快退按钮
  btn_rewind = uicontrol('Parent', control_panel, ...
                        'Style', 'pushbutton', ...
                        'String', '<<10s', ...
                        'Position', [110, 5, 60, 25], ...
                        'FontSize', 10, ...
                        'Callback', @(~,~)jumpTime(-10));
  
  % 快进按钮
  btn_forward = uicontrol('Parent', control_panel, ...
                         'Style', 'pushbutton', ...
                         'String', '10s>>', ...
                         'Position', [180, 5, 60, 25], ...
                         'FontSize', 10, ...
                         'Callback', @(~,~)jumpTime(10));
  
  % 速度控制
  uicontrol('Parent', control_panel, ...
           'Style', 'text', ...
           'String', '播放速度:', ...
           'Position', [260, 10, 70, 15], ...
           'FontSize', 10);
  
  popup_speed = uicontrol('Parent', control_panel, ...
                         'Style', 'popupmenu', ...
                         'String', {'1x', '2x', '5x', '10x', '20x'}, ...
                         'Position', [340, 10, 60, 15], ...
                         'FontSize', 10, ...
                         'Callback', @changeSpeed);
  
  % 进度条
  uicontrol('Parent', control_panel, ...
           'Style', 'text', ...
           'String', '进度:', ...
           'Position', [420, 10, 40, 15], ...
           'FontSize', 10);
  
  slider_progress = uicontrol('Parent', control_panel, ...
                             'Style', 'slider', ...
                             'Min', 1, ...
                             'Max', gui_data.data_len, ...
                             'Value', 1, ...
                             'Position', [470, 10, 400, 15], ...
                             'Callback', @changeProgress);
  
  % 时间显示和跳转
  uicontrol('Parent', control_panel, ...
           'Style', 'text', ...
           'String', '时间(s):', ...
           'Position', [890, 10, 60, 15], ...
           'FontSize', 10);
  
  edit_time = uicontrol('Parent', control_panel, ...
                       'Style', 'edit', ...
                       'String', sprintf('%.2f', gui_data.time_data(1)), ...
                       'Position', [960, 10, 80, 15], ...
                       'FontSize', 10, ...
                       'Callback', @jumpToTime);
  
  text_total_time = uicontrol('Parent', control_panel, ...
                             'Style', 'text', ...
                             'String', sprintf('/ %.2f', gui_data.time_data(end)), ...
                             'Position', [1050, 10, 80, 15], ...
                             'FontSize', 10);
  
  % 重置按钮
  btn_reset = uicontrol('Parent', control_panel, ...
                       'Style', 'pushbutton', ...
                       'String', '重置', ...
                       'Position', [1150, 5, 60, 25], ...
                       'FontSize', 10, ...
                       'Callback', @resetAnimation);
  
  % 清除轨迹按钮
  btn_clear = uicontrol('Parent', control_panel, ...
                       'Style', 'pushbutton', ...
                       'String', '清除轨迹', ...
                       'Position', [1220, 5, 80, 25], ...
                       'FontSize', 10, ...
                       'ForegroundColor', [0.8, 0.2, 0.2], ...
                       'Callback', @clearCurrentTrajectory);
  
  % 重置视图按钮
  btn_reset_view = uicontrol('Parent', control_panel, ...
                            'Style', 'pushbutton', ...
                            'String', '重置视图', ...
                            'Position', [1310, 5, 80, 25], ...
                            'FontSize', 10, ...
                            'ForegroundColor', [0.2, 0.6, 0.2], ...
                            'Callback', @resetAllViews);
  
    %% 创建显示区域 (重新布局以容纳更多图形)
  % 轨迹显示区域 (左上)
  panel_traj = uipanel('Parent', fig_main, ...
                      'Position', [0.01, 0.67, 0.32, 0.26], ...
                      'Title', '实时轨迹 (可缩放拖拽)', ...
                      'FontSize', 11);
  
  ax_traj = axes('Parent', panel_traj, ...
                 'Position', [0.1, 0.1, 0.85, 0.8]);
  
  % 状态参数显示区域 (中上)
  panel_params = uipanel('Parent', fig_main, ...
                        'Position', [0.34, 0.67, 0.32, 0.26], ...
                        'Title', '状态参数 (可缩放拖拽)', ...
                        'FontSize', 11);
  
  % 创建参数子图 (2x2布局)
  ax_attitude = subplot(2, 2, 1, 'Parent', panel_params);
  ax_velocity = subplot(2, 2, 2, 'Parent', panel_params);
  ax_position = subplot(2, 2, 3, 'Parent', panel_params);
  ax_bias = subplot(2, 2, 4, 'Parent', panel_params);
  
    % 详细参数显示区域 (右上) - 原来是状态反馈的位置
  panel_details = uipanel('Parent', fig_main, ...
                         'Position', [0.67, 0.67, 0.32, 0.26], ...
                         'Title', '详细参数 (可缩放拖拽)', ...
                         'FontSize', 11);
  
  % 创建详细参数子图 (2x3布局)
  ax_gyro = subplot(2, 3, 1, 'Parent', panel_details);
  ax_acc = subplot(2, 3, 2, 'Parent', panel_details);
  ax_install = subplot(2, 3, 3, 'Parent', panel_details);
  ax_gyro_bias = subplot(2, 3, 4, 'Parent', panel_details);
  ax_acc_bias = subplot(2, 3, 5, 'Parent', panel_details);
  ax_od_scale = subplot(2, 3, 6, 'Parent', panel_details);
  
  % 状态反馈显示区域 (下方) - 原来是详细参数的位置
  panel_feedback = uipanel('Parent', fig_main, ...
                          'Position', [0.01, 0.01, 0.98, 0.65], ...
                          'Title', '状态反馈 log.X (可缩放拖拽)', ...
                          'FontSize', 12);
  
  % 创建状态反馈子图 (2x3布局)
  ax_feedback_att = subplot(2, 3, 1, 'Parent', panel_feedback);
  ax_feedback_vel = subplot(2, 3, 2, 'Parent', panel_feedback);
  ax_feedback_pos = subplot(2, 3, 3, 'Parent', panel_feedback);
  ax_feedback_gyrbias = subplot(2, 3, 4, 'Parent', panel_feedback);
  ax_feedback_accbias = subplot(2, 3, 5, 'Parent', panel_feedback);
  ax_feedback_reserved = subplot(2, 3, 6, 'Parent', panel_feedback);
  
  % 存储所有axes句柄用于视图重置
  gui_data.all_axes = [ax_traj, ax_attitude, ax_velocity, ax_position, ax_bias, ...
                       ax_feedback_att, ax_feedback_vel, ax_feedback_pos, ...
                       ax_feedback_gyrbias, ax_feedback_accbias, ax_feedback_reserved, ...
                       ax_gyro, ax_acc, ax_install, ax_gyro_bias, ax_acc_bias, ax_od_scale];
  
  %% 初始化图形对象
  initializePlots();
  
  %% 回调函数 (保持不变)
  function togglePlay(~, ~)
      if gui_data.is_playing
          % 暂停
          gui_data.is_playing = false;
          set(btn_play, 'String', '播放');
          if ~isempty(gui_data.timer_obj) && isvalid(gui_data.timer_obj)
              stop(gui_data.timer_obj);
          end
      else
          % 播放
          gui_data.is_playing = true;
          set(btn_play, 'String', '暂停');
          startAnimation();
      end
  end
  
  function changeSpeed(~, ~)
      speed_options = [1, 2, 5, 10, 20];
      gui_data.play_speed = speed_options(get(popup_speed, 'Value'));
      
      % 如果正在播放，重新启动定时器
      if gui_data.is_playing
          if ~isempty(gui_data.timer_obj) && isvalid(gui_data.timer_obj)
              stop(gui_data.timer_obj);
          end
          startAnimation();
      end
  end
  
  function changeProgress(~, ~)
      new_idx = round(get(slider_progress, 'Value'));
      gui_data.current_idx = max(1, min(new_idx, gui_data.data_len));
      updateDisplay();
      updateTimeDisplay();
  end
  
  function jumpTime(delta_seconds)
      current_time = gui_data.time_data(gui_data.current_idx);
      target_time = current_time + delta_seconds;
      
      % 找到最接近目标时间的索引
      [~, new_idx] = min(abs(gui_data.time_data - target_time));
      gui_data.current_idx = max(1, min(new_idx, gui_data.data_len));
      
      updateDisplay();
      updateTimeDisplay();
      set(slider_progress, 'Value', gui_data.current_idx);
  end
  
  function jumpToTime(~, ~)
      target_time = str2double(get(edit_time, 'String'));
      if ~isnan(target_time)
          [~, new_idx] = min(abs(gui_data.time_data - target_time));
          gui_data.current_idx = max(1, min(new_idx, gui_data.data_len));
          updateDisplay();
          set(slider_progress, 'Value', gui_data.current_idx);
      end
      updateTimeDisplay();
  end
  
  function resetAnimation(~, ~)
      % 停止播放
      gui_data.is_playing = false;
      set(btn_play, 'String', '播放');
      if ~isempty(gui_data.timer_obj) && isvalid(gui_data.timer_obj)
          stop(gui_data.timer_obj);
      end
      
      % 重置到开始
      gui_data.current_idx = 1;
      set(slider_progress, 'Value', 1);
      updateTimeDisplay();
      
      % 清除图形并重新初始化
      clearPlots();
      initializePlots();
  end
  
  % 清除当前轨迹函数
  function clearCurrentTrajectory(~, ~)
      % 显示确认对话框
      answer = questdlg('确定要清除当前显示的轨迹吗？', ...
                       '清除轨迹确认', ...
                       '确定', '取消', '取消');
      
      if strcmp(answer, '确定')
          % 暂停播放
          if gui_data.is_playing
              gui_data.is_playing = false;
              set(btn_play, 'String', '播放');
              if ~isempty(gui_data.timer_obj) && isvalid(gui_data.timer_obj)
                  stop(gui_data.timer_obj);
              end
          end
          
          % 清除所有动态轨迹，但保持当前位置
          clearCurrentTrajectoryOnly();
          
          % 显示清除完成消息
          msgbox('轨迹已清除！可以继续播放或跳转到其他时间点。', ...
                 '清除完成', 'help');
      end
  end
  
  % 重置所有视图函数
  function resetAllViews(~, ~)
      % 重置所有axes的视图
      for i = 1:length(gui_data.all_axes)
          if ishandle(gui_data.all_axes(i))
              axis(gui_data.all_axes(i), 'auto');
              if gui_data.all_axes(i) == ax_traj
                  axis(gui_data.all_axes(i), 'equal');
              end
          end
      end
      
      % 刷新显示
      drawnow;
      
      % 显示提示消息
      msgbox('所有图形视图已重置为自动缩放！', '视图重置', 'help');
  end
  
  function startAnimation()
      if ~isempty(gui_data.timer_obj) && isvalid(gui_data.timer_obj)
          stop(gui_data.timer_obj);
          delete(gui_data.timer_obj);
      end
      
      % 计算定时器间隔 (基础间隔50ms)
      timer_interval = 0.05 / gui_data.play_speed;
      
      gui_data.timer_obj = timer('ExecutionMode', 'fixedRate', ...
                                'Period', timer_interval, ...
                                'TimerFcn', @animationStep);
      start(gui_data.timer_obj);
  end
  
  function animationStep(~, ~)
      if gui_data.current_idx >= gui_data.data_len
          % 播放完成
          gui_data.is_playing = false;
          set(btn_play, 'String', '播放');
          stop(gui_data.timer_obj);
          return;
      end
      
      gui_data.current_idx = gui_data.current_idx + gui_data.play_speed;
      gui_data.current_idx = min(gui_data.current_idx, gui_data.data_len);
      
      updateDisplay();
      updateTimeDisplay();
      set(slider_progress, 'Value', gui_data.current_idx);
  end
  
  function updateTimeDisplay()
      current_time = gui_data.time_data(gui_data.current_idx);
      set(edit_time, 'String', sprintf('%.2f', current_time));
  end
  
  function closeGUI(~, ~)
      if ~isempty(gui_data.timer_obj) && isvalid(gui_data.timer_obj)
          stop(gui_data.timer_obj);
          delete(gui_data.timer_obj);
      end
      delete(fig_main);
  end
  
  %% 图形初始化和更新函数
  function initializePlots()
      % 轨迹图初始化
      axes(ax_traj);
      cla(ax_traj);
      hold(ax_traj, 'on');
      
      % 绘制GNSS轨迹（作为背景，更浅的颜色）
      if isfield(gui_data.data, 'gnss') && isfield(gui_data.data.gnss, 'pos_enu')
          plot(ax_traj, gui_data.data.gnss.pos_enu(:,1), gui_data.data.gnss.pos_enu(:,2), ...
               'Color', [0.9 0.9 0.9], 'LineWidth', 1.5);
      end
      
      % 当前轨迹线
      gui_data.h_traj = animatedline(ax_traj, 'Color', '#D95319', 'LineWidth', 3);
      
      % 当前位置点
      gui_data.h_current = plot(ax_traj, gui_data.log.pos(1,1), gui_data.log.pos(1,2), ...
                               'ro', 'MarkerSize', 12, 'MarkerFaceColor', 'red');
      
      % 起点和终点
      plot(ax_traj, gui_data.log.pos(1,1), gui_data.log.pos(1,2), ...
           'go', 'MarkerSize', 10, 'MarkerFaceColor', 'green');
      plot(ax_traj, gui_data.log.pos(end,1), gui_data.log.pos(end,2), ...
           'bs', 'MarkerSize', 10, 'MarkerFaceColor', 'blue');
      
      xlabel(ax_traj, '东向 (m)');
      ylabel(ax_traj, '北向 (m)');
      title(ax_traj, '实时轨迹');
      grid(ax_traj, 'on');
      axis(ax_traj, 'equal');
      
      % 更新图例
      if isfield(gui_data.data, 'gnss') && isfield(gui_data.data.gnss, 'pos_enu')
          legend(ax_traj, {'GNSS轨迹', '当前轨迹', '当前位置', '起点', '终点'}, ...
                 'Location', 'best');
      else
          legend(ax_traj, {'当前轨迹', '当前位置', '起点', '终点'}, ...
                 'Location', 'best');
      end
      
      % 启用缩放和平移
      enableAxisInteraction(ax_traj);
      
      % 初始化参数图
      initializeParamPlots();
      
      % 初始化状态反馈图
      initializeFeedbackPlots();
      
      % 初始显示
      updateDisplay();
  end
  
  function initializeParamPlots()
      % 姿态角
      axes(ax_attitude);
      cla(ax_attitude);
      hold(ax_attitude, 'on');
      gui_data.h_roll = animatedline(ax_attitude, 'Color', '#0072BD', 'LineWidth', 2);
      gui_data.h_pitch = animatedline(ax_attitude, 'Color', '#D95319', 'LineWidth', 2);
      gui_data.h_yaw = animatedline(ax_attitude, 'Color', '#EDB120', 'LineWidth', 2);
      xlabel(ax_attitude, '                                  时间 (s)');
      ylabel(ax_attitude, '角度 (deg)');
      title(ax_attitude, '姿态角');
      legend(ax_attitude, {'Roll', 'Pitch', 'Yaw'}, 'Location', 'best');
      grid(ax_attitude, 'on');
      enableAxisInteraction(ax_attitude);
      
      % 速度
      axes(ax_velocity);
      cla(ax_velocity);
      hold(ax_velocity, 'on');
      gui_data.h_vel_e = animatedline(ax_velocity, 'Color', '#0072BD', 'LineWidth', 2);
      gui_data.h_vel_n = animatedline(ax_velocity, 'Color', '#D95319', 'LineWidth', 2);
      gui_data.h_vel_u = animatedline(ax_velocity, 'Color', '#EDB120', 'LineWidth', 2);
      xlabel(ax_velocity, '                                  时间 (s)');
      ylabel(ax_velocity, '速度 (m/s)');
      title(ax_velocity, '速度');
      legend(ax_velocity, {'East', 'North', 'Up'}, 'Location', 'best');
      grid(ax_velocity, 'on');
      enableAxisInteraction(ax_velocity);
      
      % 位置
      axes(ax_position);
      cla(ax_position);
      hold(ax_position, 'on');
      gui_data.h_pos_e = animatedline(ax_position, 'Color', '#0072BD', 'LineWidth', 2);
      gui_data.h_pos_n = animatedline(ax_position, 'Color', '#D95319', 'LineWidth', 2);
      gui_data.h_pos_u = animatedline(ax_position, 'Color', '#EDB120', 'LineWidth', 2);
      xlabel(ax_position, '                                  时间 (s)');
      ylabel(ax_position, '位置 (m)');
      title(ax_position, '位置');
      legend(ax_position, {'East', 'North', 'Up'}, 'Location', 'best');
      grid(ax_position, 'on');
      enableAxisInteraction(ax_position);
      
      % 修改为GNSS状态显示
      axes(ax_bias);
      cla(ax_bias);
      hold(ax_bias, 'on');
      gui_data.h_gyr_bias = animatedline(ax_bias, 'Color', '#0072BD', 'LineWidth', 2);
      gui_data.h_acc_bias = animatedline(ax_bias, 'Color', '#D95319', 'LineWidth', 2, 'LineStyle', '--');
      xlabel(ax_bias, '                                  时间 (s)');
      ylabel(ax_bias, '状态值');
      title(ax_bias, 'GNSS状态');
      legend(ax_bias, {'定位状态', '定向状态'}, 'Location', 'best');
      grid(ax_bias, 'on');
      enableAxisInteraction(ax_bias);

      
      % 详细参数图初始化
      initializeDetailPlots();
  end
  
  % 新增：初始化状态反馈图
    % 修改：初始化状态反馈图
  function initializeFeedbackPlots()
      % 状态反馈姿态角 (log.X(1:3))
      axes(ax_feedback_att);
      cla(ax_feedback_att);
      hold(ax_feedback_att, 'on');
      gui_data.h_fb_att_x = animatedline(ax_feedback_att, 'Color', '#0072BD', 'LineWidth', 2);
      gui_data.h_fb_att_y = animatedline(ax_feedback_att, 'Color', '#D95319', 'LineWidth', 2);
      gui_data.h_fb_att_z = animatedline(ax_feedback_att, 'Color', '#EDB120', 'LineWidth', 2);
      xlabel(ax_feedback_att, '                                  时间 (s)');
      ylabel(ax_feedback_att, '角度 (deg)');
      title(ax_feedback_att, '姿态反馈');
      legend(ax_feedback_att, {'X', 'Y', 'Z'}, 'Location', 'best');
      grid(ax_feedback_att, 'on');
      enableAxisInteraction(ax_feedback_att);
      
      % 状态反馈速度 (log.X(4:6))
      axes(ax_feedback_vel);
      cla(ax_feedback_vel);
      hold(ax_feedback_vel, 'on');
      gui_data.h_fb_vel_x = animatedline(ax_feedback_vel, 'Color', '#0072BD', 'LineWidth', 2);
      gui_data.h_fb_vel_y = animatedline(ax_feedback_vel, 'Color', '#D95319', 'LineWidth', 2);
      gui_data.h_fb_vel_z = animatedline(ax_feedback_vel, 'Color', '#EDB120', 'LineWidth', 2);
      xlabel(ax_feedback_vel, '                                  时间 (s)');
      ylabel(ax_feedback_vel, '速度 (m/s)');
      title(ax_feedback_vel, '速度反馈');
      legend(ax_feedback_vel, {'X', 'Y', 'Z'}, 'Location', 'best');
      grid(ax_feedback_vel, 'on');
      enableAxisInteraction(ax_feedback_vel);
      
      % 状态反馈位置 (log.X(7:9))
      axes(ax_feedback_pos);
      cla(ax_feedback_pos);
      hold(ax_feedback_pos, 'on');
      gui_data.h_fb_pos_x = animatedline(ax_feedback_pos, 'Color', '#0072BD', 'LineWidth', 2);
      gui_data.h_fb_pos_y = animatedline(ax_feedback_pos, 'Color', '#D95319', 'LineWidth', 2);
      gui_data.h_fb_pos_z = animatedline(ax_feedback_pos, 'Color', '#EDB120', 'LineWidth', 2);
      xlabel(ax_feedback_pos, '                                  时间 (s)');
      ylabel(ax_feedback_pos, '位置 (m)');
      title(ax_feedback_pos, '位置反馈');
      legend(ax_feedback_pos, {'X', 'Y', 'Z'}, 'Location', 'best');
      grid(ax_feedback_pos, 'on');
      enableAxisInteraction(ax_feedback_pos);
      
      % 新增：状态反馈陀螺零偏 (log.X(10:12))
      axes(ax_feedback_gyrbias);
      cla(ax_feedback_gyrbias);
      hold(ax_feedback_gyrbias, 'on');
      gui_data.h_fb_gyrbias_x = animatedline(ax_feedback_gyrbias, 'Color', '#0072BD', 'LineWidth', 2);
      gui_data.h_fb_gyrbias_y = animatedline(ax_feedback_gyrbias, 'Color', '#D95319', 'LineWidth', 2);
      gui_data.h_fb_gyrbias_z = animatedline(ax_feedback_gyrbias, 'Color', '#EDB120', 'LineWidth', 2);
      xlabel(ax_feedback_gyrbias, '                                  时间 (s)');
      ylabel(ax_feedback_gyrbias, '零偏 (deg/h)');
      title(ax_feedback_gyrbias, '陀螺零偏反馈');
      legend(ax_feedback_gyrbias, {'X', 'Y', 'Z'}, 'Location', 'best');
      grid(ax_feedback_gyrbias, 'on');
      enableAxisInteraction(ax_feedback_gyrbias);
      
      % 新增：状态反馈加计零偏 (log.X(13:15))
      axes(ax_feedback_accbias);
      cla(ax_feedback_accbias);
      hold(ax_feedback_accbias, 'on');
      gui_data.h_fb_accbias_x = animatedline(ax_feedback_accbias, 'Color', '#0072BD', 'LineWidth', 2);
      gui_data.h_fb_accbias_y = animatedline(ax_feedback_accbias, 'Color', '#D95319', 'LineWidth', 2);
      gui_data.h_fb_accbias_z = animatedline(ax_feedback_accbias, 'Color', '#EDB120', 'LineWidth', 2);
      xlabel(ax_feedback_accbias, '                                  时间 (s)');
      ylabel(ax_feedback_accbias, '零偏 (mg)');
      title(ax_feedback_accbias, '加计零偏反馈');
      legend(ax_feedback_accbias, {'X', 'Y', 'Z'}, 'Location', 'best');
      grid(ax_feedback_accbias, 'on');
      enableAxisInteraction(ax_feedback_accbias);
      
      % 预留图形区域 (可用于其他参数)
      axes(ax_feedback_reserved);
      cla(ax_feedback_reserved);
      title(ax_feedback_reserved, '预留区域');
      set(ax_feedback_reserved, 'XTick', [], 'YTick', []);
      enableAxisInteraction(ax_feedback_reserved);
  end
  
  function initializeDetailPlots()
      % 陀螺仪
      axes(ax_gyro);
      cla(ax_gyro);
      hold(ax_gyro, 'on');
      gui_data.h_gyro_x = animatedline(ax_gyro, 'Color', '#0072BD', 'LineWidth', 2);
      gui_data.h_gyro_y = animatedline(ax_gyro, 'Color', '#D95319', 'LineWidth', 2);
      gui_data.h_gyro_z = animatedline(ax_gyro, 'Color', '#EDB120', 'LineWidth', 2);
      xlabel(ax_gyro, '                                  时间 (s)');
      ylabel(ax_gyro, '角速度 (deg/s)');
      title(ax_gyro, '陀螺仪');
      legend(ax_gyro, {'X', 'Y', 'Z'}, 'Location', 'best');
      grid(ax_gyro, 'on');
      enableAxisInteraction(ax_gyro);
      
      % 加速度计
      axes(ax_acc);
      cla(ax_acc);
      hold(ax_acc, 'on');
      gui_data.h_acc_x = animatedline(ax_acc, 'Color', '#0072BD', 'LineWidth', 2);
      gui_data.h_acc_y = animatedline(ax_acc, 'Color', '#D95319', 'LineWidth', 2);
      gui_data.h_acc_z = animatedline(ax_acc, 'Color', '#EDB120', 'LineWidth', 2);
      xlabel(ax_acc, '                                  时间 (s)');
      ylabel(ax_acc, '加速度 (m/s²)');
      title(ax_acc, '加速度计');
      legend(ax_acc, {'X', 'Y', 'Z'}, 'Location', 'best');
      grid(ax_acc, 'on');
      enableAxisInteraction(ax_acc);
      % 安装角
      axes(ax_install);
      cla(ax_install);
      hold(ax_install, 'on');
      gui_data.h_install_p = animatedline(ax_install, 'Color', '#0072BD', 'LineWidth', 2);
      gui_data.h_install_r = animatedline(ax_install, 'Color', '#D95319', 'LineWidth', 2);
      gui_data.h_install_y = animatedline(ax_install, 'Color', '#EDB120', 'LineWidth', 2);
      xlabel(ax_install, '                                  时间 (s)');
      ylabel(ax_install, '角度 (deg)');
      title(ax_install, '安装角');
      legend(ax_install, {'Pitch', 'Roll', 'Yaw'}, 'Location', 'best');
      grid(ax_install, 'on');
      enableAxisInteraction(ax_install);
      
      % 陀螺零偏
      axes(ax_gyro_bias);
      cla(ax_gyro_bias);
      hold(ax_gyro_bias, 'on');
      gui_data.h_gbias_x = animatedline(ax_gyro_bias, 'Color', '#0072BD', 'LineWidth', 2);
      gui_data.h_gbias_y = animatedline(ax_gyro_bias, 'Color', '#D95319', 'LineWidth', 2);
      gui_data.h_gbias_z = animatedline(ax_gyro_bias, 'Color', '#EDB120', 'LineWidth', 2);
      xlabel(ax_gyro_bias, '                                  时间 (s)');
      ylabel(ax_gyro_bias, '零偏 (deg/h)');
      title(ax_gyro_bias, '陀螺零偏');
      legend(ax_gyro_bias, {'X', 'Y', 'Z'}, 'Location', 'best');
      grid(ax_gyro_bias, 'on');
      enableAxisInteraction(ax_gyro_bias);
      
      % 加计零偏
      axes(ax_acc_bias);
      cla(ax_acc_bias);
      hold(ax_acc_bias, 'on');
      gui_data.h_abias_x = animatedline(ax_acc_bias, 'Color', '#0072BD', 'LineWidth', 2);
      gui_data.h_abias_y = animatedline(ax_acc_bias, 'Color', '#D95319', 'LineWidth', 2);
      gui_data.h_abias_z = animatedline(ax_acc_bias, 'Color', '#EDB120', 'LineWidth', 2);
      xlabel(ax_acc_bias, '                                  时间 (s)');
      ylabel(ax_acc_bias, '零偏 (mg)');
      title(ax_acc_bias, '加计零偏');
      legend(ax_acc_bias, {'X', 'Y', 'Z'}, 'Location', 'best');
      grid(ax_acc_bias, 'on');
      enableAxisInteraction(ax_acc_bias);
      
      % 里程计比例因子
      axes(ax_od_scale);
      cla(ax_od_scale);
      gui_data.h_od_scale = animatedline(ax_od_scale, 'Color', '#7E2F8E', 'LineWidth', 2);
      xlabel(ax_od_scale, '                                  时间 (s)');
      ylabel(ax_od_scale, '比例因子');
      title(ax_od_scale, '里程计比例因子');
      grid(ax_od_scale, 'on');
      enableAxisInteraction(ax_od_scale);
  end
  
  % 启用坐标轴交互功能
  function enableAxisInteraction(ax)
      % 启用数据提示、缩放和平移
      datacursormode(fig_main, 'on');
      zoom(ax, 'on');
      pan(ax, 'on');
      
      % 添加右键菜单用于坐标轴操作
      cmenu = uicontextmenu(fig_main);
      uimenu(cmenu, 'Label', '自动缩放', 'Callback', @(~,~)axis(ax, 'auto'));
      uimenu(cmenu, 'Label', '紧凑显示', 'Callback', @(~,~)axis(ax, 'tight'));
      uimenu(cmenu, 'Label', '网格开/关', 'Callback', @(~,~)grid(ax));
      if ax == ax_traj
          uimenu(cmenu, 'Label', '等比例显示', 'Callback', @(~,~)axis(ax, 'equal'));
      end
      set(ax, 'UIContextMenu', cmenu);
  end
  
  function updateDisplay()
      i = gui_data.current_idx;
      
      % 更新轨迹
      addpoints(gui_data.h_traj, gui_data.log.pos(i,1), gui_data.log.pos(i,2));
      set(gui_data.h_current, 'XData', gui_data.log.pos(i,1), 'YData', gui_data.log.pos(i,2));
      
      % 更新参数图
      current_time = gui_data.time_data(i);
      
      % 姿态角
      addpoints(gui_data.h_roll, current_time, gui_data.log.roll(i));
      addpoints(gui_data.h_pitch, current_time, gui_data.log.pitch(i));
      addpoints(gui_data.h_yaw, current_time, gui_data.log.yaw(i));
      
      % 速度
      addpoints(gui_data.h_vel_e, current_time, gui_data.log.vel(i,1));
      addpoints(gui_data.h_vel_n, current_time, gui_data.log.vel(i,2));
      addpoints(gui_data.h_vel_u, current_time, gui_data.log.vel(i,3));
      
      % 位置
      addpoints(gui_data.h_pos_e, current_time, gui_data.log.pos(i,1));
      addpoints(gui_data.h_pos_n, current_time, gui_data.log.pos(i,2));
      addpoints(gui_data.h_pos_u, current_time, gui_data.log.pos(i,3));
      
      % 新增：GNSS定位状态和定向状态
      if isfield(gui_data.data, 'gnss') && isfield(gui_data.data.gnss, 'solq_pos2')
          addpoints(gui_data.h_gyr_bias, current_time, gui_data.data.gnss.solq_pos2(i));
      else
          addpoints(gui_data.h_gyr_bias, current_time, 0);
      end
      
      if isfield(gui_data.data, 'gnss') && isfield(gui_data.data.gnss, 'solq_heading2')
          addpoints(gui_data.h_acc_bias, current_time, gui_data.data.gnss.solq_heading2(i));
      else
          addpoints(gui_data.h_acc_bias, current_time, 0);
      end

            % 修改：更新状态反馈图
      % 姿态反馈 (log.X(1:3), 弧度转度)
      addpoints(gui_data.h_fb_att_x, current_time, gui_data.log.X(i,1) * R2D);
      addpoints(gui_data.h_fb_att_y, current_time, gui_data.log.X(i,2) * R2D);
      addpoints(gui_data.h_fb_att_z, current_time, gui_data.log.X(i,3) * R2D);
      
      % 速度反馈 (log.X(4:6), m/s)
      addpoints(gui_data.h_fb_vel_x, current_time, gui_data.log.X(i,4));
      addpoints(gui_data.h_fb_vel_y, current_time, gui_data.log.X(i,5));
      addpoints(gui_data.h_fb_vel_z, current_time, gui_data.log.X(i,6));
      
      % 位置反馈 (log.X(7:9), m)
      addpoints(gui_data.h_fb_pos_x, current_time, gui_data.log.X(i,7));
      addpoints(gui_data.h_fb_pos_y, current_time, gui_data.log.X(i,8));
      addpoints(gui_data.h_fb_pos_z, current_time, gui_data.log.X(i,9));
      
      % 新增：陀螺零偏反馈 (log.X(10:12), 弧度转度/小时)
      addpoints(gui_data.h_fb_gyrbias_x, current_time, gui_data.log.X(i,10) * R2D * 3600);
      addpoints(gui_data.h_fb_gyrbias_y, current_time, gui_data.log.X(i,11) * R2D * 3600);
      addpoints(gui_data.h_fb_gyrbias_z, current_time, gui_data.log.X(i,12) * R2D * 3600);
      
      % 新增：加计零偏反馈 (log.X(13:15), m/s²转mg)
      addpoints(gui_data.h_fb_accbias_x, current_time, gui_data.log.X(i,13) * 1000 / GRAVITY);
      addpoints(gui_data.h_fb_accbias_y, current_time, gui_data.log.X(i,14) * 1000 / GRAVITY);
      addpoints(gui_data.h_fb_accbias_z, current_time, gui_data.log.X(i,15) * 1000 / GRAVITY);
      
      % 详细参数
      % 陀螺仪
      addpoints(gui_data.h_gyro_x, current_time, gui_data.data.imu.gyr(i,1) * R2D);
      addpoints(gui_data.h_gyro_y, current_time, gui_data.data.imu.gyr(i,2) * R2D);
      addpoints(gui_data.h_gyro_z, current_time, gui_data.data.imu.gyr(i,3) * R2D);
      
      % 加速度计
      addpoints(gui_data.h_acc_x, current_time, gui_data.data.imu.acc(i,1));
      addpoints(gui_data.h_acc_y, current_time, gui_data.data.imu.acc(i,2));
      addpoints(gui_data.h_acc_z, current_time, gui_data.data.imu.acc(i,3));
      
      % 安装角
      addpoints(gui_data.h_install_p, current_time, gui_data.log.installangle(i,1));
      addpoints(gui_data.h_install_r, current_time, gui_data.log.installangle(i,2));
      addpoints(gui_data.h_install_y, current_time, gui_data.log.installangle(i,3));
      
      % 陀螺零偏
      addpoints(gui_data.h_gbias_x, current_time, gui_data.log.gyr_bias(i,1) * R2D * 3600);
      addpoints(gui_data.h_gbias_y, current_time, gui_data.log.gyr_bias(i,2) * R2D * 3600);
      addpoints(gui_data.h_gbias_z, current_time, gui_data.log.gyr_bias(i,3) * R2D * 3600);
      
      % 加计零偏
      addpoints(gui_data.h_abias_x, current_time, gui_data.log.acc_bias(i,1) * 1000 / GRAVITY);
      addpoints(gui_data.h_abias_y, current_time, gui_data.log.acc_bias(i,2) * 1000 / GRAVITY);
      addpoints(gui_data.h_abias_z, current_time, gui_data.log.acc_bias(i,3) * 1000 / GRAVITY);
      
      % 里程计比例因子
      addpoints(gui_data.h_od_scale, current_time, gui_data.log.od_scale_factor(i,1));
      
      drawnow limitrate;
  end
  
  function clearPlots()
      % 清除所有animatedline
      fields = fieldnames(gui_data);
      for k = 1:length(fields)
          if startsWith(fields{k}, 'h_') && ishandle(gui_data.(fields{k}))
              clearpoints(gui_data.(fields{k}));
          end
      end
  end
  
    % 只清除当前轨迹的函数
  function clearCurrentTrajectoryOnly()
      % 清除轨迹图中的动态轨迹线，但保留背景轨迹和标记点
      if isfield(gui_data, 'h_traj') && ishandle(gui_data.h_traj)
          clearpoints(gui_data.h_traj);
      end
      
      % 清除所有参数图的动态线条
      param_handles = {'h_roll', 'h_pitch', 'h_yaw', ...
                      'h_vel_e', 'h_vel_n', 'h_vel_u', ...
                      'h_pos_e', 'h_pos_n', 'h_pos_u', ...
                      'h_gyr_bias', 'h_acc_bias', ...
                      'h_fb_att_x', 'h_fb_att_y', 'h_fb_att_z', ...
                      'h_fb_vel_x', 'h_fb_vel_y', 'h_fb_vel_z', ...
                      'h_fb_pos_x', 'h_fb_pos_y', 'h_fb_pos_z', ...
                      'h_fb_gyrbias_x', 'h_fb_gyrbias_y', 'h_fb_gyrbias_z', ...
                      'h_fb_accbias_x', 'h_fb_accbias_y', 'h_fb_accbias_z', ...
                      'h_gyro_x', 'h_gyro_y', 'h_gyro_z', ...
                      'h_acc_x', 'h_acc_y', 'h_acc_z', ...
                      'h_install_p', 'h_install_r', 'h_install_y', ...
                      'h_gbias_x', 'h_gbias_y', 'h_gbias_z', ...
                      'h_abias_x', 'h_abias_y', 'h_abias_z', ...
                      'h_od_scale'};
      
      for i = 1:length(param_handles)
          handle_name = param_handles{i};
          if isfield(gui_data, handle_name) && ishandle(gui_data.(handle_name))
              clearpoints(gui_data.(handle_name));
          end
      end
      
      % 更新当前位置标记到当前索引位置
      if isfield(gui_data, 'h_current') && ishandle(gui_data.h_current)
          current_idx = gui_data.current_idx;
          set(gui_data.h_current, 'XData', gui_data.log.pos(current_idx,1), ...
                                  'YData', gui_data.log.pos(current_idx,2));
      end
      
      % 刷新显示
      drawnow;
  end
  
end
