classdef aranovskiy_freq_est < handle
    % FrequencyEstimator 基于Aranovskiy方法的频率估计器类
    
    properties (Access = private)
        % 基本参数
        dt          % 采样时间间隔
        omega_up    % 角频率上限
        
        % 估计器参数
        a           % 观测器增益
        b           % 观测器增益
        k           % 自适应增益
        
        % 状态变量
        x1_hat      % 状态估计
        theta       % theta参数
        sigma       % sigma参数
    end
    
    methods
        function obj = aranovskiy_freq_est(fs, f_up)
            % 构造函数
            % 输入:
            %   fs: 采样频率 (Hz)
            %   f_up: 假设的频率上限 (Hz)
            
            % 基本参数初始化
            obj.dt = 1/fs;
            obj.omega_up = 2*pi*f_up;
            
            % 估计器参数设置
            obj.a = obj.omega_up;
            obj.b = obj.a;
            obj.k = 25;
            
            % 状态初始化
            obj.x1_hat = 0;
            obj.theta = -(obj.omega_up^2/4);
            obj.sigma = obj.theta;
        end
        
        function f_est = update(obj, y)
            % 更新频率估计
            % 输入:
            %   y: 当前测量值
            % 输出:
            %   f_est: 估计频率 (Hz)
            
            % 状态观测器更新
            x1_dot = -obj.a * obj.x1_hat + obj.b * y;
            x1_hat_new = obj.x1_hat + obj.dt * x1_dot;
            
            % σ动态更新
            sigma_dot = -obj.k * obj.x1_hat^2 * obj.theta ...
                       - obj.k * obj.a * obj.x1_hat * x1_dot ...
                       - obj.k * obj.b * x1_dot * y;
            sigma_new = obj.sigma + obj.dt * sigma_dot;
            
            % θ更新
            theta_new = sigma_new + obj.k * obj.b * x1_hat_new * y;
            
            % 频率估计
            omega_est = sqrt(abs(theta_new));
            f_est = omega_est / (2*pi);
            
            % 更新状态
            obj.x1_hat = x1_hat_new;
            obj.theta = theta_new;
            obj.sigma = sigma_new;
        end
        
        % 可选：添加参数获取/设置方法
        function params = getParameters(obj)
            % 获取当前参数
            params.dt = obj.dt;
            params.omega_up = obj.omega_up;
            params.a = obj.a;
            params.b = obj.b;
            params.k = obj.k;
        end
        
        function setState(obj, x1_hat, theta, sigma)
            % 手动设置状态（如果需要的话）
            obj.x1_hat = x1_hat;
            obj.theta = theta;
            obj.sigma = sigma;
        end
    end
end
