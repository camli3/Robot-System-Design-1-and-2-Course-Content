function par = params()
% 四旋翼物理参数与控制增益（ENU，z 轴向上，"+" 型）

% ---------- 物理参数 ----------
par.m = 1.0;                 % 质量 kg
par.g = 9.81;                % 重力 m/s^2
par.l = 0.225;               % 力臂 m
par.Ix = 0.01;               % 滚转惯量 kg*m^2
par.Iy = 0.01;               % 俯仰惯量 kg*m^2
par.Iz = 0.02;               % 偏航惯量 kg*m^2
par.b = 1.0e-5;              % 推力系数
par.d = 3.0e-7;              % 反扭矩系数
par.Dv = 0.10;               % 线阻力系数
par.Dw = 0.01;               % 角阻力系数
par.wind = [0; 0; 0];        % 风扰 N，抗扰实验再改

% ---------- 仿真 ----------
par.dt = 0.001;              % 定步长 s（ode4）
par.x0 = [0; 0; 0.05; zeros(9, 1)];  % 初值：近地面
par.rng_seed = 2026;         % 五航点可复现
par.algo = 1;                % 1=PID  2=反步
par.exp_id = 1;              % 1悬停 2升降 3前后 4左右 5五航点

% ---------- 限幅（防姿态发散） ----------
par.att_lim = deg2rad(25);   % 期望滚转/俯仰限幅
par.U1_min = 0.2 * par.m * par.g;
par.U1_max = 3.0 * par.m * par.g;
par.tau_max = 0.8;           % 力矩限幅 N*m
par.w_max = 800;             % 电机转速上限 rad/s
par.Imax_pos = 2.0;          % 位置积分限幅
par.Imax_att = 0.4;          % 姿态积分限幅
par.tau_filt = 0.03;         % 反步虚控量滤波时间常数

% ---------- 串级 PID ----------
par.Kp_pos = [2.0; 2.0; 5.0];
par.Ki_pos = [0.10; 0.10; 0.60];
par.Kd_pos = [2.4; 2.4; 3.2];
par.Kp_att = [10.0; 10.0; 4.0];
par.Ki_att = [0.20; 0.20; 0.05];
par.Kd_att = [0.80; 0.80; 0.50];

% ---------- 反步增益（内环更快） ----------
par.bs_c1 = 1.6;             % 位置误差
par.bs_c2 = 2.0;             % 速度误差
par.bs_c3 = 8.0;             % 姿态误差
par.bs_c4 = 12.0;            % 角速度误差
end
