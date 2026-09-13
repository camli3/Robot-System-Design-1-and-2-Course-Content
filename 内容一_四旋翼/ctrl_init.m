function mem = ctrl_init()
% 控制器记忆：积分项与滤波微分
mem.int_pos = zeros(3, 1);
mem.int_att = zeros(3, 1);
mem.eta_d_prev = zeros(3, 1);
mem.omega_d_prev = zeros(3, 1);
mem.eta_d_dot_f = zeros(3, 1);
mem.omega_d_dot_f = zeros(3, 1);
mem.inited = false;
end
