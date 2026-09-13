function [U, att_d, mem] = pid_ctrl(~, x, ref, mem, par)
% 串级 PID：外环位置 -> 推力/期望姿态；内环姿态 PD+I

p = x(1:3);
v = x(4:6);
eta = x(7:9);
omega = x(10:12);

ep = ref.p - p;                          % 位置误差
ev = ref.v - v;                          % 速度误差（作 D）
mem.int_pos = sat(mem.int_pos + ep * par.dt, -par.Imax_pos, par.Imax_pos);

a_cmd = par.Kp_pos .* ep + par.Ki_pos .* mem.int_pos + ...
        par.Kd_pos .* ev + ref.a;
a_cmd(1:2) = sat(a_cmd(1:2), -3.5, 3.5);
a_cmd(3) = sat(a_cmd(3), -5.0, 5.0);

[U1, phi_d, theta_d] = acc_to_thrust_att(a_cmd, ref.yaw, par);
att_d = [phi_d; theta_d; ref.yaw];

ea = att_d - eta;
ea(3) = wrap_angle(ea(3));
mem.int_att = sat(mem.int_att + ea * par.dt, -par.Imax_att, par.Imax_att);

tau = par.Kp_att .* ea + par.Ki_att .* mem.int_att - par.Kd_att .* omega;
U = [U1; tau];
end
