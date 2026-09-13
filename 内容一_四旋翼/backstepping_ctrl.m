function [U, att_d, mem] = backstepping_ctrl(~, x, ref, mem, par)
% 位置/姿态两级反步，虚控量用一阶滤波求导

p = x(1:3);
v = x(4:6);
eta = x(7:9);
omega = x(10:12);

e1 = p - ref.p;                          % 位置误差
vd = ref.v - par.bs_c1 * e1;             % 虚拟速度
e2 = v - vd;                             % 速度误差
a_cmd = ref.a - par.bs_c1 * (v - ref.v) - par.bs_c2 * e2 - e1;
a_cmd(1:2) = sat(a_cmd(1:2), -3.5, 3.5);
a_cmd(3) = sat(a_cmd(3), -5.0, 5.0);

[U1, phi_d, theta_d] = acc_to_thrust_att(a_cmd, ref.yaw, par);
att_d = [phi_d; theta_d; ref.yaw];

if ~mem.inited
    mem.eta_d_prev = att_d;
    mem.omega_d_prev = zeros(3, 1);
    mem.inited = true;
end

alpha = par.dt / (par.tau_filt + par.dt);
raw_eta_dot = (att_d - mem.eta_d_prev) / par.dt;
mem.eta_d_dot_f = (1 - alpha) * mem.eta_d_dot_f + alpha * raw_eta_dot;
mem.eta_d_prev = att_d;

e3 = att_d - eta;                        % 写成 eta_d - eta
e3(3) = wrap_angle(e3(3));
phi = eta(1); theta = eta(2);
W = euler_W(phi, theta);
if abs(cos(theta)) < 0.15                % 避开欧拉奇异
    W = eye(3);
end
omega_d = W \ (mem.eta_d_dot_f + par.bs_c3 * e3);
omega_d = sat(omega_d, -4, 4);

raw_w_dot = (omega_d - mem.omega_d_prev) / par.dt;
mem.omega_d_dot_f = (1 - alpha) * mem.omega_d_dot_f + alpha * raw_w_dot;
mem.omega_d_prev = omega_d;

e4 = omega - omega_d;
J = diag([par.Ix, par.Iy, par.Iz]);
tau = -par.bs_c4 * e4 + cross(omega, J * omega) + J * mem.omega_d_dot_f;
U = [U1; tau];
end
