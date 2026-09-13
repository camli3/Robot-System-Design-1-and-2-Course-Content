function [U1, phi_d, theta_d] = acc_to_thrust_att(a_cmd, yaw_d, par)
% 期望惯性加速度（不含重力）-> 总推力与期望姿态
F = par.m * (a_cmd + [0; 0; par.g]);   % 所需惯性系力
U1 = sat(norm(F), par.U1_min, par.U1_max);
if norm(F) < 1e-6
    F = [0; 0; par.m * par.g];
end
bz = F / norm(F);                      % 期望机体系 z 轴
bx_c = [cos(yaw_d); sin(yaw_d); 0];    % 偏航参考
by = cross(bz, bx_c);
if norm(by) < 1e-6
    bx_c = [0; 1; 0];
    by = cross(bz, bx_c);
end
by = by / norm(by);
bx = cross(by, bz);
Rd = [bx, by, bz];                     % 期望旋转矩阵
theta_d = asin(sat(-Rd(3, 1), -1, 1));
phi_d = atan2(Rd(3, 2), Rd(3, 3));
phi_d = sat(phi_d, -par.att_lim, par.att_lim);
theta_d = sat(theta_d, -par.att_lim, par.att_lim);
end
