function xdot = plant_rhs(~, x, U, par)
% 四旋翼 12 维 Newton-Euler 动力学
% x = [p(3); v(3); eta(3); omega(3)]
% U = [U1; U2; U3; U4]

p = x(1:3); %#ok<NASGU>
v = x(4:6);
eta = x(7:9);
omega = x(10:12);
phi = eta(1); theta = eta(2); psi = eta(3);

R = rot_zyx(phi, theta, psi);          % 机体系 -> 惯性系
F = R * [0; 0; U(1)] + par.wind - par.Dv * v;
acc = F / par.m - [0; 0; par.g];       % 惯性系加速度

W = euler_W(phi, theta);               % 欧拉运动学
eta_dot = W * omega;

J = diag([par.Ix, par.Iy, par.Iz]);
tau = U(2:4);
omega_dot = J \ (-cross(omega, J * omega) + tau - par.Dw * omega);

xdot = [v; acc; eta_dot; omega_dot];
end
