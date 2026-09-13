function log = simulate(par)
% 定步长 RK4 闭环仿真（与 Simulink ode4 一致）

dt = par.dt;
N = max(2, ceil(par.traj.t_end / dt));
x = par.x0(:);
mem = ctrl_init();

log.t = zeros(1, N);
log.x = zeros(12, N);
log.U = zeros(4, N);
log.ref = zeros(3, N);
log.att_d = zeros(3, N);
log.e = zeros(3, N);

for k = 1:N
    t = (k - 1) * dt;
    ref = traj_eval(t, par);
    [U, att_d, mem] = controller(t, x, ref, mem, par);

    k1 = plant_rhs(t, x, U, par);
    k2 = plant_rhs(t + dt/2, x + dt/2*k1, U, par);
    k3 = plant_rhs(t + dt/2, x + dt/2*k2, U, par);
    k4 = plant_rhs(t + dt, x + dt*k3, U, par);
    x = x + dt/6 * (k1 + 2*k2 + 2*k3 + k4);
    x(7:9) = [x(7); x(8); wrap_angle(x(9))];

    if x(3) < 0                          % 简易地面约束
        x(3) = 0;
        x(6) = max(x(6), 0);
    end

    log.t(k) = t;
    log.x(:, k) = x;
    log.U(:, k) = U;
    log.ref(:, k) = ref.p;
    log.att_d(:, k) = att_d;
    log.e(:, k) = ref.p - x(1:3);
end
end
