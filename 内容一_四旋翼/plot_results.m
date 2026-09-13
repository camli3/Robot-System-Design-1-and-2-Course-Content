function met = plot_results(log, par)
% 轨迹/误差/姿态/角速度/控制量，并计算指标

if ~exist('report_figs', 'dir')
    mkdir('report_figs');
end
tag = sprintf('e%d_a%d', par.exp_id, par.algo);
hide = isfield(par, 'hide_fig') && par.hide_fig;
vis = 'off';
if ~hide
    vis = 'on';
end
outdir = 'report_figs';

t = log.t;
p = log.x(1:3, :);
eta = log.x(7:9, :);
omega = log.x(10:12, :);

h = figure('Name', '三维轨迹', 'Color', 'w', 'Visible', vis);
plot3(log.ref(1,:), log.ref(2,:), log.ref(3,:), 'b--', 'LineWidth', 1.2);
hold on;
plot3(p(1,:), p(2,:), p(3,:), 'r', 'LineWidth', 1.4);
plot3(log.ref(1,1), log.ref(2,1), log.ref(3,1), 'go', 'MarkerFaceColor', 'g');
grid on; axis equal;
xlabel('x (m)'); ylabel('y (m)'); zlabel('z (m)');
legend('期望', '实际', '起点');
title('三维轨迹跟踪');
saveas(h, fullfile(outdir, [tag '_traj3d.png']));
if hide, close(h); end

h = figure('Name', '位置误差', 'Color', 'w', 'Visible', vis);
plot(t, log.e(1,:), t, log.e(2,:), t, log.e(3,:), 'LineWidth', 1.1);
grid on; xlabel('t (s)'); ylabel('e (m)');
legend('e_x', 'e_y', 'e_z');
title('位置误差');
saveas(h, fullfile(outdir, [tag '_pos_err.png']));
if hide, close(h); end

h = figure('Name', '姿态', 'Color', 'w', 'Visible', vis);
plot(t, rad2deg(eta(1,:)), t, rad2deg(eta(2,:)), t, rad2deg(eta(3,:)));
hold on;
plot(t, rad2deg(log.att_d(1,:)), '--', t, rad2deg(log.att_d(2,:)), '--');
grid on; xlabel('t (s)'); ylabel('deg');
legend('\phi', '\theta', '\psi', '\phi_d', '\theta_d');
title('姿态角');
saveas(h, fullfile(outdir, [tag '_att.png']));
if hide, close(h); end

h = figure('Name', '角速度', 'Color', 'w', 'Visible', vis);
plot(t, omega(1,:), t, omega(2,:), t, omega(3,:));
grid on; xlabel('t (s)'); ylabel('rad/s');
legend('p', 'q', 'r');
title('体轴角速度（看超调/发散）');
saveas(h, fullfile(outdir, [tag '_rate.png']));
if hide, close(h); end

h = figure('Name', '控制量', 'Color', 'w', 'Visible', vis);
plot(t, log.U(1,:), t, log.U(2,:), t, log.U(3,:), t, log.U(4,:));
grid on; xlabel('t (s)'); ylabel('U');
legend('U_1', 'U_2', 'U_3', 'U_4');
title('控制输入');
saveas(h, fullfile(outdir, [tag '_u.png']));
if hide, close(h); end

met.rmse = sqrt(mean(sum(log.e.^2, 1)));
met.ess = mean(vecnorm(log.e(:, round(0.8*end):end), 2, 1));
met.max_rate = max(vecnorm(omega, 2, 1));
met.max_tilt = max(max(abs(eta(1:2, :))));
fprintf('RMSE=%.4f m  末端均值=%.4f m  max|ω|=%.3f  max倾角=%.2f deg\n', ...
    met.rmse, met.ess, met.max_rate, rad2deg(met.max_tilt));
end
