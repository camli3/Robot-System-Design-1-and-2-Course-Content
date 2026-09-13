function par = traj_waypoints(par)
% 按实验编号生成航点序列（含随机 5 点）

switch par.exp_id
    case 1                                   % 悬停
        pts = [0, 0, 1.0, 0];
        tseg = 10;
    case 2                                   % 升降
        pts = [0, 0, 1.0, 0;
               0, 0, 2.0, 0;
               0, 0, 0.5, 0];
        tseg = [6; 6; 6];
    case 3                                   % 前后
        pts = [0, 0, 1.0, 0;
               2, 0, 1.0, 0;
               0, 0, 1.0, 0];
        tseg = [6; 6; 6];
    case 4                                   % 左右
        pts = [0, 0, 1.0, 0;
               0, 2, 1.0, 0;
               0, 0, 1.0, 0];
        tseg = [6; 6; 6];
    case 5                                   % 随机五航点
        rng(par.rng_seed);
        xy = 4 * (rand(5, 2) - 0.5);         % 平面 ±2 m
        z = 0.8 + 1.2 * rand(5, 1);          % 高度 0.8~2.0 m
        pts = [0, 0, 1.0, 0; xy, z, zeros(5, 1)];
        tseg = zeros(size(pts, 1), 1);
        prev = [0, 0, 0.05, 0];
        for i = 1:size(pts, 1)
            dist = norm(pts(i, 1:3) - prev(1:3));
            tseg(i) = sat(dist / 0.50, 4, 8);
            prev = pts(i, :);
        end
    otherwise
        error('exp_id 只能是 1~5');
end

if isscalar(tseg)
    tseg = tseg * ones(size(pts, 1), 1);
end
par.traj.pts = pts;
par.traj.tseg = tseg(:);
par.traj.t_end = sum(par.traj.tseg) + 5;     % 末段再悬停 5 s
par.traj.p0 = [0; 0; 0.05; 0];               % 与 x0 一致
end
