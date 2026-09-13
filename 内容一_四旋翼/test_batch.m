% 无界面自检：PID/反步各跑短悬停
this_dir = fileparts(mfilename('fullpath'));
cd(this_dir);
addpath(this_dir);

for algo = 1:2
    par = params();
    par.algo = algo;
    par.exp_id = 1;
    par = traj_waypoints(par);
    par.traj.t_end = 8;
    log = simulate(par);
    e = log.e(:, end);
    rmse = sqrt(mean(sum(log.e.^2, 1)));
    fprintf('algo=%d  end|e|=%.4f  RMSE=%.4f  z=%.3f\n', ...
        algo, norm(e), rmse, log.x(3, end));
    if norm(e) > 0.20 || any(isnan(e))
        error('algo=%d 悬停未收敛', algo);
    end
end
fprintf('test_batch OK\n');
