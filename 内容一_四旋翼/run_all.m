% 跑完全部实验 × 两种算法，写对比表（出报告图用）
this_dir = fileparts(mfilename('fullpath'));
cd(this_dir);
addpath(this_dir);

rows = {};
for exp_id = 1:5
    for algo = 1:2
        par = params();
        par.exp_id = exp_id;
        par.algo = algo;
        par = traj_waypoints(par);
        par.hide_fig = true;
        slx_config(par);
        log = simulate(par);
        met = plot_results(log, par);
        rows(end+1, :) = {exp_id, algo, met.rmse, met.ess, ...
            met.max_rate, rad2deg(met.max_tilt)}; %#ok<AGROW>
        save(sprintf('result_e%d_a%d.mat', exp_id, algo), 'log', 'par', 'met');
    end
end
T = cell2table(rows, 'VariableNames', ...
    {'exp', 'algo', 'rmse', 'ess', 'max_rate', 'max_tilt_deg'});
disp(T);
writetable(T, fullfile('report_figs', 'compare.csv'));
