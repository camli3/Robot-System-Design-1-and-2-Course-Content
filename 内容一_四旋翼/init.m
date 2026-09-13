% 内容一入口：改下面 3 行后运行本脚本
algo = 1;          % 1=串级PID  2=反步
exp_id = 1;        % 1悬停 2升降 3前后 4左右 5五航点
use_slx = 0;       % 0=本目录 RK4  1=Simulink 模型

this_dir = fileparts(mfilename('fullpath'));
cd(this_dir);
addpath(this_dir);

par = params();
par.algo = algo;
par.exp_id = exp_id;
par = traj_waypoints(par);
slx_config(par);                       % 给 Simulink 包装函数用

if use_slx
    if ~exist('Quadrotor_Control.slx', 'file')
        build_simulink();
    end
    simout = sim('Quadrotor_Control', 'StopTime', num2str(par.traj.t_end));
    log = slx_pack_log(simout);
else
    log = simulate(par);
end

met = plot_results(log, par);
save(sprintf('result_e%d_a%d.mat', exp_id, algo), 'log', 'par', 'met');
