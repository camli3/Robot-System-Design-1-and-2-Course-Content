function p = slx_config(par_in)
% 把当前 par 存给 Simulink 包装函数（改 params 后先跑 init）
persistent par
if nargin >= 1
    par = par_in;
end
if isempty(par)
    par = params();
    par = traj_waypoints(par);
end
p = par;
end
