function y = slx_ctrl(u)
% Simulink：u = [t; x(12)] -> [U(4); att_d(3)]
persistent mem
if isempty(mem)
    mem = ctrl_init();
end
t = u(1);
if t < 1e-9
    mem = ctrl_init();
end
x = u(2:13);
par = slx_config();
ref = traj_eval(t, par);
[U, att_d, mem] = controller(t, x, ref, mem, par);
y = [U; att_d; ref.p];
end
