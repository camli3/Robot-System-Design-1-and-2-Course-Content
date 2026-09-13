function [U, att_d, mem] = controller(t, x, ref, mem, par)
% 算法切换后经 Mixer 限幅
if par.algo == 1
    [U, att_d, mem] = pid_ctrl(t, x, ref, mem, par);
else
    [U, att_d, mem] = backstepping_ctrl(t, x, ref, mem, par);
end
U(2:4) = sat(U(2:4), -par.tau_max, par.tau_max);
[U, mem.w] = mixer(U, par);
end
