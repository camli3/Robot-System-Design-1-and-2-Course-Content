function log = slx_pack_log(simout)
% 把 To Workspace 时序整理成与 simulate 相同的 log
x = simout.x_log.Data';
t = simout.x_log.Time';
U = simout.U_log.Data';
ref = simout.ref_log.Data';
att_d = simout.attd_log.Data';
log.t = t;
log.x = x;
log.U = U;
log.ref = ref;
log.att_d = att_d;
log.e = ref - x(1:3, :);
end
