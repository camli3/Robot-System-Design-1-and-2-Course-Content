function ref = traj_eval(t, par)
% 五次多项式平滑插值：位置、速度、加速度、偏航

pts = par.traj.pts;
tseg = par.traj.tseg;
p_from = par.traj.p0(:);
t0 = 0;
idx = 1;
for i = 1:numel(tseg)
    t1 = t0 + tseg(i);
    if t <= t1 || i == numel(tseg)
        idx = i;
        break;
    end
    p_from = pts(i, :)';
    t0 = t1;
end

p_to = pts(idx, :)';
T = max(tseg(idx), 1e-3);
tau = sat((t - t0) / T, 0, 1);

% s(0)=0,s(1)=1, sdot=sddot=0 at ends
s = 10*tau^3 - 15*tau^4 + 6*tau^5;
sd = (30*tau^2 - 60*tau^3 + 30*tau^4) / T;
sdd = (60*tau - 180*tau^2 + 120*tau^3) / T^2;

dp = p_to - p_from;
pv = p_from + s * dp;                    % [x y z yaw]
ref.p = pv(1:3);
ref.v = sd * dp(1:3);
ref.a = sdd * dp(1:3);
ref.yaw = wrap_angle(pv(4));
ref.yaw_rate = sd * dp(4);

if t > sum(tseg)                         % 全部航点后保持最后一点
    ref.p = pts(end, 1:3)';
    ref.v = zeros(3, 1);
    ref.a = zeros(3, 1);
    ref.yaw = pts(end, 4);
    ref.yaw_rate = 0;
end
end
