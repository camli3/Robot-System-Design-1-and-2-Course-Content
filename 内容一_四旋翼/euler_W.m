function W = euler_W(phi, theta)
% 体轴角速度 omega -> 欧拉角速率：eta_dot = W * omega
W = [1, sin(phi)*tan(theta),  cos(phi)*tan(theta);
     0, cos(phi),            -sin(phi);
     0, sin(phi)/cos(theta),  cos(phi)/cos(theta)];
end
