function [U_act, w] = mixer(U, par)
% "+" 型控制分配：U -> omega^2（限幅）-> 实际 U
% U = [U1; U2; U3; U4]  推力、滚转、俯仰、偏航力矩

b = par.b;                   % 推力系数
d = par.d;                   % 反扭矩系数
l = par.l;                   % 力臂
U1 = U(1); U2 = U(2); U3 = U(3); U4 = U(4);

% 逆分配求四桨转速平方
w2 = zeros(4, 1);
w2(1) = U1/(4*b) - U3/(2*b*l) - U4/(4*d);
w2(2) = U1/(4*b) - U2/(2*b*l) + U4/(4*d);
w2(3) = U1/(4*b) + U3/(2*b*l) - U4/(4*d);
w2(4) = U1/(4*b) + U2/(2*b*l) + U4/(4*d);

% 转速非负且封顶
w2 = sat(w2, 0, par.w_max^2);
w = sqrt(w2);

% 正分配回到实际控制量
U_act = zeros(4, 1);
U_act(1) = b * sum(w2);
U_act(2) = b * l * (w2(4) - w2(2));
U_act(3) = b * l * (w2(3) - w2(1));
U_act(4) = d * (w2(2) + w2(4) - w2(1) - w2(3));
end
