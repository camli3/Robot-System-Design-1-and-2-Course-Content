function y = sat(x, lo, hi)
% 饱和限幅，支持标量或向量
y = min(max(x, lo), hi);
end
