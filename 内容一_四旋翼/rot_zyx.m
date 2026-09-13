function R = rot_zyx(phi, theta, psi)
% ZYX 欧拉角：机体系 -> 惯性系 ENU
cp = cos(phi);   sp = sin(phi);
ct = cos(theta); st = sin(theta);
cs = cos(psi);   ss = sin(psi);
R = [cs*ct, cs*st*sp - ss*cp, cs*st*cp + ss*sp;
     ss*ct, ss*st*sp + cs*cp, ss*st*cp - cs*sp;
     -st,   ct*sp,            ct*cp];
end
