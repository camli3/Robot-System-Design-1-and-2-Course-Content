function xdot = slx_plant(u)
% Simulink：u = [x(12); U(4)] -> xdot(12)
x = u(1:12);
U = u(13:16);
par = slx_config();
xdot = plant_rhs(0, x, U, par);
end
