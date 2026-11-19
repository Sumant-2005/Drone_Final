function y = df_measure(state,p)
x = state(:); rotor = x(13:16);
v_forward = cos(x(9))*x(4) + sin(x(9))*x(5);
% Forward speed, height, N, E, attitude in degrees, rotor rpm, aerodynamic load.
y = [v_forward; -x(3); x(1:2); x(7:9)*180/pi; rotor*30/pi; p.kQ*rotor.^2];
end
