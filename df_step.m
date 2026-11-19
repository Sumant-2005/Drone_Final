function next = df_step(input,p)
% One fixed RK4 step with the four speed references held constant.
u = input(:); x = u(1:16); ref = u(17:20); dt = p.step_s;
k1 = rhs(x,ref,p);
k2 = rhs(x+dt*k1/2,ref,p);
k3 = rhs(x+dt*k2/2,ref,p);
k4 = rhs(x+dt*k3,ref,p);
next = x + dt*(k1+2*k2+2*k3+k4)/6;
end

function dx = rhs(x,ref,p)
v=x(4:6); phi=x(7); theta=x(8); psi=x(9); w=x(10:12); rotor=x(13:16);
sf=sin(phi); cf=cos(phi); st=sin(theta); ct=cos(theta); sy=sin(psi); cy=cos(psi);
body_z=[cf*st*cy+sf*sy; cf*st*sy-sf*cy; cf*ct];
loads=p.effectiveness*(p.kT*rotor.^2);
acc=[0;0;p.gravity_m_s2]-(loads(1)*body_z+p.linear_drag_N_per_m_s*v)/p.mass_kg;
I=p.inertia_kg_m2(:);
dw=(loads(2:4)-cross(w,I.*w)-p.angular_drag_Nm_per_rad_s*w)./I;
euler=[w(1)+(sf*w(2)+cf*w(3))*st/ct; cf*w(2)-sf*w(3); (sf*w(2)+cf*w(3))/ct];
dx=[v;acc;euler;dw;(ref-rotor)/p.speed_lag_s];
end
