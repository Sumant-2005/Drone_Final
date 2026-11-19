function demands = df_velocity(input,p)
% Five command channels and the delayed plant state. NED, body x forward.
u = input(:); x = u(6:end); angle = x(7:9);
c = cos(angle(3)); s = sin(angle(3)); body_to_ne = [c -s;s c];
v_ref = body_to_ne*u(1:2);
accel = p.velocity_gain_per_s*(v_ref-x(4:5)) + p.linear_drag_N_per_m_s/p.mass_kg*v_ref;
accel = accel/max(1,norm(accel)/p.max_accel_m_s2);
acc_body = body_to_ne.'*accel;
angle_ref = [atan2(acc_body(2),p.gravity_m_s2); -atan2(acc_body(1),p.gravity_m_s2)];
climb_ref = u(3) + p.height_gain_per_s*(u(4)+x(3));
up_accel = p.climb_gain_per_s*(climb_ref+x(6));
thrust = (p.mass_kg*(p.gravity_m_s2+up_accel) + p.linear_drag_N_per_m_s*climb_ref) ...
    /max(0.5,cos(angle(1))*cos(angle(2)));
demands = [thrust;angle_ref;u(5)];
end
