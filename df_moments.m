function loads = df_moments(input,p)
% Input is collective, roll/pitch demand, yaw rate demand, then the state.
u=input(:);
x=u(5:end);
rates=x(10:12);
rate_ref=[p.tilt_gain_per_s*(u(2:3)-x(7:8));u(4)];
moments=p.inertia_kg_m2(:).*p.rate_gain_per_s(:).*(rate_ref-rates) ...
    + p.angular_drag_Nm_per_rad_s*rates;
loads=[u(1);moments];
end
