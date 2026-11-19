function p = df_parameters()
% Report Ch. 3 sizing inputs and separate flight assumptions live in design.json.
p = jsondecode(fileread(fullfile(fileparts(mfilename('fullpath')),'design.json')));
p.kT = p.thrust_coefficient*p.air_density_kg_m3*p.diameter_m^4/(2*pi)^2;
area = pi*p.diameter_m^2/4;
% Ideal actuator disk in static hover: P = T^(3/2)/sqrt(2*rho*A), Q=P/omega.
p.kQ = p.kT^1.5/sqrt(2*p.air_density_kg_m3*area);
p.hover_thrust = p.mass_kg*p.gravity_m_s2/p.rotors;
p.max_thrust = p.sizing_weight_factor*p.hover_thrust;
p.hover_speed = sqrt(p.hover_thrust/p.kT);
a = p.arm_projection_m;
b = p.kQ/p.kT;
% Rows: upward thrust, body roll, pitch, yaw moments. FR, BL, FL, BR.
p.effectiveness = [1 1 1 1; -a a a -a; a -a a -a; b b -b -b];
p.allocation = p.effectiveness\eye(4);
p.x0 = [0;0;-p.initial_height_m;zeros(9,1);p.hover_speed*ones(4,1)];
end
