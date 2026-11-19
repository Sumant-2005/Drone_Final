function p = FlightParameters()
% 300 kg quad-X. The model calls this file automatically on load and on Run.
p.mass_kg = 300;                       % kg, total aircraft mass
p.gravity_m_s2 = 9.81;
p.air_density_kg_m3 = 1.225;
p.diameter_m = 1.0;                    % m, one propeller
p.thrust_coefficient = 0.1;
p.rotors = 4;
p.sizing_weight_factor = 1.1;
p.arm_projection_m = 1.1;
p.inertia_kg_m2 = [100;100;185];       % kg m^2, assumed roll/pitch/yaw inertias
p.linear_drag_N_per_m_s = 20;
p.angular_drag_Nm_per_rad_s = 50;
p.speed_lag_s = 0.035;
p.step_s = 0.00125;
p.initial_height_m = 5;
p.velocity_gain_per_s = 0.85;
p.max_accel_m_s2 = 1.4;
p.tilt_gain_per_s = 2.4;
p.rate_gain_per_s = [6;6;3];
p.height_gain_per_s = 0.85;
p.climb_gain_per_s = 1.6;

% Time (s), forward (m/s), right (m/s), climb (m/s), yaw rate (deg/s)
p.mission = [
     0  2  0  0   0
     4  0  0  0   0
     7 -2  0  0   0
    11  0  0  0   0
    14  0  2  0   0
    18  0  0  0   0
    21  0 -2  0   0
    25  0  0  0   0
    28  0  0  1   0
    31  0  0  0   0
    34  0  0 -1   0
    37  0  0  0   0
    40  0  0  0  10
    43  0  0  0   0
    46  0  0  0 -10
    49  0  0  0   0
    52  0  0  0   0
];
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
I = p.inertia_kg_m2(:);
p.gyro_gain = diag([I(3)-I(2); I(1)-I(3); I(2)-I(1)]);
p.x0 = [0;0;-p.initial_height_m;zeros(9,1);p.hover_speed*ones(4,1)];

if nargout == 0
    profile = p.mission;
    t = (0:p.step_s:profile(end,1)).';
    cmd = interp1(profile(:,1),profile(:,2:5),t,'previous');
    height = p.initial_height_m + [0;cumsum(cmd(1:end-1,3))*p.step_s];
    % Center source lookup intervals around the physical solver ticks.
    held = [t(1);t(1:end-1)+diff(t)/2];
    assignin('base','drone',p);
    assignin('base','forward_command',timeseries(cmd(:,1),held));
    assignin('base','mission_commands',timeseries([cmd(:,2:3),height,cmd(:,4)*pi/180],held));
end
end
