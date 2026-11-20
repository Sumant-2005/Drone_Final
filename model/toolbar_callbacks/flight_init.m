run(fullfile(fileparts(get_param('DroneFlight','FileName')),'FlightParameters.m'));
drone.kT = drone.thrust_coefficient*drone.air_density_kg_m3*drone.diameter_m^4/(2*pi)^2;
area = pi*drone.diameter_m^2/4;
% Ideal actuator disk in static hover: P = T^(3/2)/sqrt(2*rho*A), Q=P/omega.
drone.kQ = drone.kT^1.5/sqrt(2*drone.air_density_kg_m3*area);
drone.hover_thrust = drone.mass_kg*drone.gravity_m_s2/drone.rotors;
drone.max_thrust = drone.sizing_weight_factor*drone.hover_thrust;
drone.hover_speed = sqrt(drone.hover_thrust/drone.kT);
a = drone.arm_projection_m;
b = drone.kQ/drone.kT;
% Rows: upward thrust, body roll, pitch, yaw moments. FR, BL, FL, BR.
drone.effectiveness = [1 1 1 1; -a a a -a; a -a a -a; b b -b -b];
drone.allocation = drone.effectiveness\eye(4);
I = drone.inertia_kg_m2(:);
drone.gyro_gain = diag([I(3)-I(2); I(1)-I(3); I(2)-I(1)]);
drone.x0 = [0;0;-drone.initial_height_m;zeros(9,1);drone.hover_speed*ones(4,1)];
profile = drone.mission;
t = (0:drone.step_s:profile(end,1)).';
cmd = interp1(profile(:,1),profile(:,2:5),t,'previous');
height = drone.initial_height_m + [0;cumsum(cmd(1:end-1,3))*drone.step_s];
% Center source lookup intervals around the physical solver ticks.
held = [t(1);t(1:end-1)+diff(t)/2];
assignin('base','drone',drone);
assignin('base','forward_command',timeseries(cmd(:,1),held));
assignin('base','mission_commands',timeseries([cmd(:,2:3),height,cmd(:,4)*pi/180],held));
