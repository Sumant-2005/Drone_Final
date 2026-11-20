%% Aircraft and propellers
drone.mass_kg = 300;
drone.initial_height_m = 5;
drone.inertia_kg_m2 = [100; 100; 185];  % Roll, pitch, yaw (assumed)
drone.arm_projection_m = 1.1;
drone.rotors = 4;
drone.diameter_m = 1.0;                 % Propeller diameter
drone.thrust_coefficient = 0.1;
drone.sizing_weight_factor = 1.1;       % Maximum thrust / hover thrust

%% Environment and flight response
drone.gravity_m_s2 = 9.81;
drone.air_density_kg_m3 = 1.225;
drone.linear_drag_N_per_m_s = 20;
drone.angular_drag_Nm_per_rad_s = 50;
drone.speed_lag_s = 0.035;              % Flight-model motor response
drone.step_s = 0.00125;

%% Flight controller
drone.velocity_gain_per_s = 0.85;
drone.max_accel_m_s2 = 1.4;
drone.tilt_gain_per_s = 2.4;
drone.rate_gain_per_s = [6; 6; 3];
drone.height_gain_per_s = 0.85;
drone.climb_gain_per_s = 1.6;

%% Mission commands in body axes; zero commands mean hover
% Time (s)   Forward (m/s)   Right (m/s)   Climb (m/s)   Yaw rate (deg/s)
drone.mission = [
     0   2   0   0    0
     4   0   0   0    0
     7  -2   0   0    0
    11   0   0   0    0
    14   0   2   0    0
    18   0   0   0    0
    21   0  -2   0    0
    25   0   0   0    0
    28   0   0   1    0
    31   0   0   0    0
    34   0   0  -1    0
    37   0   0   0    0
    40   0   0   0   10
    43   0   0   0    0
    46   0   0   0  -10
    49   0   0   0    0
    52   0   0   0    0
];
