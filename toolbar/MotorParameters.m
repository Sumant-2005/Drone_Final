function MotorParameters(useFlight)
% EMRAX 228 HV. On model load, initialize mask and solver parameters.
% On Run, also read the completed flight from this MATLAB session.
if nargin == 0, useFlight = true; end
p = FlightParameters();
e.Vdc = 680;                         % V
e.fsw = 15000;                       % Hz, PWM
e.Rs = 0.0167;                       % ohm
e.Ld = 177e-6;                       % H
e.Lq = 183e-6;                       % H
e.flux = 0.0542;                     % Wb
e.pole_pairs = 10;
e.rotor_inertia = 0.0383;            % kg m^2
e.propeller_inertia = 0.70;          % kg m^2, assumed
e.continuous_phase_arms = 115;
e.speed_limit = 5500;                % mechanical rpm
e.ramp_seconds = 6;
e.settle_seconds = 1;
e.flight_step = p.step_s;
e.J = e.rotor_inertia + e.propeller_inertia;
e.current_limit = sqrt(2)*e.continuous_phase_arms; % amplitude-invariant dq, peak A
kt = 1.5*e.pole_pairs*e.flux;
current_bw = 2*pi*e.fsw/10;
inverter_gain = e.Vdc/2;
e.kp_d = current_bw*e.Ld/inverter_gain;
e.kp_q = current_bw*e.Lq/inverter_gain;
e.ki_i = current_bw*e.Rs/inverter_gain;
a = tand(65) + 1/cosd(65);
e.tau_filter = 1/(current_bw/100*a);
tau_total = e.tau_filter + 1/current_bw;
e.speed_kp = e.J/(kt*a*tau_total);
e.speed_ki = e.J/(kt*a^3*tau_total^2);
e.mission_start = e.ramp_seconds + e.settle_seconds;

e.stop_time = e.mission_start + p.mission(end,1);
assignin('base','emrax',e);
if ~useFlight, return; end
assert(evalin('base','exist(''motor1_flight_replay'',''var'')') == 1, ...
    'First open DroneFlight and press Run. Wait for the flight to finish, then press Run here.');
r = evalin('base','motor1_flight_replay');
assert(isstruct(r) && isfield(r,'time') && isfield(r,'signals') && numel(r.signals)==1, ...
    'Unexpected flight output. Run DroneFlight again in this MATLAB session.');
t = r.time(:);
v = r.signals.values;
expected = (0:p.step_s:p.mission(end,1)).';
assert(isequal(size(t),size(expected)) && max(abs(t-expected))<1e-8 && ...
    isequal(size(v),[numel(t),2]) && all(isfinite(v),'all'), ...
    'A complete flight replay is required. Let DroneFlight run to the end before starting EMRAXDrive.');
rpm = v(:,1);
loadNm = v(:,2);
assert(all(rpm>=0 & rpm<=e.speed_limit) && all(loadNm>=0),'Flight replay exceeds the drive input limits.');
warm = (0:e.flight_step:e.mission_start-e.flight_step).';
fraction = min(warm/e.ramp_seconds,1);
time = [warm; t+e.mission_start];
e.stop_time = time(end);
speed = [p.hover_speed*30/pi*fraction; rpm];
torque = [p.kQ*p.hover_speed^2*fraction.^2; loadNm];
sourceTime = [time(1);time(1:end-1)+diff(time)/2];
assignin('base','shaft_speed_command',timeseries(speed,sourceTime));
assignin('base','shaft_load',timeseries(torque,sourceTime));
assignin('base','emrax',e);
end
