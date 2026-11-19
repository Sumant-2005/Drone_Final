function e = setup_emrax()
% Initialize the separate EMRAX 228 HV drive using the latest mission replay.
root = fileparts(mfilename('fullpath'));
file = fullfile(root,'results','mission_motor1.mat');
assert(isfile(file),'Run run_final from the project folder to generate the motor 1 replay.');
r = load(file);
p = drone_parameters();
assert(isequal(r.motor,1) && strcmp(r.scenario,'mission') && isfield(r,'mass_kg') && ...
    r.mass_kg==p.mass_kg,'Replay belongs to another flight case. Run run_final.');
e = jsondecode(fileread(fullfile(root,'emrax_design.json')));
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
% The mission starts with a movement command. For spin-up use the physical
% hover trim, independently of the first commanded reference sample.
t = r.speed_ref_rpm.Time(:);
rpm = r.speed_ref_rpm.Data(:);
loadNm = r.aero_load_Nm.Data(:);
expected = (0:p.step_s:p.mission(end,1)).';
assert(isequal(size(t),size(expected)) && max(abs(t-expected))<1e-8 && ...
    isequal(t,r.aero_load_Nm.Time(:)) && all(isfinite([rpm;loadNm])) && ...
    all(rpm>=0 & rpm<=e.speed_limit) && all(loadNm>=0),'Invalid EMRAX mission inputs.');
warm = (0:e.flight_step:e.mission_start-e.flight_step).';
fraction = min(warm/e.ramp_seconds,1);
time = [warm; t+e.mission_start];
e.stop_time = time(end);
speed = [p.hover_speed*30/pi*fraction; rpm];
torque = [p.kQ*p.hover_speed^2*fraction.^2; loadNm];
assignin('base','shaft_speed_command',timeseries(speed,time));
assignin('base','shaft_load',timeseries(torque,time));
assignin('base','emrax',e);
end
