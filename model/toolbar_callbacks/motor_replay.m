replayFile = fullfile(motorFolder,'Motor1Replay.mat');
assert(isfile(replayFile),'Open DroneFlight, press Run and wait for it to finish. Then press Run in EMRAXDrive.');
record = load(replayFile,'frontRightMotor');
assert(isfield(record,'frontRightMotor'),'Motor1Replay.mat has no motor signal. Run DroneFlight again.');
samples = record.frontRightMotor;
expected = (0:drone.step_s:drone.mission(end,1)).';
assert(isnumeric(samples) && isequal(size(samples),[3 numel(expected)]) && ...
    all(isfinite(samples),'all') && max(abs(samples(1,:).'-expected))<1e-8, ...
    'Motor1Replay.mat is incomplete or uses a different mission duration. Run DroneFlight to completion.');
t = samples(1,:).';
rpm = samples(2,:).';
loadNm = samples(3,:).';
assert(all(rpm>=0 & rpm<=emrax.speed_limit) && all(loadNm>=0),'Flight inputs exceed the selected drive limits.');
kT = drone.thrust_coefficient*drone.air_density_kg_m3*drone.diameter_m^4/(2*pi)^2;
hoverRPM = sqrt(drone.mass_kg*drone.gravity_m_s2/(drone.rotors*kT))*30/pi;
warm = (0:emrax.flight_step:emrax.mission_start-emrax.flight_step).';
fraction = min(warm/emrax.ramp_seconds,1);
time = [warm; t+emrax.mission_start];
emrax.stop_time = time(end);
speed = [hoverRPM*fraction; rpm];
torque = [loadNm(1)*fraction.^2; loadNm];
sourceTime = [time(1);time(1:end-1)+diff(time)/2];
assignin('base','shaft_speed_command',timeseries(speed,sourceTime));
assignin('base','shaft_load',timeseries(torque,sourceTime));
assignin('base','emrax',emrax);
