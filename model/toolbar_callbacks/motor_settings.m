motorFolder = fileparts(get_param('EMRAXDrive','FileName'));
run(fullfile(motorFolder,'MotorParameters.m'));
run(fullfile(motorFolder,'FlightParameters.m'));
emrax.flight_step = drone.step_s;
emrax.J = emrax.rotor_inertia + emrax.propeller_inertia;
emrax.current_limit = sqrt(2)*emrax.continuous_phase_arms; % amplitude-invariant dq, peak A
kt = 1.5*emrax.pole_pairs*emrax.flux;
current_bw = 2*pi*emrax.fsw/10;
inverter_gain = emrax.Vdc/2;
emrax.kp_d = current_bw*emrax.Ld/inverter_gain;
emrax.kp_q = current_bw*emrax.Lq/inverter_gain;
emrax.ki_i = current_bw*emrax.Rs/inverter_gain;
a = tand(65) + 1/cosd(65);
emrax.tau_filter = 1/(current_bw/100*a);
tau_total = emrax.tau_filter + 1/current_bw;
emrax.speed_kp = emrax.J/(kt*a*tau_total);
emrax.speed_ki = emrax.J/(kt*a^3*tau_total^2);
emrax.mission_start = emrax.ramp_seconds + emrax.settle_seconds;

emrax.stop_time = emrax.mission_start + drone.mission(end,1);
assignin('base','emrax',emrax);
