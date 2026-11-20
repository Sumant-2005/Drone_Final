%% EMRAX 228 high-voltage motor and inverter
emrax.Vdc = 680;                    % DC bus voltage (V)
emrax.fsw = 15000;                  % Switching frequency (Hz)
emrax.pole_pairs = 10;
emrax.Rs = 0.0167;                  % Stator resistance (ohm)
emrax.Ld = 177e-6;                  % d-axis inductance (H)
emrax.Lq = 183e-6;                  % q-axis inductance (H)
emrax.flux = 0.0542;                % Magnet flux linkage (Wb)

%% Shaft inertia and limits
emrax.rotor_inertia = 0.0383;       % kg m^2
emrax.propeller_inertia = 0.70;     % kg m^2, assumed
emrax.continuous_phase_arms = 115;  % Phase current (A RMS)
emrax.speed_limit = 5500;           % Mechanical rpm

%% Start from rest before replaying the flight
emrax.ramp_seconds = 6;
emrax.settle_seconds = 1;
