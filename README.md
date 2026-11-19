# Drone_Final

An R2023a Simulink flight model for a 400 kg quad-X drone. One forward-speed command produces a collective thrust and roll, pitch and yaw moment requests. The mixer allocates these to four propellers. Each thrust becomes a rotor-speed reference. A first-order speed response and a six-degree-of-freedom rigid body close the flight loop.

## Run and view the scopes

Open MATLAB R2023a with Simulink, make the **repository root** the current folder, and run:

```matlab
run_drone
```

The first run builds `generated/DroneFlightBlocks.slx` from sums, gains, products, trigonometric blocks, integrators, limits and routing blocks. The new filename allows the earlier `DroneFlight.slx` to stay open while you try this version. Subsequent runs reuse that file. The model starts in level hover at 5 m, waits one second, commands 2 m/s forward for four seconds, then brakes and holds for four seconds. The auxiliary commands remain at zero except the constant 5 m height target. The command is a velocity setpoint, so braking ends in a hover at the new position.

The routine runs the simulation, checks its results against the Python flight reference, writes the replay inputs, and opens **Flight scopes**. Inside **Flight scopes**, open **Flight response**, **Mixer inputs** or **Motor outputs**, then double-click a Scope. For the demonstration, start with **Motor outputs → Four thrust references - N** and **Flight response → Forward speed - m per s**. The numerical checker reads the signals directly from existing block ports through Simulink signal logging. Scope units and signal order are:

| Scope | Traces |
|---|---|
| Forward speed / Height | Command and actual, in m/s or m |
| Collective thrust / Roll pitch yaw moments | Total requested upward force in N and the three requested body moments in N·m |
| Four thrust references / Four speed references | M1 front right, M2 back left, M3 front left, M4 back right, in N or rpm |
| Actual rotor speeds / Propeller loads | The same four motors, in rpm or resisting N·m |
| North and east / Roll pitch yaw | Position in m and attitude in degrees |

At the start of the forward pulse the rear pair increases thrust. At release the front pair increases thrust to brake. Compare the reference and actual speed traces to see the effect of the flight actuator lag. The command and scopes use the same closed-loop model for the full movement sequence:

```matlab
run_drone('mission')
```

The 52 s mission commands forward, back, right, left, climb, descent and paired yaw turns. The navigation frame is north/east/down. Height and climb are positive upward. The rigid-body model is intended for airborne manoeuvres with modest tilt; contact, wind and an electrical drive are outside this flight experiment.

## Motor inputs for later replay

`run_drone('demonstration',3)` selects M3 for export. The default is M1. The CSV in `results/` includes time, speed reference (rad/s and rpm), thrust reference (N), actual flight-model speed (rpm), and positive resisting aerodynamic torque (N·m). The matching MAT file holds named `timeseries` signals `speed_ref_rad_s`, `speed_ref_rpm`, and `aero_load_Nm`. The trace begins at hover speed. The aerodynamic load excludes rotor acceleration torque. The detailed EMRAX speed/current simulation and its initialization are deferred.

## Report inputs and units

The sizing inputs come from Chapter 3, pp. 15–16 of the supplied *Design of zero emission long endurance drones*. They are 400 kg, four 1 m propellers, air density 1.225 kg/m³, thrust coefficient 0.1, gravity 9.81 m/s², and a sizing thrust of 1.1 times weight. In the propeller law, n is **revolutions per second**. RPM is 60n. The model uses:

```text
T = CT × rho × D^4 × n² = kT × omega²
kT = CT × rho × D^4 / (2π)²
hover thrust per rotor = mg / 4
sizing thrust per rotor = 1.1mg / 4
```

| Per rotor | Level hover | Report sizing thrust |
|---|---:|---:|
| Thrust | 981.00 N | 1,079.10 N |
| Speed from the thrust law | 5,369.30 rpm | 5,631.37 rpm |
| Ideal static induced power | 22.15 kW | 25.55 kW |

The report rounds the sizing speed to about 5,600 rpm and uses 600 rad/s and 20 kW to estimate 33.3 N·m, with a 50 N·m peak estimate. The model uses the exact thrust law for rpm. For static hover, ideal actuator-disk theory gives P = T^(3/2)/sqrt(2 rho A), with A = πD²/4 per rotor. Even this lower bound exceeds the report's 20 kW estimate. The flight model therefore uses Q = kQ × omega² with kQ derived from the ideal static relation. It applies this quasi-steady approximation throughout the short manoeuvres. The replay is an illustrative shaft-load trace. Quantitative drive sizing requires a measured propeller torque/power curve and applicable EMRAX limits.

The 10% sizing margin serves as the model's per-rotor thrust ceiling. The projected motor arms (1.1 m), principal inertias (100, 100, 185 kg·m²) and drag terms are working assumptions carried over from the earlier flight model. The 35 ms actuator lag and flight gains are simulation design choices. These inputs and mission settings are in `design.json`.

## Flight implementation

- **Movement command** supplies the forward reference plus optional right, climb, height and yaw-rate channels. The height target is the integral of the climb command plus the initial 5 m.
- **Flight controller** rotates the horizontal command using current heading. Velocity and height errors produce tilt and collective demands. The inner loops turn attitude and rate errors into body moments.
- **Quad X mixer** applies the inverse force/moment effectiveness matrix. Each motor contributes one unit of upward thrust, a signed arm moment in roll and pitch, and a signed Q/T reaction moment in yaw per newton of thrust. When a request exceeds actuator limits, a common scale reduces the moment-producing differences while preserving feasible collective thrust.
- **Thrust to speed** takes the square root of each thrust divided by kT. The resulting four references are in rad/s.
- **Rotors and body** advances 12 NED/body states and four rotor speeds with RK4 at 1.25 ms. A Zero-Order Hold samples the state for the flight controller. The five continuous Integrator blocks advance position, velocity, Euler angles, body rates and the four rotor speeds using the fixed-step ode4 solver. Rotor yaw reaction uses quasi-steady propeller torque.

The portable text model is `model/DroneFlight.mdl`. To inspect it directly from the repository root, close any other model named DroneFlight, run `df_init`, then `open_system('model/DroneFlight.mdl')`. The recommended `run_drone` entry uses the native builder and its own SLX. Both model descriptions come from `model/generate.py`.

## Verification

From the repository root in a Python environment with NumPy and pytest:

```sh
python -m pip install -r requirements.txt
python -m pytest -q -s
```

The tests cover report-unit conversions, ideal-power consistency, all three mixer signs, saturation, command and load traces, braking, mission return and model wiring. `reference/expected.json` records the Python response metrics used by the MATLAB runner. The Python tests also evaluate the connections and arithmetic of the generated Simulink diagrams on off-axis states and integrate the wired forward demonstration from start to finish. These tests run in Python.

After a local simulation, `run_drone` checks the scope unit conversions and scenario acceptance criteria. It compares position, velocity, attitude, motor speed and thrust against `reference/checkpoints.json` at command changes and during each manoeuvre. The demonstration checks rear-pair thrust at acceleration, front-pair thrust at braking and final speed below 0.1 m/s. The mission checks that final displacement is within 0.5 m. The MATLAB scripts and models are written, not run in R2023a. Local simulation must confirm compilation, scope rendering and the numerical comparisons.
