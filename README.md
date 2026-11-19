# Drone_Final

An R2023a Simulink flight model for a 400 kg quad-X drone. One forward-speed command produces a collective thrust and roll, pitch and yaw moment requests. The mixer allocates these to four propellers. Each thrust becomes a rotor-speed reference. A first-order speed response and a six-degree-of-freedom rigid body close the flight loop.

## Run and view the scopes

Open MATLAB R2023a with Simulink, make the **repository root** the current folder, and run:

```matlab
run_drone
```

The first run builds `generated/QuadXFlight.slx` from sums, gains, products, trigonometric blocks, integrators, limits and routing blocks. It has its own filename so earlier saved models can stay open. Subsequent runs reuse that file. The model starts in level hover at 5 m, waits one second, commands 2 m/s forward for four seconds, then brakes and holds for four seconds. The auxiliary commands remain at zero except the constant 5 m height target. The command is a velocity setpoint, so braking ends in a hover at the new position.

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

## Where the SLX gets its parameters

- **`design.json`** – edit the vehicle data, controller gains, solver interval and mission settings here. Python and MATLAB read the same inputs.
- **`drone_parameters.m`** – the MATLAB parameter file. It reads those inputs and calculates the propeller constants, hover speed, thrust limit, mixing matrix and initial state. Use `drone_parameters()` at the MATLAB prompt to inspect the values.
- **`setup_flight.m`** – the model initialization callback. It places the parameter structure `drone` and the two command time series in the workspace, so each block can use names such as `drone.mass_kg`.

The earlier `df_` prefix stood for the Drone_Final project. The current diagrams use descriptive names. Two compatibility functions with the old names support earlier saved block models.

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

- **Flight command** supplies the forward reference plus optional right, climb, height and yaw-rate channels. The height target is the integral of the climb command plus the initial 5 m.
- **Flight controller** rotates the horizontal command using current heading. Velocity and height errors produce tilt and collective demands. The inner loops turn attitude and rate errors into body moments.
- **Motor mixer** applies the inverse force/moment effectiveness matrix. Each motor contributes one unit of upward thrust, a signed arm moment in roll and pitch, and a signed Q/T reaction moment in yaw per newton of thrust. When a request exceeds actuator limits, a common scale reduces the moment-producing differences while preserving feasible collective thrust.
- **Thrust to speed** takes the square root of each thrust divided by kT. The resulting four references are in rad/s.
- **Drone dynamics** advances 12 NED/body states and four rotor speeds with RK4 at 1.25 ms. A Zero-Order Hold samples the state for the flight controller. The five continuous Integrator blocks advance position, velocity, Euler angles, body rates and the four rotor speeds using the fixed-step ode4 solver. Rotor yaw reaction uses quasi-steady propeller torque.

The portable text model is `model/DroneFlight.mdl`. To inspect it directly from the repository root, close any other model named DroneFlight, run `setup_flight`, then `open_system('model/DroneFlight.mdl')`. The recommended `run_drone` entry uses the native builder and its own SLX. Both model descriptions come from `model/generate.py`.

## Verification

From the repository root in a Python environment with NumPy and pytest:

```sh
python -m pip install -r requirements.txt
python -m pytest -q -s
```

The tests cover report-unit conversions, ideal-power consistency, all three mixer signs, saturation, command and load traces, braking, mission return and model wiring. `reference/expected.json` records the Python response metrics used by the MATLAB runner. The Python tests also evaluate the connections and arithmetic of the generated Simulink diagrams on off-axis states and integrate the wired forward demonstration and the full 52 s mission from start to finish. These tests run in Python.

In the default Python demonstration, the 2 m/s forward pulse produces a peak of 1.9978 m/s, 7.9154 m of forward travel and a final height of 5.0004 m. The complete mission ends 0.0416 m from its start. A second numerical check keeps the controller sample time fixed and uses two midpoint steps per interval for the plant. Across 49 mission checkpoints it agrees with RK4 within 0.000001 m in position and 0.002 rpm in rotor speed.

A small final position error can coexist with a transient tracking error. The mission's first 10°/s yaw demand lasts 3 s. The heading is about 18.3° at release and 26.3° after the hover gap. The vehicle begins this turn within 0.02° of north. In this model a yaw-rate command provides no absolute heading target. The independent comparisons establish numerical consistency for the shared assumptions. Physical load accuracy still depends on propeller data.

After a local simulation, `run_drone` checks the scope unit conversions and scenario acceptance criteria. It compares position, velocity, attitude, motor speed and thrust against `reference/checkpoints.json` at command changes and during each manoeuvre. The demonstration checks rear-pair thrust at acceleration, front-pair thrust at braking and final speed below 0.1 m/s. The mission checks that final displacement is within 0.5 m. The MATLAB scripts and models are written, not run in R2023a. Local simulation must confirm compilation, scope rendering and the numerical comparisons.

## Scope figures for the report

The ten flight, mixer and motor scopes each have a demonstration and a mission result. `report_figures/Flight_scope_reference.pdf` pairs the two scenarios on each page, and `report_figures/Flight_scope_reference_PNGs.zip` contains the 20 individual Python reference plots.

For figures from your MATLAB R2023a run, use `export_scope_results` from the repository root. It runs both validated scenarios and writes 300 dpi PNGs to `results/scope_figures/demonstration/` and `results/scope_figures/mission/`. On the MATLAB desktop it also attempts a native capture of each Scope. Files beginning with `window_` are captured Scope views; the other PNGs plot the logged Simulink signals at the solver sample times. If a Scope window cannot be accessed as a graphics figure, the exporter reports it and keeps the trace figure. Use `export_scope_results(false)` for just the report plots.

For the separate EMRAX drive test, `results/demonstration_motor1.mat` provides the front-right motor's mechanical speed reference (rpm or rad/s) and positive resisting aerodynamic torque (N·m). Feed the speed to the existing *outer speed reference* and the torque to the mechanical *load torque* input using the sign expected by your drive. Start the drive at the first hover speed with the corresponding load/current equilibrium, or allow it to settle there before replay. A drive with rotor/propeller inertia already included uses the aerodynamic load directly. The flight rotor rpm is a first-order actuator prediction. The measured EMRAX rotor speed, current and electromagnetic torque must come from the separate drive simulation.
