# Phase 3 validation

Phase 3 closes Drifty's mandatory physics foundation. These results were recorded on Windows
with MSYS2 UCRT64, GCC 16.1.0, at 120 fixed updates per second. The headless suite opens no
window. Linux remains a headless CI-support path and is not an authoritative gameplay target.

## Implemented model

The load-transfer filter consumes the previous fixed step's solved body acceleration:

```text
alpha = 1 - exp(-loadFilterRateHz * dt)
filtered_ax += (previous_ax - filtered_ax) * alpha
solved_ax = totalBodyForceX / mass
```

Static and dynamic axle loads are:

```text
L = l_f + l_r
Fz_front_static = m * g * l_r / L
Fz_rear_static  = m * g * l_f / L
transfer        = m * filtered_ax * h / L
Fz_front        = max(Fz_front_static - transfer, MIN_NORMAL_LOAD_N)
Fz_rear         = max(Fz_rear_static  + transfer, MIN_NORMAL_LOAD_N)
```

Each axle load is divided equally between its left and right diagnostic wheels. The current
dynamic wheel load is used by the pure tire curves and the combined-friction limit. The
unclamped axle pair is retained for diagnostics and always sums to `m*g`; clamping is not
silently renormalized.

Aerodynamic drag and rolling resistance are separate:

```text
speed = sqrt(vx*vx + vy*vy)
drag_magnitude = 0.5 * AIR_DENSITY_KGM3 * Cd * area * speed^2
drag = -drag_magnitude * velocity / speed

rolling_wheel_magnitude = rollingResistanceCoefficient * wheelNormalLoad
rolling_wheel = -rolling_wheel_magnitude * contactVelocity / contactSpeed
```

Both forces are zero when their direction is undefined at rest. In the fixed update, each
body-axis resistance component is limited to the impulse required to reach zero velocity in
one tick, so resistance cannot reverse the car.

## Objective maneuver results

| Maneuver | Accepted metrics |
|---|---|
| Acceleration load transfer | peak solved `5.231 m/s²`; peak filtered `5.203 m/s²`; minimum front `5236.7 N`; maximum rear `6531.3 N`; peak rearward transfer `1224.1 N` at `3.80 s`; at 5 s `43.10 m`, `15.568 m/s` |
| Braking load transfer | entry `14.979 m/s`; peak deceleration `10.016 m/s²`; maximum front `8292.4 N`; minimum rear `3475.6 N`; peak forward transfer `1831.5 N`; stop `14.99 m` in `2.042 s` |
| Coast-down | `15.571 → 1.471 m/s`; drag `90.3 → 0.8 N`; rolling resistance `176.5 → 176.5 N`; no reversal or force spike |
| Skidpad | steady speed `11.549 m/s`; yaw `0.7967 rad/s`; lateral acceleration `9.056 m/s²`; sideslip `0.0285 rad`; radius `14.50 m`; front/rear slip `-0.0675/-0.0678 rad`; usage `0.744/0.832` |
| Step steer | 10–90% rise `1.6667 s`; peak/steady yaw `0.7946/0.7834 rad/s`; overshoot `1.4%`; settling `2.4250 s`; peak lateral acceleration `10.739 m/s²`; peak sideslip `0.0395 rad` |
| Lift-off | solved acceleration `1.843 → -2.473 m/s²`; filtered `1.877 → -2.251 m/s²`; front load `6019.2 → 6990.6 N`; rear `5748.8 → 4777.4 N`; yaw `0.7365 → 1.3360 rad/s` |
| Drift transition | 8 steering reversals; 8 sideslip zero crossings; 8 yaw sign changes; peak yaw torque `17548.4 N·m`; worst tick jump `6226.1 N·m`; finite throughout |
| Catchable drift | peak sideslip `0.8203 rad` at `6.59 s`; peak yaw `2.0372 rad/s`; rear usage `1.000`; recovery sideslip `0.0000 rad`; recovery speed `3.269 m/s`; rear usage `0.1343` |

The skidpad sweep covers four commanded speeds. Lateral acceleration and usage rise before
saturation; the deterministic speed controller changes only throttle and brake input and
injects no lateral force or yaw torque.

## Tuning decision

Structural equations, signs, filter order, load propagation, resistance, and determinism
were validated before handling. The documented default `B`, `C`, `mu`, engine, brake-bias,
handbrake, and steering values passed every objective maneuver, including transition and
catch recovery. No tunable default was changed merely to make Phase 3 pass.

`data/vehicles/default.txt` and `data/vehicles/phase3-candidate.txt` are therefore intentionally identical.
The latter is a named review checkpoint, not a claim that two different parameter sets were
tested. There are no accepted tunable changes to list (`old == new` for all 46 parameters).

## Handling reference matrix

Art of Rally and Absolute Drift are behavioral references only; their internal equations are
not public and are not asserted here.

| Behavior | Current observation / metric | Discrepancy | Likely layer |
|---|---|---|---|
| Low-speed steering | stable through the 1.5–3.0 m/s blend; stationary steering produces zero motion | keyboard turn-in feel needs human playtesting | input shaping |
| High-speed steering | step response peaks at `0.7946 rad/s`, `1.4%` overshoot | no external calibrated target | simulation / future observation |
| Drift throttle sensitivity | power-oversteer reaches rear saturation; lift restores lateral authority | sensitivity still needs controller playtesting | input shaping |
| Handbrake entry | rear brake torque locks/loads the rear friction budget; no grip multiplier | no discrepancy in invariants | simulation |
| Braking entry | braking raises front load and consumes the combined-friction budget | trail-brake feel needs human playtesting | input shaping |
| Countersteer speed | catch sequence reduces `0.8203 rad` peak sideslip to zero | keyboard steering may feel abrupt | input shaping |
| Drift stability | scripted slide is held before countersteer without a state-machine force | no real-world calibration claimed | simulation |
| Transition sharpness | 8 clean sign changes; bounded torque continuity | presentation may make transitions look sharper or softer | camera/presentation |
| Spin recovery | returns to `3.269 m/s` forward travel at `0.1343` rear usage | recovery envelope beyond this fixture is unmeasured | future fidelity |

## Baseline policy

- `phase2_*.csv`: intentionally replaced. Phase 3 changes normal loads and resistance on
  every tick, so keeping Phase 2 force histories would be artificial.
- `phase1_launch_stop.csv`: obsolete and removed when the Phase 2 canonical baseline replaced
  the Phase 1 tire model.
- Eight `scenario_*.csv` files listed in `docs/CI.md`: reviewed Phase 3 local baselines.
- CI physics regression remains head-versus-merge-base on the same runner. It does not use
  committed Windows floats as a cross-compiler equality gate.

## Acceptance

All Phase 1–3 acceptance items pass: SI units, render-scale independence, documented signs,
rigid-body contact velocities, distinct axle slips, CG-based static distribution, correct
acceleration/braking transfer, dynamic tire capacity, steered front-force rotation, physical
drivetrain and wheel slip, combined grip, rear brake-torque handbrake, stable low-speed
launch/stop/reverse, bounded fixed stepping, exact-once inputs, windowless tests,
deterministic replay, complete force/load/slip/usage telemetry, and drift initiation, hold,
transition, catch, and recovery without gameplay force input.

The accepted suite result is 36 scenarios, 715 checks, zero failures, with deterministic
replay checksum `f0b4580e`.

The 240,000-tick timer benchmark measured 571,429 ticks/s before the final force-law
correction and 591,133 ticks/s in the final acceptance run (`+3.45%`, 4,926× real time;
checksum `13b07c79`). An earlier post-change run measured 750,000 ticks/s. The built-in
Windows timer is coarse, so this establishes that Phase 3 introduced no material regression;
it is not a microbenchmark claim.
