## Specification phase

Phase:

## Change

What behaviour or infrastructure changed?

## Physical invariants affected

- [ ] None
- [ ] Units
- [ ] Coordinate/sign convention
- [ ] Tire forces
- [ ] Drivetrain
- [ ] Load transfer
- [ ] Integration
- [ ] Collision
- [ ] Scoring only

## Verification

- [ ] `mk verify-fast`
- [ ] `mk verify`
- [ ] Relevant scenario tests (`mk scenario NAME=...`)
- [ ] Sanitizers (`mk sanitize`)
- [ ] Visual inspection in the running game

## Telemetry changes

Summarise the meaningful metric changes — 0–100 km/h, peak sideslip, peak yaw rate, steady
radius, recovery time. `mk report NAME=<scenario>` produces them.

## Baselines

- [ ] No baselines changed
- [ ] Baselines changed, and the justification is below

<!-- Re-recording a baseline is never a way to make a failing test green. If the numbers
     moved, say which physical change moved them and why the new values are the correct
     ones. -->

## Hot reload

- [ ] No change to the layout of `Game`
- [ ] `Game` layout changed — the developer must restart `circuit.exe` once
- [ ] `main.c` / `hotreload_windows.c` changed — platform layer is not reloadable

## Evidence

Plots, screenshots, replay files, failure bundles, or profiler captures.
