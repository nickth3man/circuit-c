# Circuit identity and migration map

Status: accepted for issues [#43](https://github.com/nickth3man/drift-c/issues/43),
[#44](https://github.com/nickth3man/drift-c/issues/44), and
[#45](https://github.com/nickth3man/drift-c/issues/45).

## Naming map

| Surface | Previous | Circuit identity |
|---|---|---|
| Player-facing product | `Drifty` | `Circuit` |
| Repository | `nickth3man/drift-c` | `nickth3man/circuit-c` |
| Development executable | `drifty[.exe]` | `circuit[.exe]` |
| Release executable | `drifty_release[.exe]` | `circuit_release[.exe]` |
| Test runner | `drifty_tests[.exe]` | `circuit_tests[.exe]` |
| Hot-reload harness | `drifty_hotreload_harness[.exe]` | `circuit_hotreload_harness[.exe]` |
| C/build macro prefix | `DRIFTY_` | `CIRCUIT_` |
| Private C identifier prefix | `drifty_` | `circuit_` |
| Include-guard prefix | `DRIFTY_` | `CIRCUIT_` |
| Tuning-profile magic | `# drifty tuning profile v1` | `# circuit tuning profile v1` |
| Window/report title | `Drifty` | `Circuit` |
| Temporary/tool paths | `drifty-*` | `circuit-*` |
| Future config/save directory | `Drifty` | `Circuit` |
| Release/archive stem | `drifty*` | `circuit*` |

The source tree currently has no persistent player save or configuration directory. `Circuit`
is reserved as the directory name when persistence is introduced, so this migration needs no
filesystem move. The project is pre-release; old macros, executable names, and tuning-profile
magic are not compatibility contracts, so no aliases are retained.

## Neutral handling names

The identity migration keeps the same vehicle behavior and validation coverage while naming
scenarios for what they measure:

| Previous name | Replacement |
|---|---|
| `catchable-drift` | `sideslip-recovery` |
| `constant-radius-drift` | `constant-radius-limit-equilibrium` |
| `drift-recovery-envelope` | `sideslip-yaw-recovery-envelope` |
| `figure-eight-drift-transition` | `figure-eight-limit-transition` |
| `drift_hud` capture | `limit_handling_hud` |
| `Drift Car` corpus archetype | `High-Angle RWD` |

## Terminology inventory and allowlist

Identity and obsolete gameplay terms are not allowed in current product copy, runtime strings,
build definitions, identifiers, generated profile headers, or artifact names. Historical links
to the original repository may remain when they identify an issue or pull request that exists
only there.

The following uses of *drift* remain valid and must be reviewed in context rather than removed
mechanically:

- vehicle-dynamics terms and cited paper titles: drift equilibrium, controlled drifting,
  power oversteer, sustained sideslip, opposite lock, transition, and recovery;
- non-vehicle meanings: numerical drift, centroid/pivot drift, configuration drift, and prose
  such as “drift apart”;
- historical migration facts, including the removal of drift scoring and links to work in the
  original `drift-c` repository.

Names such as `sideslip`, `yaw rate`, `oversteer`, `power-oversteer`, `tire saturation`, and
`limit-handling` are preferred for current tests, metrics, UI, and documentation.

## Repository and external-link migration

The new repository was created separately, so existing issue and pull-request URLs in
`nickth3man/drift-c` do not automatically move. Documentation keeps those URLs only as
historical references. New development, badges, clones, and release links target
`nickth3man/circuit-c`.

Existing clones can move their `origin` without recloning:

```sh
git remote set-url origin https://github.com/nickth3man/circuit-c.git
git remote -v
```

This source change does not rename or delete the original GitHub repository.
