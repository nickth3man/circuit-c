# Findings

### F1 — spokeCount earns distinctness credit for a feature that is never drawn [source]

 car_visual.c:611-658 derives spokeCount from wheelInertiaKgM2. It is hashed into car_visual_bake_key() (:952) and exported as signature
 component CAR_SIG_spoke_level = spokeCount × CAR_SIG_LEVEL_STEP (:1125).

 The rasterizer never draws spokes. Case-insensitive spoke across src/render/ matches only car_visual.{c,h} — zero hits in
 car_visual_raster.c, and CarRasterLabel has CAR_LABEL_RIM/CAR_LABEL_DISC but no spoke label.

 Why this matters: CAR_SIG_LEVEL_STEP is 0.08f, exactly CV_MIN_LINF. The comment at car_visual.c:1064 states the intent — "a change of
 level is by itself always enough to separate two cars." So two corpus cars straddling a spoke threshold clear the L∞ floor on an
 invisible component. The pixel floor (CV_MIN_PIXEL_DIFF 3%) is independent and still binds, so the fleet is not actually degenerate —
 but the signature's stated job ("report WHICH feature makes two cars too similar", car_visual.h:254) returns a wrong answer, and one of
 the three documented floors can be satisfied by nothing.

 Corroborating: wheel.inertia is absent from kVisualDrivers[] (tests/scenarios/appearance_tests.c:60-99) — the team already measured it
 as not wired to pixels. And docs/CAR_VISUAL.md:148 lists Spoke count | wheelInertiaKgM2 | rule under "Drawn feature", which is false.

 Fix: either draw spokes (add CAR_LABEL_SPOKE, an L6 feature) or drop CAR_SIG_spoke_level, spokeCount, its bake-key entry, and the doc
 row. Signature components are append-only, so removal means retiring the slot, not reordering.

### F2 — exhaustTransition is write-only; the documented smoothing does not exist [source]

 car_visual.c:777 computes out->exhaustTransition = smoothstep(0.1f, 0.4f, cyl01), commented (:769, and car_visual.h:201) as "Transition
 band via exhaustTransition so count changes don't snap full-size."

 It is read in exactly one place: key_f32(&h, v->exhaustTransition) at :968. The rasterizer (car_visual_raster.c:603-609) uses raw
 exhaustCount and exhaustBoreM with no transition term. Exhaust count therefore does snap at the 4- and 8-cylinder thresholds (:773-775).

 Either apply it in the raster or delete the field and the two comments.

### F3 — collision_resolve_track is the repo's worst function and has no direct test [graph][source][run]

 ┌────────────────────────────┬──────────────────────────────────────────────┐
 │ Metric                     │ Value                                        │
 ├────────────────────────────┼──────────────────────────────────────────────┤
 │ Cognitive complexity       │ 128 (next worst: 71)                         │
 ├────────────────────────────┼──────────────────────────────────────────────┤
 │ Cyclomatic                 │ 29                                           │
 ├────────────────────────────┼──────────────────────────────────────────────┤
 │ Lines                      │ 315                                          │
 ├────────────────────────────┼──────────────────────────────────────────────┤
 │ Exact-duplicate code lines │ 143 / 234 = 61%                              │
 ├────────────────────────────┼──────────────────────────────────────────────┤
 │ Direct test references     │ 0                                            │
 ├────────────────────────────┼──────────────────────────────────────────────┤
 │ Covering scenario          │ collision-barrier, 7 checks, end-to-end only │
 └────────────────────────────┴──────────────────────────────────────────────┘

 The ~50-line impulse block (penetration push → contact velocity → effective mass → restitution → Coulomb clamp → linear + angular apply
 → lockout) is copy-pasted four times: {front, rear} circle × {left, right} barrier, at :210-278, :281-327, :338-383, :386-430. The four
 copies differ only in pushN, bA/bB, and which circle. Any physics correction must land in all four.

 Three concrete defects visible inside it:

- Contract violation. collision.h:34 promises "Returns the number of contacts resolved (0 if none)." Every success path is a bare return
   1 (:275, 324, 381, 430); the function can only ever return 0 or 1.
- Redundant work in the hot path. Each circle computes point_segment_sq() twice (:210 then :212, and again at :281/283, :338/340,
   :386/388) plus a sqrtf. That is 8 redundant distance evaluations per track segment per substep, and this runs COLLISION_SUBSTEPS = 6 ×
   track->count times inside game_fixed_update.
- Comment overstates the maths. :199 — hw = ni->halfWidthM + (nj->halfWidthM - ni->halfWidthM) * 0.5f is the constant midpoint, not
   "Interpolate half-width between the two nodes" at the contact parameter.

 Extract one resolve_contact(circleWorld, pushN, bA, bB, …) helper; the four call sites collapse to a loop over {circle} × {barrier}.

### F4 — PLAN_TESTING_PROGRESS.md claims shipped work that is not committed [tree]

 The doc's first heading is "Committed (shipped — ready for UCRT64 verification)" and marks A1/A3/A4 plus both vendored libraries as [x].
 Actual state at HEAD 9d252d0:

 ```
    M tests/scenarios/{gameplay,handling,physics}_tests.c
    M tests/support/test_harness.{c,h}
    M tests/test_main.c            (481 insertions, 17 deletions, unstaged)
   ?? tests/support/fff/           (368 KB, untracked)
   ?? tests/support/greatest/      ( 81 KB, untracked)
   ?? BASELINE_SCENARIOS.md, PLAN_TESTING_OVERHAUL.md, PLAN_TESTING_PROGRESS.md
 ```

 Nothing is committed. Mitigating: the modified sources do build and pass (build.bat --tests → clean, full suite green) [run], and
 test_harness.c is correctly wired into TEST_RUNNER_SRCS (Makefile:147).

 Also: neither greatest.h nor fff.h is #included anywhere in tests/ — 449 KB of untracked vendored code with zero adoption. fff/NOTICE.md
 states the intent (mocks for track.h/collision.h); given F3, that is the right target, but today they are dead weight.

### F5 — Four genuinely dead exported functions [graph][source]

 The graph flagged 14 zero-caller non-entry-point functions in src/. Ten are false positives, each confirmed live:

 ┌────────────────────────────────────────────┬─────────────────────────────────────────────────────────────────────────────────────────┐
 │ Candidate                                  │ Actually reached via                                                                    │
 ├────────────────────────────────────────────┼─────────────────────────────────────────────────────────────────────────────────────────┤
 │ Game_{Load,MaybeHotReload,Unload}Module    │ main.c:308/323/334/351/411, hotreload_harness.c (headless variant is a static inline    │
 │                                            │ stub — guarded duplicate)                                                               │
 ├────────────────────────────────────────────┼─────────────────────────────────────────────────────────────────────────────────────────┤
 │ profile_{zone_end,frame_mark,report}       │ macros DRIFTY_ZONE_END/FRAME_MARK/PROFILE_REPORT at game.c:412/497/539,                 │
 │                                            │ render.c:228/231/232                                                                    │
 ├────────────────────────────────────────────┼─────────────────────────────────────────────────────────────────────────────────────────┤
 │ platform_fixed_update                      │ function pointer passed to timestep_advance (main.c:381)                                │
 ├────────────────────────────────────────────┼─────────────────────────────────────────────────────────────────────────────────────────┤
 │ render_draw_gallery,                       │ render.c:148 dispatch on game->dev.galleryPage, then render_vehicle.c:285               │
 │ render_gallery_page_count                  │                                                                                         │
 ├────────────────────────────────────────────┼─────────────────────────────────────────────────────────────────────────────────────────┤
 │ lerpf                                      │ canonical in math_utils.c; graph bound calls to the shadowing static lerpf in audio.c   │
 └────────────────────────────────────────────┴─────────────────────────────────────────────────────────────────────────────────────────┘

 Four survive: definition + header declaration and nothing else, across src tests fuzz tools scripts docs Makefile build.sh README.md:

- dev_params_all — dev_params.c:369 / .h:63
- dev_replay_event_name — dev_replay.c:26 / .h:61
- dev_marker_name — dev_state.c:127 / .h:108
- replay_remaining — replay.c:110 / .h:89

### F6 — Redundant, semantically divergent clampf/lerpf in audio.c [source]

 src/core/math_utils.c owns the canonical clampf/lerpf (24 real callers across drivetrain, physics, tire, timestep, render, scoring,
 input). src/game/audio.c:43,47 defines file-local duplicates — and the audio clampf omits the bound-swap the canonical one performs when
 hi < lo. A latent divergence trap, and it is what made the graph attribute all 24 callers to audio.c. Delete both; include
 core/math_utils.h.

 Same shape, lower risk: maxf duplicated in car_visual.c and car_visual_raster.c.

### F7 — Signature components are collinear around race details [source]

 Five of the 74 signature components encode one degree of freedom (raceDetailWeight):

 ```
   race_detail_weight       = v->raceDetailWeight
   tow_hook_flag            = hasTowHook  ? 0.08 : 0      ⎫ hasTowHook ⇔ diameter > 0
   tow_hook_diameter        = towHookDiameterM            ⎬ all three are 0 iff
   tow_hook_x               = towHookXM                   ⎭ raceDetailWeight ≤ 0.50
   hood_pins_flag           = hasHoodPins ? 0.08 : 0      ⎫ hasHoodPins ⇔ diameter > 0
   hood_pin_diameter        = hoodPinDiameterM            ⎭ threshold 0.42
 ```

 car_visual.h:120 explicitly warns against exactly this — "already counted … again here would double-count it." Two cars differing only
 in raceDetailWeight accumulate up to 5 correlated contributions to L2, inflating the measured distance relative to a pair differing in 5
 independent features. That biases CV_MIN_L2 = 0.25 in favour of race-detail differences.

 Consequently hasTowHook / hasHoodPins are pure redundancy: the rasterizer gates on towHookDiameterM > 0 / hoodPinDiameterM > 0
 (raster.c:646, 649), never the bools.

### F8 — Surface_Get bound check is one-sided [source]

 ```c
   const SurfaceSpec *Surface_Get(SurfaceId id) {
       if (id >= SURFACE_COUNT) id = SURFACE_ASPHALT;   // surface.c:58
       return &kSurfaces[id];
   }
 ```

 The header promises "out-of-range id is clamped" (surface.h:24). Negative id is unclamped. SurfaceId's enumerators are all non-negative
 so GCC/Clang pick unsigned int and this is safe today — but surfaceId is a WheelState field inside Game, and Surface_Get runs 4× per
 physics tick. Make it if (id < 0 || id >= SURFACE_COUNT); it costs nothing.

### F9 — Track.nodes ownership comment is wrong [source][tree]

 track.h:24 — TrackNode *nodes; /* heap-allocated, platform-owned, survives reload */. Both calloc (track.c:80) and free (:106) are in
 src/world/track.c, which is in GAME_SRCS (Makefile:137) — the hot-reloadable module, not PLATFORM_SRCS. It works because MSYS2 UCRT64
 shares one ucrtbase heap between exe and DLL, but the comment states a guarantee the build does not provide, and would silently become
