The game has a strong technical foundation, but it is not yet the complete player-facing racing game described in the roadmap.

A simple way to put it is:

> Much of the machinery under the hood works, but many dashboard controls, menus, race-weekend features, and final presentation screens either do not exist or are not connected to that machinery.

This explanation reflects the codebase at commit `5ca62d76f631b8a1ab625ff55943df0a023182eb`, which is the version I audited.

## Overall state

| Area | Level | What that means for a player |
|---|---:|---|
| Driving physics | Strong | Cars can be driven and react to tires, suspension, drivetrain, surfaces, damage, and weather-related grip. |
| Tracks and car data | Strong | Multiple cars and tracks load from external data files and are validated. |
| Basic AI driving | Moderate to strong | Most AI cars can follow a track and race, but two shipped cars currently fail the standard four-lap run. |
| Multi-car racing rules | Moderate | Grid placement, timing, positions, contacts, some penalties, and race completion exist underneath. |
| Race setup menus | Weak | A player cannot configure the full race from the normal menu. |
| Race HUD and results | Weak | The underlying race information exists, but the actual screen shows only a small fraction of it. |
| Profiles and settings | Prototype only | Saving, records, and rebinding functions exist, but the running game does not use them. |
| Championships and race weekends | Prototype only | Points and standings calculations exist, but there is no playable practice–qualifying–race championship flow. |
| Pit stops | Partial prototype | Basic service works in tests, but strategy, proper pit-lane behavior, box assignment, and AI use are incomplete. |
| Local multiplayer | Partial | Multiple local players are supported underneath, but configuration and presentation are incomplete. |
| Network multiplayer | Not implemented | There is only a written design. |
| Windows packaging | Working | A Windows package can be built and launched. |
| Linux release | Not demonstrated | Linux packaging and clean-install interactive testing are not complete. |
| Complete roadmap experience | Not complete | The advertised start-to-finish player journey cannot currently be performed through the normal game interface. |

# What has been implemented

## 1. The core driving simulation

The strongest part of the project is its actual vehicle simulation.

Implemented systems include:

- Front-wheel-drive, rear-wheel-drive, and all-wheel-drive cars.
- Engine torque and gearing.
- Automatic transmission behavior.
- Braking and handbrake behavior.
- Tire grip and tire load behavior.
- Tire temperature and wear-related systems.
- Suspension and load transfer.
- Aerodynamic drag and downforce-related behavior.
- Weight transfer during acceleration, braking, and cornering.
- Different road surfaces and grip levels.
- Elevation, banking, crests, dips, and kerbs.
- Vehicle damage and its effect on the car.
- Fuel and tire state.
- Collision handling between cars.
- Collisions with barriers and track objects.
- Stuck-car recovery.
- Deterministic simulation, meaning the same inputs should reproduce the same result.

There is extensive automated testing around these systems. The strict verifier completed 180 scenarios and 13,710 checks without reporting a failed assertion.

For a player, this means the project is more than a visual prototype: there is a substantial racing simulation underneath it.

## 2. External car content

The six shipped cars are stored as external content rather than being permanently hard-coded into the game.

The game has support for:

- Loading vehicle definitions.
- Validating vehicle data.
- Assigning drivetrain type and vehicle class.
- Defining appearance data.
- Defining handling and physical characteristics.
- Selecting between the available cars.
- Editing some car setup values.
- Detecting invalid or incompatible car definitions.

The current menu allows the player to select a car and inspect some basic information about it.

## 3. External track content

The project ships several externally defined tracks:

- Chicane
- Grand Prix
- Parking Lot
- Sprint
- Technical

Implemented track capabilities include:

- Loading track manifests.
- Validating track geometry.
- Checkpoints.
- Start and finish lines.
- Sector markers.
- Grid positions.
- Runoff areas.
- Barriers.
- Surface types.
- Elevation and banking.
- Kerbs.
- Pit entry, exit, speed line, and service-box data.
- AI route eligibility.
- Track geometry hashes for reproducibility.

All five shipped tracks passed the track validator during the audit.

One important limitation is that every shipped track is currently configured as a closed circuit. Despite having a track called “Sprint,” there is no shipped point-to-point route demonstrating the game’s open-route support.

## 4. Basic race-session machinery

The code contains an underlying race-session system that can handle:

- A race roster.
- Human and AI entrants.
- Grid placement.
- A countdown period.
- A shared green-light simulation tick.
- Race and time-trial modes.
- Target lap counts.
- Race timing.
- Pausing and resuming.
- Restarting a session.
- Detecting finishers.
- A finishing window for cars still on track.
- Classifying non-finishers.
- Fastest laps.
- Gaps to the leader.
- A final immutable results structure.

This is real implementation, not just documentation. Automated scenarios can configure and complete multi-car races.

The problem is that the normal player interface does not expose most of this functionality.

## 5. Some race rules and penalties

Implemented rule behavior includes:

- False-start detection.
- Track-limit monitoring.
- Wrong-way warnings.
- Pit-speed penalties.
- Time penalties.
- Lap invalidation.
- Warning penalties.
- Evidence and event records associated with penalties.

However, some other penalties exist only as data types or test injections. That distinction is important and is covered below.

## 6. Basic AI driving and traffic behavior

The AI can:

- Control a car through the same controller/input layer used by the game.
- Follow a planned route.
- Accelerate, brake, and steer.
- Complete laps with most shipped cars.
- Follow a slower vehicle.
- Attempt an overtake.
- Avoid some traffic conflicts.
- Use a configurable difficulty pace multiplier.
- Produce deterministic results from the same starting conditions.
- Generate diagnostic information when it fails.

Four of the six shipped cars complete the standard four-lap AI run successfully:

- `awd_gt`
- `fwd_hot`
- `fwd_light`
- `rwd_grip`

Two do not:

- `awd_rally`
- `rwd_power`

The failure is known and reproducible.

## 7. Diagnostic and development tools

The project has unusually extensive development tooling, including:

- Headless simulation.
- Automated physics scenarios.
- Telemetry capture.
- Regression comparisons.
- State checksums.
- First-divergence reporting.
- AI failure classification.
- Track validators.
- Vehicle validators.
- Performance tests.
- Release packaging.
- A deterministic headless demo.
- Screenshot-based smoke runs.
- Replay-related infrastructure.
- A development laboratory for examining parameters and behavior.

This tooling is valuable for finishing the game. It explains why many underlying systems are relatively well tested even though the player-facing experience is incomplete.

## 8. Windows release packaging

A Windows release bundle can be created.

During the audit:

- The release executable built successfully.
- The package contained the required game data.
- The packaged executable launched from an unrelated temporary directory.
- Raylib initialized successfully.
- The game ran for 120 frames.
- A smoke screenshot was captured.
- The application shut down cleanly.

The headless acceptance demo also completed a two-lap, four-entrant race and successfully retried it.

This proves that the Windows build can launch and simulate. It does not prove that a normal player can complete the full advertised walkthrough.

# What has not been implemented

## 1. The full race setup menu

This is one of the largest missing player features.

The underlying game can represent:

- Car choice
- Track choice
- Time trial or race
- AI field size
- Number of laps
- Countdown
- Damage rules
- Recovery rules
- Weather and environment
- Driving assists

But the normal menu only lets the player:

- Select a car.
- Open the car setup editor.
- Start the current/default session.

The player cannot use the normal interface to select:

- A track.
- Race mode.
- Race distance.
- Number of AI opponents.
- Rules.
- Environment.
- Countdown behavior.
- Damage settings.
- Recovery settings.
- A full assist configuration.

There is a complete-looking race configuration function in the code, but it is called by automated tests and command-line tooling—not by the actual menu.

In player terms: the garage contains the parts for a race setup screen, but the screen itself has not been built and connected.

## 2. A complete race HUD

The project can calculate information such as:

- Current race position.
- Other entrants.
- Lap counts.
- Gaps.
- Fastest lap.
- Fuel.
- Tire condition.
- Damage.
- Penalties.
- Wrong-way status.
- Countdown time.

A special presentation snapshot gathers much of this information.

However, the actual race HUD does not use that snapshot. The snapshot is currently exercised only in automated tests.

As a result, the player does not receive the complete race information promised by the roadmap.

## 3. Proper classified results

The underlying race system creates classified results, but the actual results screen is extremely limited.

At the end of a run, the screen shows:

- “RUN COMPLETE”
- One lap time
- “P drive again”
- “R menu”

It does not display a full classification table with:

- Every driver.
- Finishing position.
- Total race time.
- Gap to the winner.
- Fastest lap.
- DNF status.
- DSQ status.
- Penalties.
- Record indicators.
- Championship points.
- Advancement to the next event.

So the calculation exists, but the player-facing results presentation does not.

## 4. Start lights and countdown presentation

The race-session countdown exists.

The game can:

- Hold cars on the grid.
- Count down a fixed number of simulation ticks.
- Release everyone at the same moment.
- Penalize a car that moves early.

What is missing is the player presentation:

- No visible start-light sequence.
- No proper countdown display in the live renderer.
- No synchronized start-light/countdown audio.
- No clear “lights out” presentation.

The automated test proves that the timer works. It does not prove that the player can see or hear it.

## 5. Player profile integration

The project contains functions for:

- Creating a default profile.
- Loading a profile.
- Saving a profile.
- Migrating versions.
- Rebinding controls.
- Detecting binding conflicts.
- Recording best laps.
- Rejecting records made against different content versions.

These functions are tested in isolation.

They are not integrated into the running game.

The game does not presently:

- Load the player profile on startup.
- Save it on exit.
- Apply saved controls.
- Apply saved settings.
- Record completed laps into it.
- Show stored records.
- Offer an actual settings screen.
- Offer an actual controls screen.
- Offer an accessibility screen.

The on-screen instructions still display fixed keys such as W, S, A, D, Space, Q, E, P, and R.

## 6. Remappable essential controls

The rebinding library exists, but the real controls remain hard-coded.

A player cannot currently open a control menu and rebind all essential actions such as:

- Steering.
- Throttle.
- Brake.
- Handbrake.
- Shifting.
- Pause.
- Restart.
- Menu navigation.
- Confirmation and cancellation.

The game also cannot update its on-screen prompts to reflect custom bindings because those prompts are literal hard-coded text.

## 7. Accessibility options

There is no complete player-facing accessibility system.

The roadmap referred to accessibility as part of the settings/profile work, but there is no connected interface demonstrating options throughout menus and races.

Missing or undemonstrated areas include:

- Accessible menu navigation.
- Customizable visual presentation.
- Alternative control behavior.
- Fully remappable menu and race actions.
- Consistent keyboard and gamepad use.
- Player-adjustable HUD/accessibility settings.
- Persistent accessibility preferences.

## 8. Practice and qualifying

There is no playable practice session.

There is no playable qualifying session.

There is no system that:

- Starts a practice session.
- Advances to qualifying.
- Times qualifying attempts.
- Produces a qualifying order.
- Handles qualifying ties.
- Uses qualifying results to create a race grid.
- Allows the player to skip an optional session.
- Applies grid penalties or grid reversal.
- Moves through a complete race weekend.

References to “qualifying” found elsewhere in the project relate to validation conditions or tuning descriptions, not a player-facing qualifying mode.

## 9. Complete championships and progression

A basic championship points calculator exists.

It can:

- Store a small event calendar.
- Assign points by finishing position.
- Apply dropped-result rules.
- Track wins.
- Break some ties.
- Serialize standings.
- Reject an incompatible serialized version.

It cannot currently provide a playable championship.

Missing elements include:

- A championship selection screen.
- A championship setup screen.
- Practice and qualifying within an event.
- Advancing from one session to another.
- Automatically applying race results to standings.
- A championship standings screen.
- Saving progress through the player profile.
- Resuming a season.
- Handling missing or changed track/car content during resume.
- Restarting or abandoning an event.
- A next-event flow.
- A full progression experience.

The points module is essentially an isolated calculator rather than a playable game mode.

## 10. Complete pit-stop gameplay

A basic pit state machine can:

- Notice that a car is in a rough pit area.
- Detect when it stops in a service box.
- Wait for a service timer.
- Replace tires.
- Add fuel.
- Repair damage.
- Apply a pit-speed penalty.
- Return the car to an exiting state.

But the full pit system has not been implemented.

Important missing or incomplete behavior includes:

- A properly authored pit-lane corridor.
- Correct crossing of pit entry and exit lines.
- Unsafe-entry rules.
- Unsafe-exit rules.
- Mandatory-stop rules.
- Properly assigning a different service box to each entrant.
- Multi-car pit congestion.
- Player pit-request controls.
- An actual pit strategy interface.
- AI pit strategy.
- AI decisions based on fuel.
- AI decisions based on tire wear.
- AI decisions based on damage.
- AI serving penalties in the pits.

At present, every entrant is assigned service box number zero when service starts.

The pit “lane” is also approximated using service-box areas and circles around the entry and exit, rather than a proper continuous lane.

## 11. AI pit strategy

The AI has traffic behavior, but no complete race strategy.

It does not currently make meaningful decisions such as:

- “I need fuel.”
- “My tires are worn.”
- “I should repair damage.”
- “I must serve a penalty.”
- “This is the best lap to stop.”
- “My assigned pit box is occupied.”
- “I need to avoid another car entering the pits.”

The code supports storing tire, fuel, and repair requests, but those requests are set manually by tests. They are not produced by AI race strategy.

## 12. Full penalty enforcement

The project defines several penalty types, including:

- Track limits.
- Cutting the track.
- Wrong-way driving.
- False starts.
- Avoidable contact.
- Pit speeding.

Production gameplay automatically handles:

- Track limits.
- Wrong-way warnings.
- False starts.
- Pit speeding.

But it does not automatically detect and enforce:

- A shortcut that gains time or position.
- Responsibility for avoidable contact.
- Serving a penalty through a pit procedure.
- Disqualification.
- A complete escalating penalty lifecycle.

The tests for cuts and avoidable contact call the penalty function directly. In other words, they prove that the game can store such a penalty after being told one happened; they do not prove that the game can recognize that it happened.

## 13. Complete DNF, DNS, and DSQ statuses

Race results can distinguish a finisher from a non-finisher using a simple true/false value.

They do not have a complete final-status system for:

- Finished.
- DNF.
- DNS.
- DSQ.
- Retired.
- Abandoned.

The championship points test has a separate way to represent disqualification, but the race-session classification itself does not generate a proper DSQ row.

That means a future results screen does not yet have authoritative, complete status information to display.

## 14. A shipped point-to-point event

The track system contains some code for open, point-to-point routes.

However, all five shipped tracks are configured as closed loops.

Therefore the game does not currently ship a real example of:

- A point-to-point sprint.
- A finish at the end of an open route.
- Point-to-point timing.
- Point-to-point records.
- Point-to-point AI completion.
- Point-to-point classification.

## 15. Complete track discovery information

The track data includes the technical information needed to simulate a track, but it does not contain the full player-facing information requested by the roadmap.

Missing or incomplete metadata includes:

- Track author.
- License.
- Track type.
- Driving direction.
- Supported game modes.
- Preview identity.
- Fully exposed grid capacity.
- Complete selection-screen information.

The player also cannot browse and select tracks in the current menu.

## 16. Complete track preview tooling

There is a tool that generates a top-down SVG picture of a track.

It can draw:

- The route.
- Racing surface.
- Runoff.
- Barriers.
- Checkpoints.
- Sectors.
- Grid positions.
- Pit markers.

Its description claims that it also draws elevation, banking, and kerb profile information in a side strip. The code does not actually do that.

The preview generator also has no automated golden-image test, so changes to its output are not currently protected by the normal verification suite.

## 17. Complete performance evidence

The project has a working performance benchmark.

On the audit machine, it tested:

- 1 car
- 4 cars
- 8 cars

All three met the stated 120 Hz budget on the chicane track.

What remains untested against the original issue is:

- 16 cars or the declared maximum field.
- Multiple track sizes.
- A large synthetic track.
- Dense pileups.
- Wet conditions.
- A full AI fleet.
- Worst-frame or tail-latency measurements.
- Per-simulation-stage timing.
- Runtime allocation measurements.

So performance looks promising, but the full promised scale has not been proven.

## 18. Complete Windows/Linux release support

Windows packaging works.

Linux has some non-blocking automated quality checks, but there is no equivalent demonstrated Linux release process that:

- Builds the interactive game.
- Runs the complete test suite as a required gate.
- Creates the release package.
- Installs it cleanly.
- Launches it interactively.
- Verifies the same selectable content.
- Completes a smoke test comparable to Windows.

The project’s documentation says Linux is supported, but the release evidence does not yet prove the full claim.

## 19. Network multiplayer

Network multiplayer has not been implemented.

The project has a written design describing possible:

- Session compatibility checks.
- Input synchronization.
- Tick ordering.
- Checksum exchange.
- Desync reporting.
- Join and leave rules.

But there is no working:

- Network transport.
- Lobby.
- Host/join flow.
- Peer connection.
- Latency simulation.
- Packet-loss handling.
- Packet-reordering handling.
- Disconnect/reconnect behavior.
- Network content compatibility rejection.
- Live network checksum exchange.
- Network desync detection.

Local multiplayer is partly implemented, but network multiplayer is currently a design document.

# Level of implementation

The most useful distinction is between four levels.

## Fully or substantially implemented

These systems have real production code and meaningful automated coverage:

- Core car physics.
- Multiple drivetrain types.
- Tires, suspension, load transfer, and surfaces.
- Vehicle content loading and validation.
- Track content loading and validation.
- Multi-car simulation.
- Car-to-car collisions.
- Grid placement.
- Countdown state.
- False-start detection.
- Timing and sector calculations.
- Basic finishing and classification.
- Basic AI route following.
- AI traffic following and passing.
- Deterministic checksums.
- Telemetry and regression testing.
- Windows build and package launch.

These may still need polish, but they are genuine functional systems.

## Implemented underneath but not connected to the player experience

These systems exist as models, libraries, or test-only flows:

- Full session configuration.
- Race-presentation snapshots.
- Player profiles.
- Saving settings.
- Control rebinding.
- Persistent records.
- Championship points and standings.
- Ghost storage.
- Complete classified race data.
- Retry helper.
- Pit service requests.

This is the largest source of misleading “completion.” A test can call these systems directly, but a normal player cannot necessarily access them.

## Partially implemented prototypes

These systems contain a working slice but do not cover the full feature:

- AI racecraft.
- Pit stops.
- Penalties.
- Championships.
- Local multiplayer.
- Track preview generation.
- Cross-platform packaging.
- Performance benchmarking.
- Final classification statuses.
- Race HUD.

## Not implemented

These areas are effectively absent:

- Practice sessions.
- Qualifying sessions.
- A complete race weekend.
- A playable championship progression loop.
- A full race setup menu.
- A complete settings menu.
- A complete controls menu.
- A complete accessibility menu.
- Visible start lights.
- Full classified results presentation.
- Next-event flow.
- AI pit strategy.
- Network multiplayer.
- A shipped point-to-point event.
- A verified Linux release package and interactive clean-install test.

# Known issues, errors, and bugs

## 1. Two AI cars fail the required four-lap test

This is the clearest current gameplay defect.

The test claims that one shared AI configuration completes a four-lap run with all six cars. In reality:

- `awd_rally` completes only one lap.
- `rwd_power` completes only one lap.

`awd_rally` spends approximately 82% of its run off-track and becomes stalled.

`rwd_power` spends approximately 85% of its run off-track and ends with a disagreement between the AI’s planned location and the game’s authoritative track location.

The test still passes because both cars are explicitly exempted from the completion assertion.

This is not merely missing evidence. It is a known behavioral failure being treated as acceptable by the verification gate.

## 2. The test description is misleading

The scenario is described as:

> “uniform AiDriverConfig completes the full run on all 6 roster cars”

But it does not require that outcome for all six cars.

It requires it for only four cars and applies reduced structural checks to the two known failures.

The wording and the actual pass criteria should be brought back into agreement.

## 3. Green tests overstate feature completion

The full test suite passes because several scenarios verify only a narrow internal component:

- The HUD scenario verifies a data snapshot, not the actual displayed HUD.
- The championship scenario verifies points, not a playable championship.
- The multiplayer scenario verifies local inputs, not network multiplayer.
- The penalty scenario directly inserts some penalties instead of detecting the underlying behavior.
- The session configuration scenario calls the configuration function directly rather than navigating a real menu.
- The profile scenario saves and loads a profile in isolation rather than integrating it into game startup and shutdown.
- The package smoke test runs for 120 frames rather than completing the release walkthrough.

The tests are not necessarily wrong at the component level. The issue is that their names and use as closure evidence imply a greater level of feature completeness than they demonstrate.

## 4. The normal start path bypasses the full session configuration system

The menu’s start command does not use the full race configuration path. It restarts the existing/default session.

This is why a headless test can launch a configured AI race while a normal player cannot choose the same options.

## 5. The results screen ignores the classified results system

The session builds detailed result rows, but the renderer displays only one lap time.

This is a connection defect between the game’s authoritative data and its player-facing presentation.

## 6. The profile system is unused by the game

The save/load/rebinding/record code exists but has no production integration.

Consequences include:

- Settings do not persist.
- Bindings do not affect actual controls.
- Completed laps do not update player records.
- On-screen prompts cannot reflect custom controls.
- Profile migration is not exercised through real startup.

## 7. Pit box assignment is incorrect for multiple cars

Every car is assigned pit box zero when it stops.

That is incompatible with the intended multi-entrant pit system and will create incorrect service-box behavior once multiple cars try to pit.

## 8. Pit-lane detection is an approximation

The pit-lane code considers a car to be in the lane when it is:

- Inside a service box, or
- Within ten metres of the pit-entry marker, or
- Within ten metres of the pit-exit marker.

This does not model a continuous pit lane. A car may leave those small areas and no longer be considered in the pit lane even if it is geographically between entry, box, and exit.

## 9. Wrong-way and track-limit state share a counter

The wrong-way warning logic reuses an off-track counter as an “already warned” marker.

This may work in the presently tested configurations, but it couples two different rule concepts and makes interaction between wrong-way and track-limit enforcement more fragile.

## 10. Cut and contact penalties are not detected automatically

The game can store those penalties, but it does not determine:

- Whether a shortcut produced an advantage.
- Which car caused a collision.
- Whether the contact was avoidable.
- What consequence should follow.

That behavior is simulated in tests by directly telling the system to add a penalty.

## 11. No complete DSQ path

The race classification does not contain a proper disqualification status.

Championship scoring can be manually told that an entrant was disqualified, but there is no authoritative race rule that produces and displays a DSQ.

## 12. Missing controller database file in the packaged smoke run

The Windows package smoke output reported:

> `data/input/gamecontrollerdb.txt` failed to open

The game continued running, so this was not fatal. Nevertheless, it suggests that the packaged build may be missing the intended external controller-mapping database.

That could affect recognition or correct mapping of less common gamepads and should be investigated as a packaging defect.

## 13. Compiler warnings

The builds completed, but generated warnings including:

- Missing initialization for recently added track elevation fields in older fixtures and fallback code.
- A function without a previous prototype.
- Possible string truncation when reading ghost track/car identifiers.
- Possible path-message truncation in validation tooling.
- Static-analysis warnings involving error handling in track-manifest file loading.

None caused the current test suite to fail. They are still technical debt and, in the case of truncated identifiers or paths, could become real bugs with unusually long or malformed input.

## 14. Release documentation claims more than the release evidence proves

The release document describes a complete player walkthrough, but the current menu cannot perform it.

The document also describes Linux support more strongly than the required CI and packaging evidence currently demonstrates.

## 15. Internal tracker contradiction

The tracker marks individual issues complete while leaving the corresponding phase gates unchecked.

Examples of still-unchecked statements include:

- A player can choose the complete session.
- AI can use pits.
- The player receives understandable classified results.
- Practice, qualifying, races, and championships share the session system.
- Settings and bindings persist.
- Accessibility works through menus and sessions.
- Windows and Linux packages pass clean-install tests.
- Network compatibility and desync detection work.

The unchecked gates describe the actual state more accurately than the checked issue rows.

# What remains to finish the roadmap

## Priority 1: Create a real player setup flow

Build and connect a menu that lets a player select:

- Car.
- Car setup.
- Track.
- Time trial or race.
- Lap count.
- AI field size.
- AI difficulty.
- Damage.
- Recovery.
- Track-limit rules.
- Pit rules.
- Weather and environment.
- Driving assists.

The menu must call the existing session configuration system rather than simply restarting a default session.

Success means a fresh player can launch the packaged game and configure an ordinary AI race without a command line.

## Priority 2: Connect the race presentation to the renderer

The HUD should consume the existing presentation snapshot and display:

- Current position.
- Lap and target laps.
- Session time.
- Countdown/start state.
- Nearby competitors or race order.
- Gaps.
- Fastest lap.
- Fuel.
- Tire wear.
- Damage.
- Penalties.
- Wrong-way warning.
- Pit status.

This should work for both keyboard and gamepad users.

## Priority 3: Build the classified results screen

The results screen needs to display:

- Finishing order.
- Driver/car identity.
- Finish time.
- Gap to leader.
- Best lap.
- Fastest-lap holder.
- Penalties.
- Finished/DNF/DNS/DSQ status.
- Personal record.
- Track record where applicable.
- Retry.
- Return to menu.
- Next session or next event.

The race-session result model must first be expanded to represent all final statuses cleanly.

## Priority 4: Integrate profiles, settings, controls, and records

The game must:

- Load a profile during startup.
- Recover safely from a missing or corrupt profile.
- Apply saved settings.
- Apply saved control bindings.
- Save changes safely.
- Save records after authoritative session finalization.
- Show record information to the player.
- Provide settings, controls, and accessibility menus.
- Update button/key prompts dynamically.
- Verify migration using a real application startup path.

## Priority 5: Fix the AI roster failures

The shared AI controller must be made capable of completing the standard run with:

- `awd_rally`
- `rwd_power`

This should be fixed without:

- Giving those cars private AI configuration values.
- Giving AI hidden grip or power.
- Weakening the track.
- Shortening the run.
- Exempting the cars.
- Reclassifying a failed run as success.

Afterward, the test must assert four completed laps and a passing classification for all six cars.

## Priority 6: Complete AI race behavior

Add AI decisions for:

- Fuel usage.
- Tire wear.
- Damage.
- Pit timing.
- Service requests.
- Serving penalties.
- Mandatory stops.
- Recovery after contact.
- Safe pit entry and exit.
- Avoiding occupied service boxes.

AI should use the same physical and rule systems as the player.

## Priority 7: Complete the penalty system

Implement production detection and enforcement for:

- Advantage gained by cutting.
- Avoidable contact.
- Contact responsibility.
- Repeated infringements.
- Pit-entry and pit-exit violations.
- Mandatory-stop violations.
- Penalty serving.
- Disqualification.

The tests should create the actual driving circumstances and verify that the game detects them. They should not directly insert the expected penalty.

## Priority 8: Complete pits

Replace the approximate pit area with proper authored lane behavior.

Add:

- Entry-line crossing.
- Exit-line crossing.
- Speed-limit start and end.
- Continuous pit-lane containment.
- One assigned box per entrant.
- Occupancy and congestion handling.
- Player service requests.
- AI service requests.
- Mandatory-stop tracking.
- Safe release.
- Appropriate pit penalties.

## Priority 9: Add practice, qualifying, and race weekends

Implement a real session sequence:

1. Practice
2. Qualifying
3. Grid formation
4. Race
5. Classification
6. Event results
7. Next event

Add:

- Optional session skipping.
- Qualifying ties.
- Grid penalties.
- Grid reversal if required.
- Restart/abandon rules.
- Shared AI and physics behavior across all session types.

## Priority 10: Turn the championship calculator into a game mode

Connect the existing points code to:

- Menu selection.
- Event calendar.
- Session progression.
- Race results.
- Standings display.
- Save/resume.
- Content-version validation.
- Missing-content errors.
- Next-event flow.
- Championship completion.

## Priority 11: Ship and test a point-to-point route

Either make the existing Sprint track genuinely point-to-point or add another open route.

Verify:

- Start behavior.
- Final checkpoint behavior.
- Point-to-point finish.
- Timing.
- Records.
- AI completion.
- Classification.
- Menu metadata.

## Priority 12: Finish track discovery and previews

Add player-facing track metadata such as:

- Type.
- Length.
- Direction.
- Sector count.
- Grid capacity.
- Supported modes.
- Pit availability.
- Author.
- License.
- Preview.

Finish the preview tool’s promised elevation, banking, and kerb strip, and add automated deterministic output testing.

## Priority 13: Finish local multiplayer as a player feature

The underlying local input ownership needs a real setup flow for:

- Adding local players.
- Assigning controllers.
- Selecting cars.
- Configuring the session.
- Starting the race.
- Displaying each player’s information.
- Classifying all players independently.
- Handling reconnect or controller loss.

## Priority 14: Implement network multiplayer or explicitly descope it

Because multiplayer was classified as post-core, there are two honest options:

- Reopen #59 and implement the network requirements, or
- Split local and network multiplayer into separate issues, close only the completed local portion, and leave network multiplayer open.

The issue should not remain closed while the project explicitly says networking is not implemented.

## Priority 15: Prove Linux release support

Add a required Linux release job that:

- Builds the game.
- Runs relevant tests.
- Builds the interactive executable.
- Creates a package.
- Launches from a clean directory.
- Confirms bundled content is available.
- Performs a smoke check.
- Produces release evidence.

## Priority 16: Expand performance verification

Benchmark:

- 1 car.
- 8 cars.
- 16 cars.
- Maximum supported field.
- Small and large tracks.
- Dense contacts.
- Wet conditions.
- A full AI grid.
- Pit-lane congestion.

Report:

- Average simulation time.
- Worst or high-percentile simulation time.
- Time per major simulation stage.
- Runtime allocations.
- Available headroom.

## Priority 17: Perform the actual release walkthrough

Once the features are connected, test the package as a normal player would:

1. Install or extract the clean package.
2. Launch without developer tools.
3. Configure car, track, mode, rules, environment, and AI field.
4. Start from a valid grid.
5. Observe start lights and countdown.
6. Race against AI.
7. Experience contact, penalties, damage, tires, fuel, and pits.
8. Finish and view complete classification.
9. Save records.
10. Retry.
11. Continue to the next event.
12. Exit.
13. Relaunch.
14. Confirm settings, controls, records, and championship progress persisted.

That walkthrough—not merely a passing headless simulation—is the clearest definition of roadmap completion.

## Bottom line

Circuit-C is currently best described as:

> A well-tested racing simulation foundation with several partially implemented game systems, but without the complete menus, presentation, persistence, race-weekend structure, AI strategy, and release workflow needed to call it a finished player-facing racing game.

The physics and low-level race machinery are much further along than the visible product. The main work remaining is not to restart the project; it is to connect the existing systems, fill the missing rule and strategy behavior, fix the two failing AI cars, and prove the complete experience through the packaged application.