# gridiron-physics — design

Date: 2026-11-07
Source spec: [`docs/PHYSICS_SPEC.md`](../../PHYSICS_SPEC.md) (Peter, Nov 7 2026). This document does not restate the spec's
physics. It records the why, the decisions made on top of it, and the gaps it left open.
Where the two disagree, the spec wins unless this document says it deliberately overrides it.

## Intent and the why

- `gridiron-physics` is a **standalone, reusable** football physics engine. It must not be tied to any one game.
- Its first user is a **new, separate version of Gridiron Dynasty** that runs on Android, Windows, macOS and Linux.
  The existing Gridiron Dynasty (`AndroidStudioProjects/gridiron-dynasty-app`) is a text-only Android sim. The new
  version is a separate project, designed later in its own brainstorm. It is out of scope here.
- The engine works the way the existing game's `:engine` already does: pure Kotlin, seeded RNG, determinism as a
  product requirement. A Gridiron Dynasty developer should find it familiar.

## Goals

The spec's five goals, unchanged: realistic player motion, contact data for football outcomes, believable 2.5D ball
flight, bit-for-bit determinism, and 60 Hz mobile performance with zero allocations per step.

## Non-goals

The spec's non-goals, plus:

- No game code in this repo. Ratings→`AthleteParams` mapping, play logic and rules belong to each game.
- No game-specific names in `physics-core` (no "Gridiron Dynasty", no team names, no roster types).
- Publishing to a package registry is deferred (see Approaches).

## Anti-goals (it fails even if it "works")

- The engine decides a football outcome (a tackle, a catch, a block won). It may only emit events.
- Any nondeterminism: two runs with the same seed and inputs ever disagree on `stateHash()`.
- `physics-core` gains a dependency beyond the Kotlin standard library, or uses an OS, file, thread or clock API.
- `step()` allocates on the heap.
- A game has to fork or patch the engine to use it.

## Constraints

- **Toolchain:** Gradle 9.4.1 wrapper (already on this machine, matches Gridiron Dynasty), Kotlin 2.2.10 (matches),
  JDK 21 toolchain (Android Studio's bundled JBR 21 is present).
- **Bytecode target: Java 11** for `physics-core` (deliberate override of an implied JDK 21 target). The existing
  Gridiron Dynasty compiles at Java 11, and Android API 26 consumers need a conservative target. The toolchain stays
  JDK 21; only `jvmTarget`/`release` is 11. Desktop-only modules (`physics-debug`, `physics-bench`) may target 21.
- **Determinism rules** as in spec §Determinism, enforced by tests and by a CI grep that bans `Math.`, `kotlin.math`,
  `java.util.Random`, `HashMap`/`HashSet` iteration, `System.nanoTime`/`currentTimeMillis` and `Thread` in
  `physics-core/src/main`.
- **One milestone per session.** A milestone is done only when its acceptance tests pass and its scenario looks
  right in the viewer. The user checks it in the viewer before the next milestone starts.
- **Commits:** `M<n>: <summary>` after each green milestone, per the spec's CLAUDE.md.

## Approaches (how games consume the engine)

| | Approach | Trade-off |
|---|---|---|
| **A (chosen)** | Separate repo at `C:\Users\12817\Projects\gridiron-physics`; games add `includeBuild("../gridiron-physics")` (Gradle composite build) | Engine edits are visible to the game on the next build, with no publish step. Both repos must sit side by side, including in the game's CI (two checkouts). |
| B | Separate repo, published to GitHub Packages as a versioned artifact | Clean versions and simple game CI, but every engine tweak needs a publish. That is painful while tuning feel. |
| C | Module inside the new game's repo | Simplest at first, but the engine is no longer standalone. Rejected: it works against the reason the engine exists. |

The user picked **A**. Publishing (B) can be added around M8 without changing consumers beyond one line, so the
build should set real `group`/`version` coordinates from day one to keep that switch cheap.

## Decided structure

**Repo:** `C:\Users\12817\Projects\gridiron-physics`, its own git repo (default branch `main`).

**Package root:** `io.github.itstrue54.gridironphysics` (matches the owner namespace of the Gridiron Dynasty Play
listing). *My call.*

**Group / version:** `io.github.itstrue54:gridiron-physics-core:0.<milestone>.0`. *My call.*

**Modules** (the spec's "module layout · 5 modules" diagram was not exported, so this is a reconstruction; *my call*):

| Module | Contents | Ships in games? |
|---|---|---|
| `physics-core` | World, bodies, shapes, collision, solver, joints, locomotion, ball flight, events, queries, snapshot, replay encode/decode to `ByteArray`, `Yards`, `DMath`, `SplitMix64` | Yes, the only one |
| `physics-scenarios` | Kotlin DSL scenario scripts (head-on tackle, sideline run, …), shared by tests and viewer | No (test fixture) |
| `physics-debug` | Compose Desktop viewer: field, bodies, contacts, joints, ball and shadow, pause/step/speed, inspect, replay load and divergence flag | No |
| `physics-bench` | kotlinx-benchmark harness and allocation check | No |
| `physics-android-test` | Minimal Android test module that runs `physics-core`'s determinism replays as instrumented tests on an emulator | No |

Dependency direction: everything depends on `physics-core`; `physics-debug` and tests depend on `physics-scenarios`.
Nothing depends on `physics-debug`, `physics-bench` or `physics-android-test`.

**Replay file access:** `physics-core` encodes and decodes `.gpr` as a little-endian `ByteArray` with a version header,
and never touches paths. File I/O lives in `physics-debug` and tests.

**Repo files:** `docs/PHYSICS_SPEC.md` (spec copy, already saved), root `CLAUDE.md` (the spec's block, verbatim),
`README.md` with how a game adds `includeBuild`, Gradle wrapper scripts for both `gradlew` and `gradlew.bat`,
`.github/workflows/ci.yml` with the spec's four-runner matrix.

## Milestones (reconstructed; the spec's 8-phase table was not exported)

The spec fixes three anchor points: the viewer comes in **M1, before any collision code**; **M1–M3** together give a
playable feel test; **ball flight is M6** because it needs CCD and events. The rest is *my call*. Each gate names
rows from the spec's Testing table.

| # | Scope | Test gate | Viewer scenario |
|---|---|---|---|
| **M1** | Gradle scaffold, CLAUDE.md, CI; `Vec2`/`Vec3`, `DMath`, `SplitMix64`, `Yards`; `FieldConfig` (pro and college), `WorldConfig`; `World` with fixed 1/60 step, `advance()` accumulator (≤4 ticks, alpha); bodies (dynamic, kinematic, static) with circle shape only; velocity and position integration, damping; `stateHash()`; `physics-debug` viewer drawing field, hashes, line sensors, bodies, velocity vectors, with pause/step/speed/inspect | Determinism, same device (run twice, identical hash every tick, small random scenario); fixed-step and accumulator unit tests; `Yards` round trip | Free-moving bodies drifting with damping on a correct field |
| **M2** | Shapes (capsule, polygon ≤8, segment, sensor flag); filtering bits; brute-force `BroadPhase` with sorted pairs; all 8 narrow-phase pairs in both orders; manifolds with feature ids; sequential-impulse solver with warm start, soft contact, friction, restitution threshold, relax; `MaterialTable`; sleeping; runoff walls | Momentum conservation; Mass matters; Rest stability; narrow-phase both-order and property tests | Head-on tackle (collision only, no locomotion yet) |
| **M3** | `LocomotionController`, `LocomotionIntent`, `AthleteParams`, turn-rate falloff, traction limit, `SurfaceConfig`; stance and collider swap (circle/capsule) | Top speed and acceleration; Traction limit (with `Slip`) | Sideline run; head-on tackle with locomotion. **Playable feel test.** |
| **M4** | Event ring buffer and `EventReader`; contact begin/end, `HitEvent`, sensors (sidelines, end lines, goal lines), out of bounds, goal line crossed, `TripContact`; queries `raycast`, `circleCast`, `queryAabb`, `nearestBodies` | Event and query unit tests (spec has no table row, so the gate is my call) | Events overlaid in the viewer |
| **M5** | Joints: grapple (break force, drag, stacking), block (push, leverage, shed), distance, weld; forward-progress tracking and `Down` event | Grapple break | Double-team block on a 150 kg lineman; tackle with grapple |
| **M6** | 2.5D ball: flight modes, drag, wind, launch solvers, `timeToArrive`; CCD for ball and bullets; bounces with seeded yaw; ground handoff to loose-ball body; kick checks and `KickResult`; catch windows; `deflect` | Vacuum ball range; Drag sanity; Launch solver; No tunneling | Deep pass to the sideline with wind; punt bouncing near the goal line; 45-yard FG hitting the upright |
| **M7** | `snapshot`/`restore`; `.gpr` replay format; viewer replay loading with first-divergence flag; 1,000-play seeded random harness; Android instrumented module; full CI matrix | Determinism, same device (1,000 plays); Determinism, cross-device | Any replay, loaded and verified |
| **M8** | Zero-allocation audit (pools, ring buffers); kotlinx-benchmark; uniform grid broad phase only if the benchmark needs it; optional publishing (approach B) | Performance (≤0.5 ms device, 0 alloc, ≥20k ticks/s desktop) | All scenarios at 4× |

Zero allocation is a rule from M1 on, but it is only *gated* in M8. Earlier milestones must not introduce patterns
that M8 would have to rewrite (no per-step lists, lambdas, boxing or iterators in `step()`).

## Decisions settled after the builder check (*my calls*)

1. **Replays record every mutating call, not only intents.** A replay that stores only intents would diverge at the
   first tackle or pass. From M1, every public `World` method that changes state (`createBody`, `destroyBody`,
   `setStance`, `setIntent`, `setAthlete`, joint create/destroy, `launchBall`, `deflect`, wind/surface/carrier
   setters) goes through an internal command path. That path can record a tick-stamped command log without
   allocating. In M7, `.gpr` = header + seed + initial snapshot + command log + per-tick `stateHash` list.
2. **Airborne ball vs goal posts is 3D.** The goal-post assembly is engine-owned geometry from `FieldConfig`: two
   uprights as vertical cylinders (radius 0.05 m) from the crossbar (3.048 m) to 3.048 + 10.668 m (35 ft), a
   crossbar cylinder, and a gooseneck base. The airborne ball runs swept-sphere CCD against these in 3D and
   bounces off them with ball–post material (μ 0.3, e 0.6). `KickResult` is classified when the ball crosses the
   end-line plane, or with `HitPost` on contact. The gooseneck base is also a 2D static polygon for players and the
   ground ball. 2D CCD (swept circle against static segments and polygons) applies to the ground ball and `bullet`
   bodies. The No-tunneling test fires the airborne ball at an upright in 3D.
3. **The spec's Public API is a minimum. Additions are allowed** when a spec feature needs them: `setBallCarrier(id?)`
   (possession set by the game; ends flight; drives forward-progress tracking), `setAttackDirection(+1/-1)`,
   `createDistance`, `createWeld`, `setWind`, `setSurface`, `circleCast`, `queryAabb`, `timeToArrive`, and the
   speed-based `launchVelocityFor` overload. The ball states are `Dead` → `Airborne` ↔ `Loose` (2D body) → `Carried`.
4. **Locomotion acceleration tapers linearly with speed:** `a = maxAccel · (1 − v/maxSpeed)` along the run
   direction. This gives the exponential sprint curve `v(t) = maxSpeed · (1 − e^(−t·maxAccel/maxSpeed))` seen in real
   sprinters, rather than a robotic constant-accel ramp. The "analytic value" for the 90% test is
   `t90 = (maxSpeed/maxAccel) · ln 10`. "Reaches `maxSpeed` ±1%" is checked after `5 · maxSpeed/maxAccel`
   seconds. Deceleration uses `maxDecel` without taper. Everything stays clamped by traction.
5. **Momentum/energy test runs with production solver settings.** Restitution is applied Box2D-v3 style: the
   pre-solve normal velocity is stored, and a restitution pass runs after relax. That gives an exact elastic
   response for a single circle–circle contact at e = 1, μ = 0, while soft contact still governs penetration
   recovery. If energy error misses 1e-6 with soft push-out active, the test may disable contact softness through
   `WorldConfig`. That is allowed because the spec's setup row describes an idealized case, but it must be recorded
   in the test.

## Hosting (user's decision)

**Public GitHub repo** `itstrue54/gridiron-physics` (the same account as Gridiron Dynasty). It is created and pushed
when M1 is green, so the desktop CI matrix (ubuntu, windows, macos) runs from M1 on. The Android emulator job joins
in M7 along with `physics-android-test`. Public means free
runner minutes, including macOS. M7's cross-device gate closes on CI results from all four.

## Known limits of verification on this machine

- **Cross-device determinism** needs macOS, Linux and an Android device. Locally only Windows runs. The CI matrix
  covers the desktop OSes once the repo is on GitHub. The Android emulator job needs a GitHub-hosted emulator
  action. Whether a local emulator exists in Android Studio has not been checked.
- **≤0.5 ms on a mid-range Android phone** can only be measured on a device. Locally we gate the desktop budget.
- **"Looks right in the viewer"** is the user's call at the end of each milestone.

## Left to the builder

- Exact Compose Multiplatform plugin version (must be compatible with Kotlin 2.2.10) and kotlinx-benchmark version.
- Internal class layout inside `physics-core` (struct-of-arrays versus pooled body objects), provided the public API
  matches the spec and `step()` stays allocation-free.
- Viewer visual styling (colors, layout).
- Default `Cd` values per flight mode, within the spec's ordering (spiral < wobble < end-over-end).
- Whether `BodyId`/`JointId` are `@JvmInline value class`es over `Int` (recommended) or plain `Int`.
