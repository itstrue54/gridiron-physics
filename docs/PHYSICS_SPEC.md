# Football Physics Engine Spec

Nov 7, 2026 · @Peter

## Overview & goals

Build `gridiron-physics`: a deterministic, top-down 2D rigid-body engine in pure Kotlin, tuned for 22 players and one ball on a regulation field. The engine owns motion, contact and ball flight. The game layer owns ratings, AI, play logic and rules.

1. **Realistic player motion:** momentum, acceleration and deceleration limits, traction-limited cuts, and collisions that respect mass and speed.
2. **Contact data for football outcomes:** every hit reports enough data (impulse, angle, closing speed) for the game layer to decide tackles, broken tackles and blocks.
3. **Believable ball flight:** passes, punts, kickoffs and field goals in 2.5D (ground plane plus height), with bounces and drag.
4. **Bit-for-bit determinism:** the same seed and the same inputs produce the same result on every device, which enables replays and future head-to-head play.
5. **Mobile performance:** 60 Hz with headroom on a mid-range Android phone, and zero memory allocations per step.

## Scope & non-goals

The engine simulates bodies and emits state and events; it never renders, decides plays or applies football rules.

**In scope**

- Rigid bodies: dynamic, kinematic and static
- Shapes: circle, capsule, convex polygon (8 vertices max), segment; any shape can be a sensor
- Contact solver with friction and restitution
- Joints: grapple (wrap-up), block engagement, distance, weld
- 2.5D ball flight with drag, bounces and goal-post checks
- Queries: raycast, circle cast, area query, nearest bodies
- Event stream: contacts, sensors, out of bounds, goal line crossed, catch opportunities
- Desktop debug viewer, replay files, benchmark harness

**Non-goals**

- Rendering, animation or sprites (the game reads body state)
- Full 3D, ragdolls or limb simulation
- AI, play calling, player ratings, penalties or game rules
- Networking (the engine only guarantees determinism)
- Third-party physics libraries of any kind

## Tech stack & module layout

Kotlin 2.x on a JDK 21 toolchain, built with Gradle (Kotlin DSL). Tests use JUnit 5; the debug viewer uses Compose Desktop; benchmarks use kotlinx-benchmark. The modules can be added to any existing Kotlin multi-module project.

**Target platforms:** Android, Windows, macOS and Linux are all first-class. `physics-core` uses only the Kotlin standard library and `java.lang.StrictMath`, so the same artifact runs unchanged on all four.

| Platform | Runtime | Engine | Debug viewer | CI |
| --- | --- | --- | --- | --- |
| Android 8.0+ (API 26) | ART | Yes | No (in-app overlay later) | Instrumented tests on an emulator |
| Windows 10/11 x64 | JDK 21 | Yes | Yes (Compose Desktop) | `windows-latest` runner |
| macOS 13+ (Apple Silicon and Intel) | JDK 21 | Yes | Yes (Compose Desktop) | `macos-latest` runner |
| Linux x64 | JDK 21 | Yes | Yes (Compose Desktop) | `ubuntu-latest` runner |

Rules that keep it portable: no file paths, threads or OS calls in `physics-core`; replay files use little-endian binary with a version header; Gradle wrapper scripts are committed for both `gradlew` and `gradlew.bat`.

&#91;embedded content: module layout · 5 modules\]

Only `physics-core` ships inside the game. The viewer and benchmarks are development tools that depend on it, and the game layer is the only code that translates ratings into physics.

## Units, coordinates, field model

All internal math is SI: meters, kilograms, seconds and radians, stored as `Double`. Yard conversion lives only in a `Yards` helper for the game layer.

- **Origin:** the back corner of the home end zone, on the near sideline.
- **Axes:** +x runs down the field (0 to 109.728 m), +y runs across it (0 to 48.768 m), and +z is height (used by the ball only).
- **Field config:** a `FieldConfig` data class holds every field constant, so pro and college layouts are presets, not code changes.

| Constant | Default | Notes |
| --- | --- | --- |
| Field length incl. end zones | 109.728 m (120 yd) |  |
| Field width | 48.768 m (53⅓ yd) |  |
| End zone depth | 9.144 m (10 yd) |  |
| Hash spacing | 5.639 m pro / 12.192 m college | Preset in `FieldConfig` |
| Crossbar height | 3.048 m (10 ft) | Used by kick checks |
| Upright spacing | 5.639 m (18 ft 6 in) | Same in both presets |
| Runoff buffer | 6 m beyond each line | Static walls stop bodies leaving the world |
| Ball mass | 0.41 kg | Ground collider: circle, r = 0.09 m |
| Player radius | 0.30–0.38 m | Set per body type by the game layer |
| Player mass | 80–160 kg | Set from roster data |
| Player top speed | up to \~10 m/s | Set from ratings; tunable |

Sidelines, end lines and goal lines are **sensors**, not walls. Bodies cross them and the engine emits an event; the game layer decides what it means.

## World & step loop

The world advances in fixed 1/60 s ticks. A frame-time accumulator runs at most 4 ticks per rendered frame and exposes an interpolation alpha for smooth drawing. A variable timestep is never allowed.

Each `world.step()` runs these phases in this order:

1. **Apply intents:** read each player's locomotion target from the game layer and convert it to forces.
2. **Integrate velocities:** apply forces and damping; gravity acts on ball height only.
3. **Broad phase:** find candidate pairs, sorted by body id.
4. **Narrow phase:** build contact manifolds (up to 2 points each).
5. **Solve velocities:** warm start, then run contact and joint constraints for 8 iterations (configurable).
6. **Integrate positions.**
7. **Relax:** run soft position correction for 3 iterations (configurable).
8. **Ball flight:** advance height and check ground, goal posts and catch windows.
9. **Events:** emit contact begin/end, sensor crossings, out of bounds, goal line and catch opportunities.
10. **Bookkeeping:** update sleep states, increment the tick counter, and optionally compute the state hash.

## Bodies & shapes

Players are circles while upright and capsules while diving or down. Everything else is built from the same four shapes.

**Body types:** `Dynamic` (players, loose ball), `Kinematic` (scripted movers such as officials), `Static` (goal posts, runoff walls).

**Body fields:** `id: Int`, `type`, `position`, `angle`, `linearVelocity`, `angularVelocity`, `mass`, `invMass`, `inertia`, `linearDamping`, `angularDamping`, `shapes`, `userData: Int` (game entity handle), `flags` (sleeping, bullet, stance).

| Shape | Used for | Notes |
| --- | --- | --- |
| Circle | Upright players, ball on the ground | Fastest tests; the default |
| Capsule | Diving, falling and downed players | Oriented along the body angle |
| Convex polygon | Goal-post bases, sleds, props | 8 vertices max; SAT tests |
| Segment | Runoff walls, line sensors | Static only |

**Player stance** is a body flag the game layer sets: `Upright`, `Diving`, `Down`. Changing stance swaps the collider in place. Moving players that contact a `Down` body raise a `TripContact` event.

**Collision filtering** uses 16-bit category and mask bits: `HOME`, `AWAY`, `BALL`, `OFFICIAL`, `STATIC`, `LINE_SENSOR`. Player-to-ball contact is sensor-only; possession is game logic, never an impulse.

## Collision pipeline

With about 25 dynamic bodies, the broad phase starts as brute force; the interface allows a grid later without touching the solver.

**Broad phase:** a `BroadPhase` interface. v1 tests all pairs (25 bodies is about 300 pairs). v2 is a uniform grid with 2 m cells, added only if the benchmark requires it. Pair lists are always sorted by `(minId, maxId)` for determinism.

**Narrow phase:** one function per shape pair, each tested in both argument orders.

- Circle–circle, circle–capsule, capsule–capsule
- Circle–polygon, capsule–polygon, polygon–polygon (SAT)
- Circle–segment, capsule–segment

Each test outputs a manifold: a unit normal, up to 2 contact points, penetration depth and feature ids. Feature ids let the solver match contacts frame to frame for warm starting.

**Continuous collision (CCD):** applies to the ball and to any body flagged `bullet`. It uses a swept circle against static segments and polygons, so a 30 m/s ball never passes through an upright. Player-to-player CCD is not needed at player speeds.

**Queries** (for AI and game logic): `raycast`, `circleCast`, `queryAabb`, `nearestBodies(point, count, filter)`. Queries are read-only and never allocate.

## Contact solver & materials

Use a sequential-impulse solver (Erin Catto / Box2D style) with accumulated impulse clamping, warm starting and soft contacts.

- **Normal impulse:** clamp the accumulated value at ≥ 0.
- **Friction impulse:** clamp at ±μ × normal impulse.
- **Restitution:** apply only when the closing speed exceeds 1 m/s, so players don't jitter while in contact.
- **Soft contact:** use a spring-damper formulation (contact hertz 30, damping ratio 10, max push-out 3 m/s). All values are configurable.
- **Iterations:** 8 velocity iterations, 3 relax iterations.
- **Sleeping:** a body sleeps after 0.5 s below 0.05 m/s; any contact or intent wakes it.

Materials resolve per contact pair through a `MaterialTable`. All values below are tunable starting points, not measured data.

| Pair | Friction μ | Restitution |
| --- | --- | --- |
| Player – player | 0.6 | 0.02 |
| Player – downed player | 0.8 | 0.0 |
| Player – goal post / wall | 0.4 | 0.1 |
| Ball – turf (bounce) | 0.5 | 0.5 |
| Ball – goal post | 0.3 | 0.6 |
| Ball – player (deflection) | 0.3 | 0.3 |

The **surface** setting (natural grass, artificial turf, wet, frozen) is a `SurfaceConfig` that scales locomotion traction and ball bounce. It does not change the contact table.

## Football systems: locomotion, tackling, blocking, pile-ups

The engine supplies the physical tools for football outcomes; the game layer decides the outcomes. Ratings never enter the engine directly. The game layer maps them to `AthleteParams`.

### Locomotion controller

Each player body has a `LocomotionController` that turns an intent into a force each tick.

- **Intent (from the game layer):** a target velocity or a target point, effort (0–1) and stance.
- **AthleteParams:** `maxSpeed`, `maxAccel`, `maxDecel`, `turnRateAtRest`, `turnRateAtMax` (the turn rate falls as speed rises), `traction`, `balance`.
- **Traction limit:** total horizontal acceleration ≤ traction × surface multiplier × g. Starting value: traction 0.9 for cleats on dry turf.
- **Slip:** an intent that exceeds the traction limit is clamped and emits a `Slip` event with the excess. The game layer may turn it into a stumble.

### Tackling

Every player–player contact emits a `HitEvent`:

| Field | Meaning |
| --- | --- |
| `a`, `b` | Body ids, lower id first |
| `point`, `normal` | Contact location and direction |
| `closingSpeed` | Relative speed along the normal at impact (m/s) |
| `normalImpulse` | Impulse applied on the first tick (N·s) |
| `angleToHeading` | Hit angle relative to each body's heading (rad) |
| `momentumA`, `momentumB` | Momentum vector of each body before impact |

To act on a tackle attempt, the game layer creates a **GrappleJoint**: a soft distance joint between tackler and carrier.

- It has a `breakForce`, set from the tackler's strength against the carrier's balance. The joint breaks when exceeded and emits `GrappleBroken`.
- It applies a `dragForce` that slows the carrier. Multiple grapples on one carrier stack.
- The game layer decides when the carrier is down and switches the stance to `Down`.

### Blocking

A **BlockJoint** couples a blocker and a defender while they are engaged.

- Each side pushes with up to `maxPushForce` (from the game layer). The net force moves the pair, so a stronger blocker drives the defender back.
- The defender sheds the block when the lateral force exceeds `shedThreshold`. The joint breaks and emits `BlockShed`.
- Leverage: push is scaled by the cosine of the contact angle against the pusher's heading, so a blocker who is beaten to the side loses force.

### Pile-ups and forward progress

In top-down 2D, a pile is a cluster of bodies, not a stack. The engine tracks the ball carrier's maximum +x (or −x) position while the carrier is upright and holding the ball, and reports it as `forwardProgressX` on the `Down` event. The game layer uses that value to spot the ball.

## Ball flight (2.5D)

While airborne, the ball moves in a 3D point model (x, y, z), separate from the 2D rigid-body world. It joins the 2D world as a circle body once it lands.

**State:** position (x, y, z), velocity (vx, vy, vz), and `flightMode` (`Spiral`, `Wobble`, `EndOverEnd`).

**Forces:** gravity (9.81 m/s² on z), an optional wind vector, and quadratic drag:

```latex
\vec{F}_d = -\tfrac{1}{2}\,\rho\,C_d\,A\,\lvert\vec{v}_{rel}\rvert\,\vec{v}_{rel}
```

Here ρ = 1.225 kg/m³, A is the ball's cross-section, and the velocity is measured relative to the wind. Cd is set per flight mode: spiral lowest, end-over-end highest. All values are tunable.

**Launch helpers** (used by QB and kicker AI):

- `launchVelocityFor(target, apexHeight)`: solves the velocity to reach a point at a chosen arc height.
- `launchVelocityFor(target, speed)`: solves the lower or higher arc for a given release speed.
- `timeToArrive(launch, target)`: lets AI lead receivers.

The solvers iterate against the drag model, not the vacuum formula, so the ball lands where the AI aims.

**Kicks:** goal-post checks run in the x–z plane at the end line. The crossbar is a segment at 3.048 m; the uprights are vertical segments. The engine emits `KickResult` (`Good`, `WideLeft`, `WideRight`, `Short`, `HitPost`).

**Catch windows:** the ball is catchable when z is between 0.3 m and the player's `reachHeight` (game-supplied; jumping raises it) and the ball is within `reachRadius`. The engine emits `CatchOpportunity` with ball speed and arrival angle; the game layer decides the catch.

**Bounces:** when the ball reaches the ground, it bounces using the turf restitution, plus a seeded random deflection (±35° yaw, tunable) to mimic a prolate ball. It then rolls with friction as a loose-ball body.

**Deflections:** `deflect(bodyId, impulse)` applies a tip or batted-ball impulse from game logic.

## Determinism & performance

The same seed, initial state and per-tick inputs must produce an identical `stateHash()` on every tick, on every device.

**Determinism rules**

1. Use `Double` everywhere. Desktop JVMs have used strict IEEE double math since Java 17 (JEP 306), and Android's ARM64 runtime also uses IEEE doubles. The cross-device test in Testing must prove it.
2. Use `StrictMath` for `sin`, `cos`, `atan2` and `exp`, never `Math` or `kotlin.math`. `sqrt` is exact and allowed. Wrap these in one `DMath` object so a future iOS/multiplatform build can swap in its own implementations.
3. Never iterate a `HashMap` or `HashSet` inside `step()`. Use arrays ordered by body id.
4. Use one seeded PRNG (SplitMix64) owned by the `World`. Never use `Random()` or `Math.random()`.
5. No clocks, threads or coroutines inside `step()`.
6. `stateHash()` is an FNV-1a hash over the raw bits of every body's position, angle and velocity, the ball state and the tick count.

**Performance budgets**

| Metric | Budget |
| --- | --- |
| `step()` time, 25 dynamic bodies | ≤ 0.5 ms on a mid-range Android phone |
| Heap allocations per `step()` | 0 |
| Ticks per second, headless desktop | ≥ 20,000 (for fast whole-game simulation) |

Use preallocated arrays and object pools for contacts, manifolds and events. Events go into a ring buffer that the game drains each tick.

## Public API

The game layer sees one small surface. Everything else is `internal`.

```kotlin
class World(config: WorldConfig, field: FieldConfig, seed: Long) {
    fun createBody(def: BodyDef): BodyId
    fun destroyBody(id: BodyId)
    fun setStance(id: BodyId, stance: Stance)
    fun setIntent(id: BodyId, intent: LocomotionIntent)
    fun setAthlete(id: BodyId, params: AthleteParams)

    fun createGrapple(tackler: BodyId, carrier: BodyId, def: GrappleDef): JointId
    fun createBlock(blocker: BodyId, defender: BodyId, def: BlockDef): JointId
    fun destroyJoint(id: JointId)

    fun launchBall(from: Vec3, velocity: Vec3, mode: FlightMode)
    fun deflect(id: BodyId, impulse: Vec2)
    fun launchVelocityFor(from: Vec3, target: Vec2, apexHeight: Double): Vec3

    fun step()                          // exactly one 1/60 s tick
    fun advance(frameSeconds: Double): Double  // runs 0–4 ticks; returns alpha

    fun events(): EventReader           // drained each tick
    fun body(id: BodyId): BodyView      // read-only snapshot
    fun ball(): BallView
    fun raycast(from: Vec2, to: Vec2, filter: Filter): RayHit?
    fun nearestBodies(p: Vec2, count: Int, filter: Filter, out: IntArray): Int

    fun stateHash(): Long
    fun snapshot(): WorldSnapshot
    fun restore(s: WorldSnapshot)
}
```

`Vec2` and `Vec3` are mutable value holders pooled inside the engine; the API never returns a freshly allocated vector during play.

## Debug tools

Build the debug viewer in Milestone 1, before any collision code; physics bugs are hard to find without seeing them.

**Desktop viewer** (`physics-debug` module, Compose Desktop, never shipped in the app):

- Draws the field lines, hashes and sensors.
- Draws bodies by stance, velocity vectors, contact points with normals and impulse magnitudes, and joints colored by load against the break force.
- Draws the ball with a ground shadow and a height label.
- Controls: pause, single-step, 0.25× / 1× / 4× speed, click a body to inspect every field.

**Replays:** a `.gpr` file holds the seed, initial snapshot and per-tick intents. Loading a replay must reproduce every recorded `stateHash`; the viewer flags the first tick that diverges.

**Scenario scripts:** Kotlin DSL setups that run headless in tests and visually in the viewer.

- Head-on tackle: 90 kg back vs 115 kg linebacker at full speed
- Sideline run: carrier at top speed, pursuer at an angle
- Double-team block on a 150 kg lineman
- Deep pass to the sideline with wind
- Punt that bounces near the goal line
- 45-yard field goal that hits the upright

## Testing & acceptance criteria

A milestone is done only when its tests below pass in CI and the matching scenario looks right in the viewer.

| Test | Setup | Pass criterion |
| --- | --- | --- |
| Momentum conservation | Two free circles, restitution 1, μ = 0, head-on | Total momentum error < 1e-9; kinetic energy error < 1e-6 |
| Mass matters | 140 kg vs 90 kg, equal speed, head-on | Heavier body keeps moving forward after impact |
| Rest stability | 22 standing players, zero intent, 10 s | Max drift < 1 mm; all bodies asleep by 1 s |
| Top speed & acceleration | Locomotion, effort 1 from rest | Reaches `maxSpeed` within ±1%; time to 90% speed within ±2% of the analytic value |
| Traction limit | Hard cut at top speed | Horizontal acceleration never exceeds traction × g; `Slip` emitted |
| Vacuum ball range | 25 m/s at 45°, drag off | Lands at 63.71 m ±1 cm |
| Drag sanity | Same launch, drag on | Range shorter than vacuum; spiral range > wobble range |
| Launch solver | 500 random targets 5–55 m | Lands within 0.25 m of the target |
| No tunneling | Ball at 30 m/s into an upright, 1,000 random angles | Zero pass-throughs |
| Grapple break | Carrier force above `breakForce` | Joint breaks within 1 tick; `GrappleBroken` emitted |
| Determinism, same device | 1,000 seeded random plays, run twice | Identical `stateHash` on every tick |
| Determinism, cross-device | Same replays on Windows, macOS, Linux and an Android device | Identical hash sequences |
| Performance | Benchmark, 25 bodies | ≤ 0.5 ms per step on device; 0 allocations per step |

Every narrow-phase function also gets unit tests in both argument orders, plus property-based tests on random shape pairs: the normal is unit length, and penetration is ≥ 0 when overlapping.

## Milestones

Give Claude Code one milestone per session. Each gate maps to rows in the Testing table.

&#91;embedded content: milestones · 8 phases, each with a test gate\]

M1 through M3 already produce a playable feel test: players running, cutting and colliding in the viewer. Ball flight waits until M6 because it depends on CCD and the event system.

## CLAUDE.md rules

Save this spec as `docs/PHYSICS_SPEC.md` in the repo, and put the block below in the repo's root `CLAUDE.md` so Claude Code follows it every session.

```markdown
# gridiron-physics — rules for Claude Code

Spec: docs/PHYSICS_SPEC.md. Read it before any physics work.

## Hard rules
- physics-core is pure Kotlin. No Android, no third-party physics or math libraries.
- Targets: Android, Windows, macOS, Linux. No OS-specific code, paths or threads in physics-core.
- CI runs the full test suite on ubuntu-latest, windows-latest, macos-latest and an Android emulator.
- Fixed timestep 1/60 s only. Never pass frame time into step().
- SI units internally. Yards appear only in the Yards helper.
- Determinism: no HashMap/HashSet iteration in step(), no Random(), no clocks,
  no threads. Trig goes through DMath (StrictMath). Pairs sorted by body id.
- Zero allocations inside step(). Use pools and preallocated arrays.
- The engine never decides football outcomes. It emits events; the game decides.

## Workflow
- Work one milestone at a time. Do not start the next until its acceptance
  tests pass.
- Write the test first, then the code. Run ./gradlew :physics-core:test
  after every change.
- Every new narrow-phase function gets tests in both argument orders.
- Every new feature gets a scenario script viewable in physics-debug.
- If a determinism test fails, stop and fix it before any other work.
- Commit after each green milestone with message "M<n>: <summary>".

## References
- Erin Catto, GDC talks on sequential impulses and soft constraints
- Box2D v3 design notes (solver structure only; do not copy code)
- Ian Millington, Game Physics Engine Development
```

Start each Claude Code session with a prompt like: *"Implement Milestone 2 from docs/PHYSICS\_SPEC.md. Write the acceptance tests first, run them, then implement until green."*
