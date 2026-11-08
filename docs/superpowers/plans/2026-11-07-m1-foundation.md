# M1: Foundation, World Loop and Debug Viewer — Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Stand up the `gridiron-physics` repo with a deterministic fixed-step `World` of circle bodies (no collision yet), a state hash, scenario scripts, and a Compose Desktop viewer that draws the field and moving bodies.

**Architecture:** `physics-core` is pure Kotlin (stdlib + `StrictMath`) and compiles to Java 11 bytecode. Bodies live in a preallocated array indexed by `BodyId`. `step()` runs the spec's ten phases as private functions, most of them empty until later milestones. `physics-scenarios` holds a small DSL that builds worlds for both tests and the viewer. `physics-debug` is a Compose Desktop app that calls `advance()` each frame and draws interpolated positions.

**Tech Stack:** Kotlin 2.2.10, Gradle 9.4.1 (wrapper), JDK 21 toolchain, JUnit 5.11.4, Compose Multiplatform 1.9.0 (desktop), GitHub Actions.

**Spec:** `docs/superpowers/specs/2026-11-07-gridiron-physics-design.md` (design) and `docs/PHYSICS_SPEC.md` (physics spec). Read both before starting.

## Global Constraints

- `physics-core` depends on the Kotlin stdlib only. No third-party libraries, no `android.*`, no `java.io`/`java.nio.file`, no threads, clocks or coroutines.
- `physics-core` bytecode target is **Java 11** (`jvmTarget = 11`, `-Xjdk-release=11`, Java `release = 11`); toolchain is JDK 21.
- Package root: `io.github.itstrue54.gridironphysics`. Gradle group `io.github.itstrue54`, version `0.1.0`.
- All internal math is SI (m, kg, s, rad) in `Double`. Yards appear only in `Yards`.
- Fixed timestep `DT = 1.0 / 60.0`. `step()` never takes a time argument. `advance()` runs at most 4 ticks per call.
- Trig, `exp` and `sqrt` go through `DMath` (backed by `StrictMath`). Never `Math.*` or `kotlin.math.*` in `physics-core/src/main`.
- No `HashMap`/`HashSet`/`mutableMapOf`/`mutableSetOf` in `physics-core/src/main`. Iterate bodies in id order.
- Randomness only from `SplitMix64` owned by `World` (seeded by the constructor's `seed`).
- No allocation inside `step()`: no lists, lambdas, boxing, iterators or `Vec2` creation there (gated in M8; don't create debt now).
- Every public `World` mutator calls the matching method on an optional `CommandRecorder` (design decision 1).
- Field constants (spec §Units): length 109.728 m, width 48.768 m, end zone 9.144 m, hash spacing 5.639 m pro / 12.192 m college, crossbar 3.048 m, upright spacing 5.639 m, upright height above crossbar 10.668 m, runoff 6 m.
- Solver defaults stored in `WorldConfig` now (used from M2): 8 velocity iterations, 3 relax iterations, contact hertz 30, damping ratio 10, max push-out 3 m/s, sleep after 0.5 s below 0.05 m/s, gravity 9.81.
- Commits: `M1: <summary>` messages; end each with `Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>`.

## Review Focus

1. **Huge frame time** (app resumed after a 10 s pause): `advance(10.0)` runs exactly 4 ticks, drops the rest, and returns alpha in `[0, 1)`. Test in Task 5.
2. **Zero, negative or NaN frame time:** `advance(0.0)` and `advance(-1.0)` run 0 ticks and return the current alpha; `advance(NaN)` throws `IllegalArgumentException`. Test in Task 5.
3. **Accumulator rounding:** sixty calls of `advance(1.0/60.0)` run exactly 60 ticks, not 59. Test in Task 5.
4. **Bad body data from a roster** (NaN/Infinity position or velocity, mass ≤ 0 on a dynamic body, radius ≤ 0): `createBody` throws `IllegalArgumentException` instead of poisoning the hash later. Test in Task 4.
5. **Id misuse:** creating more than `maxBodies` bodies throws `IllegalStateException`. `destroyBody` on an unknown or destroyed id, and `body()` on a destroyed id, throw `IllegalArgumentException`. Ids are never reused. Test in Task 4.

---

## File Structure

```
settings.gradle.kts                     modules + foojay toolchain resolver
build.gradle.kts                        plugin aliases (apply false)
gradle/libs.versions.toml               versions
gradle/wrapper/*, gradlew, gradlew.bat  copied from gridiron-dynasty-app (9.4.1)
.gitignore, CLAUDE.md, README.md
.github/workflows/ci.yml

physics-core/build.gradle.kts
physics-core/src/main/kotlin/io/github/itstrue54/gridironphysics/
  math/DMath.kt            StrictMath wrappers
  math/Vec2.kt, Vec3.kt    mutable holders
  math/SplitMix64.kt       PRNG
  math/Fnv1a64.kt          hash mixer
  Yards.kt                 yard <-> meter
  FieldConfig.kt           field constants + PRO/COLLEGE presets
  WorldConfig.kt           solver/step settings + DT constant
  BodyId.kt                BodyId value class, BodyType
  ShapeDef.kt              sealed ShapeDef (Circle only in M1)
  BodyDef.kt               creation params + validation
  BodyView.kt              read-only flyweight
  CommandRecorder.kt       mutation hook interface
  internal/Body.kt         internal mutable body record
  internal/BallState.kt    placeholder ball state (Dead) hashed from M1
  World.kt                 public API, step phases, advance, stateHash
physics-core/src/test/kotlin/io/github/itstrue54/gridironphysics/
  MathTest.kt, ConfigTest.kt, WorldBodiesTest.kt, AdvanceTest.kt,
  StateHashTest.kt, DeterminismTest.kt, DeterminismLintTest.kt

physics-scenarios/build.gradle.kts
physics-scenarios/src/main/kotlin/io/github/itstrue54/gridironphysics/scenarios/
  Scenario.kt              Scenario, ScenarioBuilder, Tags
  Scenarios.kt             registry + M1 "Drift" scenario
physics-scenarios/src/test/kotlin/.../scenarios/ScenariosTest.kt

physics-debug/build.gradle.kts
physics-debug/src/main/kotlin/io/github/itstrue54/gridironphysics/debug/
  ViewTransform.kt         world<->screen mapping (pure, tested)
  FieldPainter.kt          field, hashes, sensors
  BodyPainter.kt           bodies, velocity vectors, selection
  ViewerState.kt           pause/step/speed/selection/world
  Main.kt                  window, frame loop, controls, inspect panel
physics-debug/src/test/kotlin/.../debug/ViewTransformTest.kt
```

---

### Task 1: Repo scaffold

**Files:**
- Create: `settings.gradle.kts`, `build.gradle.kts`, `gradle/libs.versions.toml`, `.gitignore`, `CLAUDE.md`, `README.md`, `physics-core/build.gradle.kts`
- Copy: `gradle/wrapper/gradle-wrapper.jar`, `gradle/wrapper/gradle-wrapper.properties`, `gradlew`, `gradlew.bat` from `C:\Users\12817\AndroidStudioProjects\gridiron-dynasty-app`
- Test: `physics-core/src/test/kotlin/io/github/itstrue54/gridironphysics/SmokeTest.kt`

**Interfaces:**
- Produces: version catalog aliases `libs.plugins.kotlin.jvm`, `libs.plugins.kotlin.compose`, `libs.plugins.compose.multiplatform`, `libs.junit.bom`; modules `:physics-core` (this task), `:physics-scenarios`, `:physics-debug` (included later by Tasks 7–8).

- [ ] **Step 1:** `git init -b main`. Write `.gitignore` (`.gradle/`, `build/`, `.idea/`, `*.iml`, `local.properties`, `.kotlin/`).
- [ ] **Step 2:** Copy the wrapper files. Confirm `gradle-wrapper.properties` says `gradle-9.4.1-bin.zip`.
- [ ] **Step 3:** Write `libs.versions.toml`: `kotlin = "2.2.10"`, `composeMultiplatform = "1.9.0"`, `junit = "5.11.4"`. Plugins: `org.jetbrains.kotlin.jvm`, `org.jetbrains.kotlin.plugin.compose` (version.ref kotlin), `org.jetbrains.compose`. Library: `junit-bom`.
- [ ] **Step 4:** `settings.gradle.kts`: pluginManagement repos (`gradlePluginPortal()`, `mavenCentral()`, `google()`), `org.gradle.toolchains.foojay-resolver-convention` 1.0.0, dependencyResolutionManagement repos (`mavenCentral()`, `google()`), `rootProject.name = "gridiron-physics"`, `include(":physics-core")`.
- [ ] **Step 5:** `physics-core/build.gradle.kts`: kotlin jvm plugin, `group`/`version` per constraints, `kotlin { jvmToolchain(21); compilerOptions { jvmTarget = JVM_11; freeCompilerArgs.add("-Xjdk-release=11") } }`, `tasks.withType<JavaCompile> { options.release = 11 }`, test deps `platform(libs.junit.bom)`, `kotlin("test")`, `junit-jupiter`, runtimeOnly `junit-platform-launcher`, `tasks.test { useJUnitPlatform() }`. No `implementation` deps.
- [ ] **Step 6:** Write `SmokeTest.kt` with `@Test fun buildRuns() { assertEquals(2, 1 + 1) }`.
- [ ] **Step 7:** Run `./gradlew :physics-core:test`. Expected: `BUILD SUCCESSFUL`, 1 test passed.
- [ ] **Step 8:** Write `CLAUDE.md` with the spec's "CLAUDE.md rules" block verbatim, plus one line: `Design: docs/superpowers/specs/2026-11-07-gridiron-physics-design.md`. Write `README.md`: one paragraph on what the engine is, the module table from the design, and the consumer snippet `includeBuild("../gridiron-physics")` + `implementation("io.github.itstrue54:physics-core:0.1.0")`.
- [ ] **Step 9:** Commit (include `docs/`): `M1: scaffold Gradle build, CLAUDE.md, spec and design`.

---

### Task 2: Math foundation (DMath, Vec2/Vec3, SplitMix64, Fnv1a64, Yards)

**Files:**
- Create: `math/DMath.kt`, `math/Vec2.kt`, `math/Vec3.kt`, `math/SplitMix64.kt`, `math/Fnv1a64.kt`, `Yards.kt`
- Test: `MathTest.kt` (delete `SmokeTest.kt`)

**Interfaces:**
- Produces:
  - `object DMath { fun sin(a: Double): Double; fun cos(a: Double): Double; fun atan2(y: Double, x: Double): Double; fun exp(a: Double): Double; fun sqrt(a: Double): Double; fun log(a: Double): Double; fun abs(a: Double): Double; fun min(a: Double, b: Double): Double; fun max(a: Double, b: Double): Double; fun clamp(v: Double, lo: Double, hi: Double): Double; fun hypot2(x: Double, y: Double): Double /* sqrt(x*x+y*y), not StrictMath.hypot */ ; const val PI: Double }` (all `StrictMath`-backed)
  - `class Vec2(var x: Double = 0.0, var y: Double = 0.0) { fun set(x: Double, y: Double): Vec2; fun set(o: Vec2): Vec2; fun length(): Double; fun lengthSq(): Double }`
  - `class Vec3(var x: Double = 0.0, var y: Double = 0.0, var z: Double = 0.0) { fun set(x: Double, y: Double, z: Double): Vec3 }`
  - `class SplitMix64(seed: Long) { var state: Long; fun nextLong(): Long; fun nextDouble(): Double /* [0,1) */; fun nextDouble(lo: Double, hi: Double): Double }`
  - `object Fnv1a64 { const val OFFSET: Long = -0x340d631b7bdddcdbL; const val PRIME: Long = 0x100000001b3L; fun mixByte(h: Long, b: Int): Long; fun mixLong(h: Long, v: Long): Long /* 8 bytes, little-endian */; fun mixDouble(h: Long, d: Double): Long /* doubleToRawLongBits */ }`
  - `object Yards { const val METERS_PER_YARD = 0.9144; fun toMeters(yards: Double): Double; fun fromMeters(meters: Double): Double }`

- [ ] **Step 1: Write the failing tests**

```kotlin
@Test fun splitMix64MatchesReferenceVectors() {
    val r = SplitMix64(0L)
    assertEquals(0xe220a8397b1dcdafuL.toLong(), r.nextLong())
    assertEquals(0x6e789e6aa1b965f4uL.toLong(), r.nextLong())
    assertEquals(0x06c45d188009454fuL.toLong(), r.nextLong())
}
@Test fun splitMix64DoubleInUnitRange() {
    val r = SplitMix64(42L); repeat(10_000) { val d = r.nextDouble(); assertTrue(d >= 0.0 && d < 1.0) }
}
@Test fun fnvMatchesReferenceVectors() {
    assertEquals(0xcbf29ce484222325uL.toLong(), Fnv1a64.OFFSET)
    assertEquals(0xaf63dc4c8601ec8cuL.toLong(), Fnv1a64.mixByte(Fnv1a64.OFFSET, 'a'.code))
}
@Test fun fnvMixLongIsLittleEndianBytes() {
    var h = Fnv1a64.OFFSET; for (i in 0 until 8) h = Fnv1a64.mixByte(h, ((0x0102030405060708L ushr (8 * i)) and 0xFF).toInt())
    assertEquals(h, Fnv1a64.mixLong(Fnv1a64.OFFSET, 0x0102030405060708L))
}
@Test fun yardsRoundTrip() {
    assertEquals(109.728, Yards.toMeters(120.0), 1e-12)
    assertEquals(48.768, Yards.toMeters(160.0 / 3.0), 1e-12)
    assertEquals(45.0, Yards.fromMeters(Yards.toMeters(45.0)), 1e-12)
}
@Test fun dMathUsesStrictMath() {
    assertEquals(StrictMath.sin(1.234), DMath.sin(1.234)); assertEquals(StrictMath.atan2(0.3, -0.7), DMath.atan2(0.3, -0.7))
    assertEquals(StrictMath.exp(-2.5), DMath.exp(-2.5)); assertEquals(5.0, DMath.hypot2(3.0, 4.0))
}
```

- [ ] **Step 2:** Run `./gradlew :physics-core:test`. Expected: compile FAIL (unresolved references).
- [ ] **Step 3:** Implement the listed types. SplitMix64 `nextLong`: `state += 0x9e3779b97f4a7c15; z = state; z = (z xor (z ushr 30)) * 0xbf58476d1ce4e5b9; z = (z xor (z ushr 27)) * 0x94d049bb133111eb; return z xor (z ushr 31)`. `nextDouble = (nextLong() ushr 11) * 2^-53`.
- [ ] **Step 4:** Run `./gradlew :physics-core:test`. Expected: all PASS.
- [ ] **Step 5:** Commit: `M1: add DMath, vectors, SplitMix64, FNV-1a and Yards`.

---

### Task 3: FieldConfig and WorldConfig

**Files:**
- Create: `FieldConfig.kt`, `WorldConfig.kt`
- Test: `ConfigTest.kt`

**Interfaces:**
- Produces:
  - `data class FieldConfig(val lengthM: Double = 109.728, val widthM: Double = 48.768, val endZoneDepthM: Double = 9.144, val hashSpacingM: Double = 5.639, val crossbarHeightM: Double = 3.048, val uprightSpacingM: Double = 5.639, val uprightHeightAboveCrossbarM: Double = 10.668, val runoffM: Double = 6.0)` with derived `val homeGoalLineX get() = endZoneDepthM`, `val awayGoalLineX get() = lengthM - endZoneDepthM`, `val midfieldX get() = lengthM / 2`, `val nearHashY get() = (widthM - hashSpacingM) / 2`, `val farHashY get() = (widthM + hashSpacingM) / 2`; `companion object { val PRO = FieldConfig(); val COLLEGE = FieldConfig(hashSpacingM = 12.192) }`
  - `data class WorldConfig(val maxBodies: Int = 64, val maxTicksPerFrame: Int = 4, val velocityIterations: Int = 8, val relaxIterations: Int = 3, val contactHertz: Double = 30.0, val contactDampingRatio: Double = 10.0, val maxPushoutSpeed: Double = 3.0, val sleepTimeS: Double = 0.5, val sleepSpeed: Double = 0.05, val gravity: Double = 9.81, val hashEveryTick: Boolean = false)` with `init` requiring positive counts; `companion object { const val DT: Double = 1.0 / 60.0 }`

- [ ] **Step 1: Write the failing tests**

```kotlin
@Test fun proPresetMatchesSpec() { val f = FieldConfig.PRO
    assertEquals(109.728, f.lengthM); assertEquals(48.768, f.widthM); assertEquals(9.144, f.homeGoalLineX)
    assertEquals(100.584, f.awayGoalLineX, 1e-12); assertEquals(54.864, f.midfieldX, 1e-12)
    assertEquals(5.639, f.farHashY - f.nearHashY, 1e-12) }
@Test fun collegePresetOnlyChangesHashes() {
    assertEquals(12.192, FieldConfig.COLLEGE.hashSpacingM); assertEquals(FieldConfig.PRO.uprightSpacingM, FieldConfig.COLLEGE.uprightSpacingM) }
@Test fun worldConfigDefaultsMatchSpec() { val c = WorldConfig()
    assertEquals(8, c.velocityIterations); assertEquals(3, c.relaxIterations); assertEquals(4, c.maxTicksPerFrame)
    assertEquals(30.0, c.contactHertz); assertEquals(10.0, c.contactDampingRatio); assertEquals(3.0, c.maxPushoutSpeed)
    assertEquals(1.0 / 60.0, WorldConfig.DT) }
@Test fun worldConfigRejectsNonPositiveCounts() { assertFailsWith<IllegalArgumentException> { WorldConfig(maxBodies = 0) } }
```

- [ ] **Step 2:** Run `./gradlew :physics-core:test`. Expected: compile FAIL.
- [ ] **Step 3:** Implement both classes as listed.
- [ ] **Step 4:** Run `./gradlew :physics-core:test`. Expected: PASS.
- [ ] **Step 5:** Commit: `M1: add FieldConfig presets and WorldConfig defaults`.

---

### Task 4: Bodies and the World step loop

**Files:**
- Create: `BodyId.kt`, `ShapeDef.kt`, `BodyDef.kt`, `BodyView.kt`, `CommandRecorder.kt`, `internal/Body.kt`, `internal/BallState.kt`, `World.kt`
- Test: `WorldBodiesTest.kt`

**Interfaces:**
- Consumes: `WorldConfig`, `FieldConfig`, `SplitMix64`, `DMath` (Tasks 2–3).
- Produces:
  - `@JvmInline value class BodyId(val value: Int)`; `enum class BodyType { Dynamic, Kinematic, Static }`
  - `sealed class ShapeDef { class Circle(val radius: Double) : ShapeDef() }`
  - `class BodyDef(var type: BodyType = BodyType.Dynamic, var x: Double = 0.0, var y: Double = 0.0, var angle: Double = 0.0, var vx: Double = 0.0, var vy: Double = 0.0, var angularVelocity: Double = 0.0, var mass: Double = 100.0, var linearDamping: Double = 0.0, var angularDamping: Double = 0.0, var shape: ShapeDef = ShapeDef.Circle(0.34), var userData: Int = 0, var bullet: Boolean = false)`
  - `class BodyView` (reused flyweight; valid until the next `world.body()` call) with read-only `id: BodyId, type, x, y, prevX, prevY, angle, vx, vy, angularVelocity, mass, invMass, inertia, linearDamping, angularDamping, radius, userData, bullet, awake`
  - `interface CommandRecorder { fun onCreateBody(tick: Long, id: BodyId, def: BodyDef); fun onDestroyBody(tick: Long, id: BodyId) }` (later milestones add one method per new mutator)
  - `class World(val config: WorldConfig, val field: FieldConfig, seed: Long)` with `var recorder: CommandRecorder?`, `val tick: Long`, `val rng: SplitMix64` (internal), `fun createBody(def: BodyDef): BodyId`, `fun destroyBody(id: BodyId)`, `fun body(id: BodyId): BodyView`, `fun bodyIds(out: IntArray): Int` (fills live ids ascending, returns count), `fun step()`

- [ ] **Step 1: Write the failing tests**

```kotlin
private fun world() = World(WorldConfig(), FieldConfig.PRO, seed = 1L)
@Test fun constantVelocityBodyMovesVdtPerTick() { val w = world()
    val id = w.createBody(BodyDef(x = 10.0, y = 20.0, vx = 6.0, vy = -3.0))
    repeat(60) { w.step() }
    val b = w.body(id); assertEquals(16.0, b.x, 1e-9); assertEquals(17.0, b.y, 1e-9); assertEquals(60L, w.tick) }
@Test fun linearDampingFollowsBox2dFormula() { val w = world()
    val id = w.createBody(BodyDef(vx = 10.0, linearDamping = 2.0)); w.step()
    assertEquals(10.0 / (1.0 + WorldConfig.DT * 2.0), w.body(id).vx, 1e-12) }
@Test fun staticNeverMovesKinematicIgnoresDamping() { val w = world()
    val s = w.createBody(BodyDef(type = BodyType.Static, x = 5.0, vx = 3.0))
    val k = w.createBody(BodyDef(type = BodyType.Kinematic, vx = 3.0, linearDamping = 5.0)); repeat(60) { w.step() }
    assertEquals(5.0, w.body(s).x); assertEquals(3.0, w.body(k).x, 1e-9); assertEquals(3.0, w.body(k).vx) }
@Test fun prevPositionIsPositionBeforeLastStep() { val w = world()
    val id = w.createBody(BodyDef(vx = 6.0)); w.step(); w.step(); assertEquals(0.1, w.body(id).prevX, 1e-12) }
@Test fun circleMassProperties() { val w = world()
    val b = w.body(w.createBody(BodyDef(mass = 100.0, shape = ShapeDef.Circle(0.3))))
    assertEquals(0.01, b.invMass, 1e-15); assertEquals(0.5 * 100.0 * 0.09, b.inertia, 1e-12)
    val s = w.body(w.createBody(BodyDef(type = BodyType.Static))); assertEquals(0.0, s.invMass) }
@Test fun bodyIdsAscendingAndNeverReused() { val w = world()
    val a = w.createBody(BodyDef()); val b = w.createBody(BodyDef()); w.destroyBody(a); val c = w.createBody(BodyDef())
    val out = IntArray(8); assertEquals(2, w.bodyIds(out)); assertEquals(listOf(b.value, c.value), out.take(2)); assertTrue(c.value > b.value) }
@Test fun recorderSeesCreateAndDestroy() { /* fake recorder appends "c:<id>@<tick>" / "d:<id>@<tick>"; create at tick 0, step, destroy at tick 1 */ }
// Review Focus 4
@Test fun rejectsBadBodyData() { val w = world()
    assertFailsWith<IllegalArgumentException> { w.createBody(BodyDef(x = Double.NaN)) }
    assertFailsWith<IllegalArgumentException> { w.createBody(BodyDef(vy = Double.POSITIVE_INFINITY)) }
    assertFailsWith<IllegalArgumentException> { w.createBody(BodyDef(mass = 0.0)) }
    assertFailsWith<IllegalArgumentException> { w.createBody(BodyDef(shape = ShapeDef.Circle(0.0))) } }
// Review Focus 5
@Test fun idMisuseFailsClearly() { val w = World(WorldConfig(maxBodies = 2), FieldConfig.PRO, 1L)
    val a = w.createBody(BodyDef()); w.createBody(BodyDef())
    assertFailsWith<IllegalStateException> { w.createBody(BodyDef()) }
    w.destroyBody(a)
    assertFailsWith<IllegalArgumentException> { w.destroyBody(a) }
    assertFailsWith<IllegalArgumentException> { w.body(a) }
    assertFailsWith<IllegalArgumentException> { w.destroyBody(BodyId(99)) } }
```

Note on `maxBodies`: ids are never reused, so `maxBodies` caps **ids ever issued**. That is deliberate in M1, because replays and determinism are simpler with stable ids. With `maxBodies = 2`, the third create throws even after a destroy. Write the test accordingly: after `destroyBody(a)`, a new create still throws.

- [ ] **Step 2:** Run `./gradlew :physics-core:test`. Expected: compile FAIL.
- [ ] **Step 3:** Implement. `World` holds `bodies: Array<Body?>(config.maxBodies)` and `nextId`. `createBody` validates (finite numbers; mass > 0 unless Static; radius > 0), computes `invMass` (0 for Static/Kinematic), `inertia = 0.5·m·r²` (`invInertia` 0 for non-dynamic), notifies `recorder`, and returns `BodyId(nextId++)`. `step()` calls private phases in spec order: `applyIntents()`, `integrateVelocities()`, `broadPhase()`, `narrowPhase()`, `solveVelocities()`, `integratePositions()`, `relax()`, `ballFlight()`, `emitEvents()`, `bookkeeping()`. In M1 only these have bodies: `integrateVelocities` (dynamic: `v *= 1/(1 + DT·linearDamping)`, `w *= 1/(1 + DT·angularDamping)`), `integratePositions` (store `prevX/prevY`, then semi-implicit Euler for dynamic and kinematic; angle += w·DT), and `bookkeeping` (tick++). The rest are empty, with a `// M<n>` comment naming the milestone that fills them. `BallState` holds `x,y,z,vx,vy,vz` (0.0) and `mode = 0` (Dead).
- [ ] **Step 4:** Run `./gradlew :physics-core:test`. Expected: PASS.
- [ ] **Step 5:** Commit: `M1: add bodies, World step phases and command recorder hook`.

---

### Task 5: Frame accumulator — `advance()`

**Files:**
- Modify: `World.kt`
- Test: `AdvanceTest.kt`

**Interfaces:**
- Produces: `fun World.advance(frameSeconds: Double): Double` (member; runs 0..`maxTicksPerFrame` ticks; returns alpha in `[0, 1)`), `val World.alpha: Double`.

- [ ] **Step 1: Write the failing tests**

```kotlin
@Test fun oneFrameOneTick() { val w = world(); w.advance(1.0 / 60.0); assertEquals(1L, w.tick) }
@Test fun sixtyFramesSixtyTicks() { val w = world(); repeat(60) { w.advance(1.0 / 60.0) }; assertEquals(60L, w.tick) }  // Review Focus 3
@Test fun halfTickGivesAlphaHalf() { val w = world(); val a = w.advance(0.5 / 60.0); assertEquals(0L, w.tick); assertEquals(0.5, a, 1e-9) }
@Test fun hugeFrameClampsToFourTicks() { val w = world(); val a = w.advance(10.0)                                   // Review Focus 1
    assertEquals(4L, w.tick); assertTrue(a >= 0.0 && a < 1.0); w.advance(0.0); assertEquals(4L, w.tick) }
@Test fun zeroNegativeNaN() { val w = world(); w.advance(0.0); w.advance(-1.0); assertEquals(0L, w.tick)          // Review Focus 2
    assertFailsWith<IllegalArgumentException> { w.advance(Double.NaN) } }
@Test fun at144HzTicksAverageSixtyPerSecond() { val w = world(); repeat(144) { w.advance(1.0 / 144.0) }; assertEquals(60L, w.tick) }
```

- [ ] **Step 2:** Run `./gradlew :physics-core:test --tests '*AdvanceTest*'`. Expected: compile FAIL.
- [ ] **Step 3:** Implement with a `Double` accumulator and a tolerance: `acc += max(frameSeconds, 0.0)`, then `while (ticks < maxTicksPerFrame && acc >= DT - EPS) { step(); acc -= DT; ticks++ }` with `EPS = 1e-9` s. Afterwards `acc = clamp(acc, 0.0, DT - EPS)`, which drops time beyond the cap and absorbs rounding. Alpha = `acc / DT`. NaN/Infinite throw `IllegalArgumentException`. Frame timing is not part of the deterministic simulation (replays record ticks, not frame times), so a `Double` accumulator is fine here. The tolerance is what makes the 60-at-60 Hz and 60-at-144 Hz tests exact. Do **not** use an integer-nanosecond accumulator: `round(1e9/144)·144` is short of one second and yields 59 ticks.
- [ ] **Step 4:** Run `./gradlew :physics-core:test`. Expected: PASS.
- [ ] **Step 5:** Commit: `M1: add frame accumulator with 4-tick cap and alpha`.

---

### Task 6: State hash and same-device determinism

**Files:**
- Modify: `World.kt`
- Test: `StateHashTest.kt`, `DeterminismTest.kt`, `DeterminismLintTest.kt`

**Interfaces:**
- Produces: `fun World.stateHash(): Long`; `val World.lastTickHash: Long` (set in `bookkeeping()` when `config.hashEveryTick`).

- [ ] **Step 1: Write the failing tests**

```kotlin
// StateHashTest
@Test fun emptyWorldHashIsStable() { assertEquals(world().stateHash(), world().stateHash()) }
@Test fun hashChangesWithAnyBodyField() { /* for each of x, y, angle, vx, vy, angularVelocity: two worlds identical except that field differs by 1 ulp (Math.nextUp in test code is fine) -> hashes differ */ }
@Test fun hashChangesWithTick() { val w = world(); val h0 = w.stateHash(); w.step(); assertNotEquals(h0, w.stateHash()) }
@Test fun hashEveryTickMatchesStateHash() { val w = World(WorldConfig(hashEveryTick = true), FieldConfig.PRO, 1L)
    w.createBody(BodyDef(vx = 1.0)); w.step(); assertEquals(w.stateHash(), w.lastTickHash) }
// DeterminismTest
@Test fun sameSeedSameHashEveryTick() {
    fun run(): LongArray { val w = randomWorld(seed = 1234L); return LongArray(600) { w.step(); w.stateHash() } }
    assertArrayEquals(run(), run()) }
@Test fun differentSeedDifferentHash() { assertNotEquals(randomWorld(1L).also { it.step() }.stateHash(), randomWorld(2L).also { it.step() }.stateHash()) }
// randomWorld(seed): 22 dynamic circles + 1 kinematic, positions/velocities/damping drawn from a SplitMix64(seed) created in the test
// DeterminismLintTest: scans physics-core/src/main/kotlin (path relative to the module dir, Gradle's test working dir)
@Test fun mainSourcesUseNoBannedApis() {
    val banned = listOf(Regex("""(?<!Strict)Math\."""), Regex("""kotlin\.math"""), Regex("""java\.util\.Random"""), Regex("""kotlin\.random"""),
        Regex("""HashMap|HashSet|hashMapOf|hashSetOf|mutableMapOf|mutableSetOf"""), Regex("""System\.(nanoTime|currentTimeMillis)"""),
        Regex("""\bThread\b|kotlinx\.coroutines"""), Regex("""java\.io\.|java\.nio\.file"""), Regex("""^import android\.""", RegexOption.MULTILINE))
    // collect "<file>:<line>: <match>" for every hit; assertTrue(hits.isEmpty(), hits.joinToString("\n")) }
```

- [ ] **Step 2:** Run `./gradlew :physics-core:test`. Expected: compile FAIL on `stateHash`.
- [ ] **Step 3:** Implement `stateHash()`: start at `Fnv1a64.OFFSET`. For each live body in ascending id, mix `id` (as Long), `x, y, angle, vx, vy, angularVelocity` (raw bits). Then mix the ball's `x, y, z, vx, vy, vz` and `mode`, then `tick`. No allocation.
- [ ] **Step 4:** Run `./gradlew :physics-core:test`. Expected: all PASS, including the lint test.
- [ ] **Step 5:** Commit: `M1: add FNV-1a state hash and determinism tests`.

---

### Task 7: Scenario DSL and the "Drift" scenario

**Files:**
- Modify: `settings.gradle.kts` (`include(":physics-scenarios")`)
- Create: `physics-scenarios/build.gradle.kts` (kotlin jvm, toolchain 21, jvmTarget 11, `api(project(":physics-core"))`, JUnit as in core), `scenarios/Scenario.kt`, `scenarios/Scenarios.kt`
- Test: `physics-scenarios/src/test/kotlin/io/github/itstrue54/gridironphysics/scenarios/ScenariosTest.kt`

**Interfaces:**
- Consumes: `World`, `BodyDef`, `BodyType`, `ShapeDef`, `FieldConfig`, `WorldConfig`, `SplitMix64`.
- Produces:
  - `object Tags { const val HOME = 0; const val AWAY = 1; const val OFFICIAL = 2; const val PROP = 3 }` (viewer color convention via `userData`)
  - `class Scenario(val name: String, val description: String, val seed: Long, private val setup: ScenarioBuilder.() -> Unit) { fun build(): World }`
  - `class ScenarioBuilder(val world: World, val rng: SplitMix64) { val field: FieldConfig; fun player(x: Double, y: Double, vx: Double = 0.0, vy: Double = 0.0, mass: Double = 100.0, radius: Double = 0.34, damping: Double = 0.5, tag: Int = Tags.HOME): BodyId; fun official(x: Double, y: Double, vx: Double, vy: Double): BodyId; fun prop(x: Double, y: Double, radius: Double): BodyId }`
  - `fun scenario(name: String, description: String, seed: Long = 1L, field: FieldConfig = FieldConfig.PRO, config: WorldConfig = WorldConfig(), setup: ScenarioBuilder.() -> Unit): Scenario`
  - `object Scenarios { val drift: Scenario; val all: List<Scenario> }`

- [ ] **Step 1: Write the failing tests**

```kotlin
@Test fun driftHas22PlayersOneOfficialOneProp() { val w = Scenarios.drift.build(); val ids = IntArray(64); val n = w.bodyIds(ids)
    assertEquals(24, n); assertEquals(11, (0 until n).count { w.body(BodyId(ids[it])).userData == Tags.HOME })
    assertEquals(11, (0 until n).count { w.body(BodyId(ids[it])).userData == Tags.AWAY }) }
@Test fun driftSlowsDownAndStaysFinite() { val w = Scenarios.drift.build(); val ids = IntArray(64); val n = w.bodyIds(ids)
    fun maxPlayerSpeed() = (0 until n).map { w.body(BodyId(ids[it])) }.filter { it.userData <= Tags.AWAY }.maxOf { DMath.hypot2(it.vx, it.vy) }
    val v0 = maxPlayerSpeed(); repeat(600) { w.step() }
    assertTrue(maxPlayerSpeed() < v0 * 0.05); (0 until n).forEach { val b = w.body(BodyId(ids[it])); assertTrue(b.x.isFinite() && b.y.isFinite()) } }
@Test fun buildIsDeterministic() { assertEquals(Scenarios.drift.build().stateHash(), Scenarios.drift.build().stateHash()) }
```

- [ ] **Step 2:** Run `./gradlew :physics-scenarios:test`. Expected: FAIL (module or symbols missing).
- [ ] **Step 3:** Implement. "Drift" (seed 7): 11 HOME players in a rough offensive set just short of midfield (x ≈ 50–54 m, y spread across the hashes) and 11 AWAY players mirrored at x ≈ 56–61 m. Masses are drawn from 85–150 kg and radii from 0.30–0.38 m (spec ranges) using the builder's rng, and initial velocities are random, up to 8 m/s, with damping 0.5. One kinematic official walks 1.2 m/s down the near sideline at y = −2.0, and one static prop (radius 0.5) sits at the home goal line center. Damping 0.5 decays velocity by `e^(−5) ≈ 0.0067` in 10 s, which satisfies the `< 0.05` assertion.
- [ ] **Step 4:** Run `./gradlew :physics-scenarios:test :physics-core:test`. Expected: PASS.
- [ ] **Step 5:** Commit: `M1: add scenario DSL and Drift scenario`.

---

### Task 8: Debug viewer (Compose Desktop)

**Files:**
- Modify: `settings.gradle.kts` (`include(":physics-debug")`)
- Create: `physics-debug/build.gradle.kts`, `debug/ViewTransform.kt`, `debug/FieldPainter.kt`, `debug/BodyPainter.kt`, `debug/ViewerState.kt`, `debug/Main.kt`
- Test: `physics-debug/src/test/kotlin/io/github/itstrue54/gridironphysics/debug/ViewTransformTest.kt`

**Interfaces:**
- Consumes: `World`, `BodyView`, `BodyId`, `FieldConfig`, `Scenarios`, `Scenario`, `Tags`.
- Produces:
  - `class ViewTransform(val field: FieldConfig, val canvasW: Float, val canvasH: Float, val marginPx: Float = 16f)` fitting the field **plus runoff** with uniform scale. Field +x maps to screen right; the near sideline (y = 0) is at the bottom, so +y maps to screen up. `fun toScreenX(x: Double): Float; fun toScreenY(y: Double): Float; fun toWorldX(sx: Float): Double; fun toWorldY(sy: Float): Double; val pxPerMeter: Float`
  - `class ViewerState { var scenario: Scenario; var world: World; var paused: Boolean; var speed: Double /* 0.25, 1.0, 4.0 */; var selected: BodyId?; var alpha: Double; fun reset(); fun stepOnce(); fun frame(frameSeconds: Double); fun pick(wx: Double, wy: Double): BodyId? }`

- [ ] **Step 1: Write the failing tests**

```kotlin
private val t = ViewTransform(FieldConfig.PRO, canvasW = 1280f, canvasH = 720f)
@Test fun originIsBottomLeftOfFieldInsideRunoff() {
    assertEquals(16f + 6f * t.pxPerMeter, t.toScreenX(0.0), 0.01f); assertTrue(t.toScreenY(0.0) > t.toScreenY(FieldConfig.PRO.widthM)) }
@Test fun roundTrip() { assertEquals(37.5, t.toWorldX(t.toScreenX(37.5)), 1e-3); assertEquals(12.25, t.toWorldY(t.toScreenY(12.25)), 1e-3) }
@Test fun wholeWorldFits() { assertTrue(t.toScreenX(-6.0) >= 0f && t.toScreenX(109.728 + 6.0) <= 1280f)
    assertTrue(t.toScreenY(-6.0) <= 720f && t.toScreenY(48.768 + 6.0) >= 0f) }
@Test fun pickFindsBodyUnderCursor() { val s = ViewerState(); val ids = IntArray(64); s.world.bodyIds(ids)
    val b = s.world.body(BodyId(ids[0])); val x = b.x; val y = b.y; assertEquals(BodyId(ids[0]), s.pick(x, y)); assertNull(s.pick(-5.0, -5.0)) }
@Test fun pausedFrameDoesNotTickButStepDoes() { val s = ViewerState(); s.paused = true; s.frame(0.5); assertEquals(0L, s.world.tick); s.stepOnce(); assertEquals(1L, s.world.tick) }
@Test fun speedScalesFrameTime() { val s = ViewerState(); s.speed = 4.0; s.frame(1.0 / 60.0); assertEquals(4L, s.world.tick) }
```

- [ ] **Step 2:** Run `./gradlew :physics-debug:test`. Expected: FAIL.
- [ ] **Step 3:** `build.gradle.kts`: plugins `kotlin.jvm`, `kotlin.compose`, `compose.multiplatform`; `kotlin { jvmToolchain(21) }`; deps `implementation(project(":physics-scenarios"))`, `implementation(compose.desktop.currentOs)`, `implementation(compose.material3)`, JUnit as in core; `compose.desktop.application { mainClass = "io.github.itstrue54.gridironphysics.debug.MainKt" }`.
- [ ] **Step 4:** Implement `ViewTransform` and `ViewerState` (the default scenario is `Scenarios.drift`; `pick` returns the nearest body whose radius contains the point, iterating `bodyIds`).
- [ ] **Step 5:** Run `./gradlew :physics-debug:test`. Expected: PASS.
- [ ] **Step 6:** Implement painters and `Main.kt`:
  - **FieldPainter.** Runoff in dark green, field in green, end zones shaded. Yard lines every 5 yd (thicker every 10), hash marks every yard at `nearHashY`/`farHashY`, and yard numbers every 10. Draw sidelines, end lines and both goal lines in a distinct **sensor color** (amber, dashed) because they are sensors, not walls. Goal posts: a small static mark at each end line center.
  - **BodyPainter.** Each body is a circle at `lerp(prev, cur, alpha)`. Colors by tag: HOME blue, AWAY red, OFFICIAL striped white/black, PROP gray. A static body gets a gray outline only. Draw a heading tick at `angle`, a velocity vector line (`v · 0.25 s` long), and a yellow ring on the selected body.
  - **Main.** A `Window` titled "gridiron-physics viewer". A frame loop uses `withFrameNanos` and calls `state.frame(dtSeconds)`. A top bar holds Play/Pause, Step (enabled when paused), speed toggle 0.25× / 1× / 4×, a scenario dropdown (`Scenarios.all`), Reset, and a tick counter. Clicking the canvas runs `pick` on the world coordinates. A right-side inspect panel lists every `BodyView` field of the selected body to 4 decimals, plus `stateHash()` in hex.
- [ ] **Step 7:** Run `./gradlew :physics-debug:run`. Expected: the window opens. 24 bodies drift and slow near midfield, and the official walks the near sideline. Pause/Step/speed work, and clicking a player shows its fields. Take a screenshot for the user.
- [ ] **Step 8:** Commit: `M1: add Compose Desktop debug viewer`.

---

### Task 9: CI, milestone gate and publish

**Files:**
- Create: `.github/workflows/ci.yml`
- Modify: `README.md` (CI badge)

- [ ] **Step 1:** Write `ci.yml`: on push/PR to `main`; matrix `os: [ubuntu-latest, windows-latest, macos-latest]`; steps `actions/checkout@v4`, `actions/setup-java@v4` (temurin 21), `gradle/actions/setup-gradle@v4`, then `./gradlew build --stacktrace` (use `shell: bash` so `./gradlew` works on Windows). `chmod +x gradlew` is committed via `git update-index --chmod=+x gradlew`.
- [ ] **Step 2:** Run `./gradlew build` locally. Expected: `BUILD SUCCESSFUL`. All tests in all three modules pass.
- [ ] **Step 3: Milestone gate.** Ask the user to run `./gradlew :physics-debug:run` (or show them the screenshot) and confirm the Drift scenario looks right. **Stop here until they confirm.**
- [ ] **Step 4:** Commit: `M1: add CI workflow`.
- [ ] **Step 5:** Create the public repo and push: `gh repo create itstrue54/gridiron-physics --public --source . --push --description "Deterministic top-down 2D football physics engine in pure Kotlin"`. The user approved public hosting in the design.
- [ ] **Step 6:** Check the CI run with `gh run watch` once. Expected: all three OS jobs green. If a job fails, use superpowers:systematic-debugging and do not start M2.

---

## Self-review notes

- Spec coverage for M1 (design table): scaffold ✔ T1, CLAUDE.md ✔ T1, CI ✔ T9, Vec2/Vec3 ✔ T2, DMath ✔ T2, SplitMix64 ✔ T2, Yards ✔ T2, FieldConfig/WorldConfig ✔ T3, fixed step ✔ T4, advance + alpha ✔ T5, three body types with circles ✔ T4, damping ✔ T4, stateHash ✔ T6, viewer (field, hashes, sensors, bodies, velocity vectors, pause/step/speed/inspect) ✔ T8, determinism same-device ✔ T6, accumulator tests ✔ T5, Yards round trip ✔ T2. Replay command hook ✔ T4.
- Deferred on purpose: contact points, joints and the ball in the viewer arrive with M2/M5/M6. `Stance`, `setIntent` and `setAthlete` come in M3.
