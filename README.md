# gridiron-physics

[![CI](https://github.com/itstrue54/gridiron-physics/actions/workflows/ci.yml/badge.svg)](https://github.com/itstrue54/gridiron-physics/actions/workflows/ci.yml)

A deterministic, top-down 2D rigid-body engine in pure Kotlin, tuned for 22 players and one ball on a
regulation football field. The engine owns motion, contact and ball flight; the game owns ratings, AI,
play logic and rules. Same seed + same inputs = the same result on Android, Windows, macOS and Linux.

- Physics spec: [`docs/PHYSICS_SPEC.md`](docs/PHYSICS_SPEC.md)
- Design: [`docs/superpowers/specs/2026-11-07-gridiron-physics-design.md`](docs/superpowers/specs/2026-11-07-gridiron-physics-design.md)

![Debug viewer, M1 Drift scenario](docs/images/m1-viewer.png)

## Modules

| Module | Contents | Ships in games? |
|---|---|---|
| `physics-core` | The engine (Kotlin stdlib + `StrictMath` only, Java 11 bytecode) | Yes |
| `physics-scenarios` | Scenario scripts, the 1,000-play determinism harness and its golden data, shared by tests and the viewer | No |
| `physics-debug` | Compose Desktop debug viewer | No |
| `physics-bench` | Benchmarks (M8) | No |
| `physics-android-test` | Android instrumented determinism tests (built only where an Android SDK is installed) | No |

## Building

```bash
./gradlew build                  # everything
./gradlew :physics-core:test     # engine tests
./gradlew :physics-debug:run     # debug viewer
./gradlew :physics-debug:run -Preplay=path/to/play.gpr          # open a replay in the viewer
./gradlew :physics-android-test:connectedDebugAndroidTest        # determinism on a device or emulator
./gradlew :physics-scenarios:generateGolden                      # regenerate golden digests (only when behaviour should change)
```

## Snapshots and replays

`World.snapshot()` captures the whole simulation state (bodies, joints, contacts with warm-start impulses, the ball,
events, the rng) as bytes; `restore()` puts any world with the same `WorldConfig` and `FieldConfig` back in that state,
and it steps on identically. A `ReplayRecorder` attached to a world records every command and every tick's
`stateHash()` without allocating; `Replay.encode()` gives the `.gpr` bytes (little-endian, versioned) and
`ReplayPlayer` replays them, reporting the first tick whose hash does not match. File I/O is the caller's job.

## Using it from a game

Clone this repo next to your game, then in the game's `settings.gradle.kts`:

```kotlin
includeBuild("../gridiron-physics")
```

and in the module that uses it:

```kotlin
dependencies {
    implementation("io.github.itstrue54:physics-core:0.1.0")
}
```
