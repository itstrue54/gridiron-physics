# gridiron-physics — rules for Claude Code

Spec: docs/PHYSICS_SPEC.md. Read it before any physics work.
Design: docs/superpowers/specs/2026-11-07-gridiron-physics-design.md

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
