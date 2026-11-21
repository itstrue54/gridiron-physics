package io.github.itstrue54.gridironphysics

/**
 * The game's hook into every tick, so a game that decides between ticks can still let [World.advance] run the
 * clock. Set it with [World.tickListener]; [World.step] calls it on every tick, however the tick was started.
 *
 * [beforeTick] runs before the tick's first phase, at [World.tick] = the tick about to run: commands issued there
 * (intents, stances, joints, the ball) count as issued before this tick, exactly as between two steps.
 * [afterTick] runs after the last phase, when [World.events] holds this tick's events. Neither may step the world.
 *
 * The engine still never decides anything: the listener is the game's code, called at fixed points. Because it runs
 * on ticks, not frames, a game that decides only here plays out the same whatever the frame times.
 */
interface TickListener {
    fun beforeTick(world: World) {}
    fun afterTick(world: World) {}
}
