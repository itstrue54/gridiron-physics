plugins {
    alias(libs.plugins.kotlin.jvm)
    alias(libs.plugins.kotlin.allopen)
    alias(libs.plugins.kotlinx.benchmark)
}

// Benchmarks and the performance gates. Desktop only; never shipped in a game.
//   ./gradlew :physics-bench:benchmark        JMH: microseconds per step of the 25-player scrimmage, plus a whole play
//   ./gradlew :physics-bench:smokeBenchmark   the same, short
//   ./gradlew :physics-bench:ticksPerSecond   headless ticks per second (engine only and with the game layer)
//   ./gradlew :physics-bench:test             gates: >= 20,000 ticks/s, 0 bytes allocated per step
kotlin {
    jvmToolchain(21)
}

// JMH subclasses @State classes, so they must be open.
allOpen {
    annotation("org.openjdk.jmh.annotations.State")
}

dependencies {
    implementation(project(":physics-scenarios"))
    implementation(libs.kotlinx.benchmark.runtime)

    testImplementation(platform(libs.junit.bom))
    testImplementation(kotlin("test"))
    testImplementation("org.junit.jupiter:junit-jupiter")
    testRuntimeOnly("org.junit.platform:junit-platform-launcher")
}

benchmark {
    targets {
        register("main")
    }
    configurations {
        named("main") {
            warmups = 5
            iterations = 10
            iterationTime = 1
            iterationTimeUnit = "s"
            mode = "avgt"
            outputTimeUnit = "us"
            // JMH's GC profiler adds gc.alloc.rate.norm (bytes per step).
            advanced("jvmProfiler", "gc")
        }
        register("smoke") {
            warmups = 2
            iterations = 3
            iterationTime = 500
            iterationTimeUnit = "ms"
            mode = "avgt"
            outputTimeUnit = "us"
        }
    }
}

tasks.test {
    useJUnitPlatform()
    maxHeapSize = "1g"
}

tasks.register<JavaExec>("ticksPerSecond") {
    group = "benchmark"
    description = "Prints headless ticks per second for the 25-player scrimmage and the random-play harness."
    classpath = sourceSets["main"].runtimeClasspath
    mainClass.set("io.github.itstrue54.gridironphysics.bench.TicksPerSecond")
}
