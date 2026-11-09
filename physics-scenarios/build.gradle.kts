import org.jetbrains.kotlin.gradle.dsl.JvmTarget

plugins {
    alias(libs.plugins.kotlin.jvm)
}

// Scenario scripts shared by tests and the debug viewer. Never shipped in a game.
kotlin {
    jvmToolchain(21)
    compilerOptions {
        jvmTarget.set(JvmTarget.JVM_11)
        freeCompilerArgs.add("-Xjdk-release=11")
    }
}

tasks.withType<JavaCompile>().configureEach {
    options.release.set(11)
}

dependencies {
    api(project(":physics-core"))

    testImplementation(platform(libs.junit.bom))
    testImplementation(kotlin("test"))
    testImplementation("org.junit.jupiter:junit-jupiter")
    testRuntimeOnly("org.junit.platform:junit-platform-launcher")
}

tasks.test {
    useJUnitPlatform()
}

// Regenerates the cross-device golden data (digests of the 1,000 harness plays and a few .gpr replays).
// Run only when engine behaviour is meant to change, then commit src/main/resources/golden.
tasks.register<JavaExec>("generateGolden") {
    group = "verification"
    description = "Writes golden determinism digests and replays into src/main/resources/golden."
    classpath = sourceSets["main"].runtimeClasspath
    mainClass.set("io.github.itstrue54.gridironphysics.scenarios.GenerateGolden")
    args(layout.projectDirectory.dir("src/main/resources").asFile.path)
}

tasks.test {
    // The determinism harness plays 1,000 plays several times.
    maxHeapSize = "1g"
}

// The allocation gate again with escape analysis off. HotSpot's escape analysis can hide a temporary object (a boxed
// value, an iterator, a lambda) that Android's runtime would really allocate, so step() must allocate nothing even
// without it.
val allocationTestNoEscapeAnalysis by tasks.registering(Test::class) {
    group = "verification"
    description = "Runs StepAllocationTest with -XX:-DoEscapeAnalysis."
    testClassesDirs = sourceSets["test"].output.classesDirs
    classpath = sourceSets["test"].runtimeClasspath
    useJUnitPlatform()
    filter { includeTestsMatching("*.StepAllocationTest") }
    jvmArgs("-XX:-DoEscapeAnalysis")
    maxHeapSize = "1g"
}
tasks.check { dependsOn(allocationTestNoEscapeAnalysis) }
