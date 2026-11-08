plugins {
    alias(libs.plugins.kotlin.jvm)
    alias(libs.plugins.kotlin.compose)
    alias(libs.plugins.compose.multiplatform)
}

// Desktop debug viewer. Development tool only; never shipped in a game.
kotlin {
    jvmToolchain(21)
}

dependencies {
    implementation(project(":physics-scenarios"))
    implementation(compose.desktop.currentOs)
    implementation(compose.material3)

    testImplementation(platform(libs.junit.bom))
    testImplementation(kotlin("test"))
    testImplementation("org.junit.jupiter:junit-jupiter")
    testRuntimeOnly("org.junit.platform:junit-platform-launcher")
}

tasks.test {
    useJUnitPlatform()
}

compose.desktop {
    application {
        mainClass = "io.github.itstrue54.gridironphysics.debug.MainKt"
    }
}

// Headless PNG of the viewer: ./gradlew :physics-debug:snapshot -Pout=build/viewer.png -Pseconds=1.5
tasks.register<JavaExec>("snapshot") {
    group = "application"
    description = "Renders the viewer headlessly to a PNG."
    classpath = sourceSets["main"].runtimeClasspath
    mainClass.set("io.github.itstrue54.gridironphysics.debug.SnapshotKt")
    args(
        (project.findProperty("out") as String?) ?: layout.buildDirectory.file("viewer.png").get().asFile.path,
        (project.findProperty("seconds") as String?) ?: "1.5",
        (project.findProperty("scenario") as String?) ?: "Head-on tackle",
        (project.findProperty("colliders") as String?) ?: "false",
    )
}
