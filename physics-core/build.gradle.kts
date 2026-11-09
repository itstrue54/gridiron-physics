import org.jetbrains.kotlin.gradle.dsl.JvmTarget

plugins {
    alias(libs.plugins.kotlin.jvm)
    `maven-publish`
}

// physics-core ships inside games. Kotlin stdlib + StrictMath only; Java 11 bytecode
// so Android API 26 consumers (and the existing Gridiron Dynasty build) can use it.
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
    testImplementation(platform(libs.junit.bom))
    testImplementation(kotlin("test"))
    testImplementation("org.junit.jupiter:junit-jupiter")
    testRuntimeOnly("org.junit.platform:junit-platform-launcher")
}

tasks.test {
    useJUnitPlatform()
}

// Approach B (optional): games normally use includeBuild("../gridiron-physics"). To publish instead:
//   ./gradlew :physics-core:publishToMavenLocal                     -> mavenLocal(), for trying the artifact
//   ./gradlew :physics-core:publish  (GITHUB_ACTOR, GITHUB_TOKEN)   -> GitHub Packages
// A game depends on io.github.itstrue54:gridiron-physics-core either way (README: includeBuild with a substitution);
// switching to the published artifact replaces the includeBuild block with the repository.
java {
    withSourcesJar()
}

publishing {
    publications {
        create<MavenPublication>("core") {
            artifactId = "gridiron-physics-core"
            from(components["java"])
            pom {
                name.set("gridiron-physics-core")
                description.set("Deterministic top-down football physics engine in pure Kotlin.")
                url.set("https://github.com/itstrue54/gridiron-physics")
            }
        }
    }
    repositories {
        maven {
            name = "GitHubPackages"
            url = uri("https://maven.pkg.github.com/itstrue54/gridiron-physics")
            credentials {
                username = System.getenv("GITHUB_ACTOR")
                password = System.getenv("GITHUB_TOKEN")
            }
        }
    }
}
