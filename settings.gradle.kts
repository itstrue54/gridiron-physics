pluginManagement {
    repositories {
        gradlePluginPortal()
        mavenCentral()
        google()
    }
}
plugins {
    id("org.gradle.toolchains.foojay-resolver-convention") version "1.0.0"
}
dependencyResolutionManagement {
    repositoriesMode.set(RepositoriesMode.FAIL_ON_PROJECT_REPOS)
    repositories {
        mavenCentral()
        google()
    }
}

rootProject.name = "gridiron-physics"
include(":physics-core")
include(":physics-scenarios")
include(":physics-debug")
include(":physics-bench")

// Android instrumented determinism tests: only where an Android SDK is installed (ANDROID_HOME, ANDROID_SDK_ROOT or
// sdk.dir in local.properties), so the desktop modules build anywhere. CI's emulator job runs it.
val localProperties = file("local.properties")
val hasAndroidSdk = System.getenv("ANDROID_HOME") != null || System.getenv("ANDROID_SDK_ROOT") != null ||
    (localProperties.exists() && localProperties.readText().contains("sdk.dir"))
if (hasAndroidSdk) include(":physics-android-test")
