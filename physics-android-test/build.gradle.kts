plugins {
    alias(libs.plugins.android.library)
}

// Runs physics-core's determinism replays as instrumented tests on an Android device or emulator:
//   ./gradlew :physics-android-test:connectedDebugAndroidTest
// The module has no code of its own; it only exists to put the shared DeterminismSuite on ART.
android {
    namespace = "io.github.itstrue54.gridironphysics.androidtest"
    compileSdk = 36

    defaultConfig {
        minSdk = 26
        testInstrumentationRunner = "androidx.test.runner.AndroidJUnitRunner"
    }

    compileOptions {
        sourceCompatibility = JavaVersion.VERSION_11
        targetCompatibility = JavaVersion.VERSION_11
    }

    lint {
        // Nothing ships from this module.
        checkReleaseBuilds = false
        abortOnError = false
    }
}

dependencies {
    androidTestImplementation(project(":physics-scenarios"))
    androidTestImplementation(libs.androidx.test.runner)
    androidTestImplementation(libs.androidx.test.junit)
}
