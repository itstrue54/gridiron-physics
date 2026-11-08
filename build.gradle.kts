plugins {
    alias(libs.plugins.kotlin.jvm) apply false
    alias(libs.plugins.kotlin.compose) apply false
    alias(libs.plugins.compose.multiplatform) apply false
}

allprojects {
    group = "io.github.itstrue54"
    version = "0.1.0"
}
