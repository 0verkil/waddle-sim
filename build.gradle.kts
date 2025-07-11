plugins {
    id("com.android.library") version libs.versions.android
    kotlin("android") version libs.versions.kotlin
}

group = "me.zharel.hermes"
version = "1.0-SNAPSHOT"

android {
    namespace = "com.necessaryevil.simulatedsdk"
    //noinspection GradleDependency
    compileSdk = 33

    defaultConfig {
        minSdk = 24

        testInstrumentationRunner = "android.support.test.runner.AndroidJUnitRunner"
        consumerProguardFiles("consumer-rules.pro")
    }

    compileOptions {
        sourceCompatibility = JavaVersion.VERSION_1_8
        targetCompatibility = JavaVersion.VERSION_1_8
    }

    kotlinOptions {
        jvmTarget = "1.8"
        freeCompilerArgs += ("-Xjvm-default=all")
    }

    testOptions {
        unitTests {
            isReturnDefaultValues = true
            all {
                it.useJUnitPlatform()
            }
        }
    }
}

repositories {
    google()
    mavenCentral()
    maven("https://repo.dairy.foundation/releases/")
}

dependencies {
    implementation(libs.bundles.ftcsdk)
    implementation(libs.psikit)
    implementation(libs.ejml)

    testImplementation(libs.kotlin.reflect)
    testImplementation(libs.bundles.kotest)
}