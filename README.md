# Welcome to Waddle.

**Waddle** is an integrated FTC simulation environment. This means that it runs directly in your FTC
codebase! Waddle allows you to simulate and test your robot, whether it hasn't even been built yet
or you're trying to test a change remotely.

# Installation

In your TeamCode `build.gradle`, insert the following lines:

```gradle

repositiories {
    // If you already have this maven link, you can skip adding this. If you're not sure, don't skip it.
    // Add the below 3 lines in addition to whatever else is in your repositories block.
    maven {
        url = "https://repo.dairy.foundation/releases/"
    }
}

dependencies {
    // Add the below 2 lines in addition to whatever else is in your dependencies block.
    implementation "com.necessaryevil.waddle:version"
    implementation ""org.psilynx:psikit:0.0.3" // You can use whichever version of PsiKit you prefer, but having it is highly recommended. If you aren't sure, copy what's here.
}
```

Then, in the top menu, press "File > Sync Project with Gradle Files".

That's it, you're good to go!

# Usage

## Setting up a Simulation

Create a `main` function using the default functionality for your language.

Java:

```java

public class SimulationUtility {

    public static void main(String[] args) {
        // simulation stuff goes here
    }

}

```

Kotlin:

```kotlin
fun main() {
    // simulation stuff goes here
}
```

There should be a green "play" arrow on the left of your `main` function. This is how you are
actually going to run the simulation.

We aren't quite ready to run a simulation, but we can set up the code to do so.
To run any given OpMode, use the following code:

Java:

```java
Simulation.Companion.addSimulation(new ExampleOpMode(), 30); // The first argument is the OpMode you wish to run, and the second argument is the duration.

Simulation.Companion.runNextSimulation();
```

Kotlin:

```kotlin
Simulation.addSimulation(
    ExampleOpMode(),
    30.0
) // The first argument is the OpMode you wish to run, and the second argument is the duration.

Simulation.runNextSimulation()
```

This almost certainly gives you a `NullPointerException`, with the issue being that `hardwareMap`
does not have the objects you need. This brings us to the next step of the setup.

## Setting up HardwareMap

To add `HardwareDevice`s to `HardwareMap`, use the `SimulatedHardware` class.

Here's a brief example usage:

```kotlin
SimulatedHardware.addMotor(
    "exampleMotor",
    SimulatedMotor.GOBILDA_435,
    SimulatedDcMotor.MotorType.GOBILDA_435
) // motors have special syntax

// link axon
val axon = SimulatedServo(0.0, 180.0, 30.0) // create servo
val axonFourthWire = SimulatedAnalogInput() // create analog input
axonFourthWire.voltageSupplier =
    { axon.getTruePosition() * 3.3 / 180.0 } // bind the sensor to the true servo position

// add hardware devices, this handles all required logic for you if using an existing simulation hardware device
SimulatedHardware.addHardwareDevice("exampleServo", axon)
SimulatedHardware.addHardwareDevice("exampleAnalogInput", axonFourthWire)
```

Further examples are in `src/test/com/necessaryevil/waddle/HardwareDemo.kt`.

Included `HardwareDevice`s:

`SimulatedDcMotor`, `SimulatedServo`, `SimulatedAnalogInput`, `SimulatedDigitalChannel`,
`SimulatedGoBildaPinpointDriver`, `SimulatedLynxModule`, `SimulatedVoltageSensor`, `SimulatedIMU`.

The `LynxModule`s and `VoltageSensor` are configured by default. You do not need to add them.

## Setting up Mechanisms

The last thing you have to do is set up the mechanisms on your robot, to visualize them and allow
your robot to use them. See `src/test/com/necessaryevil/waddle/MechanismDemo.kt` for how to do this.

Currently supported mechanisms:

`MecanumDrivetrain` (special), `LinearExtension`, custom mechanisms.

Further elaboration on custom mechanisms will come in a future update.