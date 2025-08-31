package org.necessaryevil.waddle

import org.necessaryevil.waddle.ftc.SimulatableLinearOpMode
import org.necessaryevil.waddle.ftc.SimulatedHardware
import org.necessaryevil.waddle.ftc.Simulation
import org.necessaryevil.waddle.ftc.hardware.SimulatedAnalogInput
import org.necessaryevil.waddle.ftc.hardware.SimulatedDcMotor
import org.necessaryevil.waddle.ftc.hardware.SimulatedDigitalChannel
import org.necessaryevil.waddle.ftc.hardware.SimulatedGoBildaPinpointDriver
import org.necessaryevil.waddle.ftc.hardware.SimulatedServo
import org.necessaryevil.waddle.ftc.hardware.asPsikitPose2d
import org.necessaryevil.waddle.physics.chassis.MecanumDrivetrain
import org.necessaryevil.waddle.physics.common.SimulatedMotor
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver
import com.qualcomm.robotcore.hardware.AnalogInput
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DigitalChannel
import com.qualcomm.robotcore.hardware.Servo
import io.kotest.core.spec.style.FunSpec
import org.firstinspires.ftc.robotcore.external.JavaUtil.isPrime
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.psilynx.psikit.Logger
import kotlin.math.cos
import kotlin.math.round
import kotlin.math.sin

class HardwareDemo : SimulatableLinearOpMode() {

    val exampleMotor: DcMotor by lazy { hardwareMap.get(DcMotor::class.java, "exampleMotor") } // we can use normal hardwareMap syntax
    val exampleServo: Servo by lazy { hardwareMap.servo.get("exampleServo") } // and also this syntax!

    val exampleAnalogInput: AnalogInput by lazy { hardwareMap.get(AnalogInput::class.java, "exampleAnalogInput") }
    val exampleDigitalChannel: DigitalChannel by lazy { hardwareMap.get(DigitalChannel::class.java, "exampleDigitalChannel") }

    val pinpoint: GoBildaPinpointDriver by lazy { hardwareMap.get(GoBildaPinpointDriver::class.java, "pinpoint") } // this is the only i2c device supported out-of-the-box

    override fun runOpMode() {

        // instantiate hardware objects
        exampleMotor
        exampleServo

        exampleAnalogInput
        exampleDigitalChannel

        pinpoint

        waitForStart()

        while (opModeIsActive()) {
            Logger.periodicBeforeUser()

            // do stuff
            exampleMotor.power = sin(runtime)
            exampleServo.position = cos(runtime)
            pinpoint.update()

            // simulation can be tracked through advantagescope via psikit. this also works for your actual robot!
            Logger.recordOutput("Motor Position", exampleMotor.currentPosition)
            Logger.recordOutput("Servo Position", exampleServo.position)

            // in this simulation, the AnalogInput is the 4th wire coming out of an axon (the exampleServo). See how it tracks!
            Logger.recordOutput("Analog Input Voltage", exampleAnalogInput.voltage)
            Logger.recordOutput("Servo True Position", exampleAnalogInput.voltage * 180.0 / 3.3) // the axon (ostensibly) has a 180 degree range

            // the digital channel is tracking... something. not sure what.
            Logger.recordOutput("Digital Channel", exampleDigitalChannel.state)

            // aaand pinpoint. both position and velocity reads work.
            Logger.recordOutput("Robot Pose", pinpoint.position.asPsikitPose2d()) // utility function for pose2d conversion
            Logger.recordOutput("Robot X Velocity", pinpoint.getVelX(DistanceUnit.INCH))

            Logger.periodicAfterUser(0.0, 0.0)

            // VERY IMPORTANT! you must include this if you are using a LinearOpMode.
            // OpModes do not have to include this line.
            // This will not affect non-simulation performance.
            Simulation.update()

        }

    }

}

class HardwareSimulator : FunSpec( {

    test("hardwareDemo") {
        // add a motor!
        SimulatedHardware.addMotor("exampleMotor", SimulatedMotor.GOBILDA_435, SimulatedDcMotor.MotorType.GOBILDA_435)

        // link axon
        val axon = SimulatedServo(0.0, 180.0, 30.0) // create servo
        val axonFourthWire = SimulatedAnalogInput() // create analog input
        axonFourthWire.voltageSupplier = { axon.getTruePosition() * 3.3 / 180.0 } // bind the sensor to the true servo position

        // add hardware devices, this handles all required logic for you if using an existing simulation hardware device
        SimulatedHardware.addHardwareDevice("exampleServo", axon)
        SimulatedHardware.addHardwareDevice("exampleAnalogInput", axonFourthWire)

        // now we see that the digital channel is actually checking if the axon's position is prime.
        val isPrime = SimulatedDigitalChannel()
        isPrime.stateSupplier = { isPrime(round(axon.getTruePosition())) }

        SimulatedHardware.addHardwareDevice("exampleDigitalChannel", isPrime)

        // huh, we also have a drivetrain here! that's how the pinpoint was moving.
        val drive = MecanumDrivetrain(
            10.0,
            0.08,
            0.048,
            0.35,
            0.35,
            SimulatedMotor.GOBILDA_435
        )

        // and here's the pinpoint!
        SimulatedHardware.addHardwareDevice("pinpoint", SimulatedGoBildaPinpointDriver(drive))

        // woah, that's why the drive is moving!
        drive.drive(0.7, 0.3, 0.3)
        // since the drive is not a hardware device, we instead add it as a simulation object.
        SimulatedHardware.addSimulationObject(drive)

        // and we're good to go!
        Simulation.addSimulation(HardwareDemo(), 120.0)

        // we can also add in a function here that runs in the opmode loop, so we can access our simulation objects
        // (for instance, if you wanted to see actual motor position versus tracked position in ticks)
        Simulation.runNextSimulation(Runnable { Logger.recordOutput("Simulation/True Drive Pose", drive.pose) })

    }

})