package com.necessaryevil.waddle

import com.necessaryevil.waddle.ftc.SimulatableLinearOpMode
import com.necessaryevil.waddle.ftc.SimulatedHardware
import com.necessaryevil.waddle.ftc.Simulation
import com.necessaryevil.waddle.ftc.hardware.SimulatedDcMotor
import com.necessaryevil.waddle.ftc.hardware.SimulatedGoBildaPinpointDriver
import com.necessaryevil.waddle.ftc.hardware.SimulatedIMU
import com.necessaryevil.waddle.ftc.hardware.SimulatedServo
import com.necessaryevil.waddle.ftc.hardware.asPsikitPose2d
import com.necessaryevil.waddle.physics.chassis.MecanumDrivetrain
import com.necessaryevil.waddle.physics.common.PhysicsLigament
import com.necessaryevil.waddle.physics.common.SimulatedMotor
import com.necessaryevil.waddle.physics.common.SimulationLigament
import com.necessaryevil.waddle.physics.mechanism.ExtensionArm
import com.necessaryevil.waddle.physics.mechanism.LinearExtension
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver
import com.qualcomm.hardware.rev.Rev9AxisImuOrientationOnRobot
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.IMU
import com.qualcomm.robotcore.hardware.ImuOrientationOnRobot
import com.qualcomm.robotcore.hardware.Servo
import io.kotest.core.spec.style.FunSpec
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D
import org.psilynx.psikit.Logger
import org.psilynx.psikit.mechanism.LoggedMechanismLigament2d
import org.psilynx.psikit.wpi.Color8Bit
import org.psilynx.psikit.wpi.Pose2d
import org.psilynx.psikit.wpi.Rotation2d
import java.util.function.Supplier
import kotlin.math.abs
import kotlin.math.cos
import kotlin.math.sin

class FullRobotDemo : SimulatableLinearOpMode() {

    // ===== Hardware ======
    val leftFront: DcMotor by lazy { hardwareMap.dcMotor.get("leftFront") }
    val leftBack: DcMotor by lazy { hardwareMap.dcMotor.get("leftBack") }
    val rightFront: DcMotor by lazy { hardwareMap.dcMotor.get("rightFront") }
    val rightBack: DcMotor by lazy { hardwareMap.dcMotor.get("rightBack") }

    val liftLeft: DcMotor by lazy { hardwareMap.dcMotor.get("liftLeft") }
    val liftRight: DcMotor by lazy { hardwareMap.dcMotor.get("liftRight") }

    val extension: DcMotor by lazy { hardwareMap.dcMotor.get("extension") }

    val intake: DcMotor by lazy { hardwareMap.dcMotor.get("intake") }

    val outtakeClaw: Servo by lazy { hardwareMap.servo.get("outtakeClaw") }
    val outtakeArm: Servo by lazy { hardwareMap.servo.get("outtakeArm") }

    val intakePitch: Servo by lazy { hardwareMap.servo.get("intakePitch") }

    val imu: IMU by lazy { hardwareMap.get(IMU::class.java, "imu") }

    val pinpoint: GoBildaPinpointDriver by lazy { hardwareMap.get(GoBildaPinpointDriver::class.java, "pinpoint") }

    // ===== Utilities =====
    val axialPid: PID = PID(0.15, 0.0, 0.0) // Yes, I really did tune these for this demo. Thank me later.
    val coaxialPid: PID = PID(0.2, 0.0, 0.0)
    val rotationalPid: PID = PID(1.0, 0.0, 0.0)
    val liftPid: PID = PID(0.025, 0.0, 0.0)
    val extensionPID: PID = PID(0.025, 0.0, 0.0)

    val outtakeTransfer = 0.0
    val outtakeScore = 1.0

    val clawOpen = 0.0
    val clawClosed = 1.0

    val intakeUp = 0.7
    val intakeDown = 0.0

    // ===== PID targets =====
    var liftTarget = 2000.0
    var extensionTarget = 0.0
    var robotTarget = Pose2d(-55.0, 55.0, Rotation2d.fromDegrees(-45.0))

    // state
    enum class AutoState {
        DEPO_FIRST,
        INTAKE_SPIKE,
        DEPO_SECOND
    }
    var state = AutoState.DEPO_FIRST

    override fun runOpMode() {
        leftFront
        leftBack
        rightFront
        rightBack

        liftLeft
        liftRight

        extension

        intake

        outtakeClaw
        outtakeArm

        intakePitch

        imu.initialize(
            IMU.Parameters(
                RevHubOrientationOnRobot(
                    RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                    RevHubOrientationOnRobot.UsbFacingDirection.UP
                )
            )
        )

        pinpoint.initialize()
        pinpoint.recalibrateIMU()

        pinpoint.setPosX(-63.0, DistanceUnit.INCH)
        pinpoint.setPosY(37.0, DistanceUnit.INCH)

        waitForStart()

        while (opModeIsActive()) {
            Logger.periodicBeforeUser()

            driveToPoint(robotTarget)

            update1dPid(liftPid, liftTarget - liftLeft.currentPosition, liftLeft, liftRight)

            update1dPid(extensionPID, extensionTarget - extension.currentPosition, extension)

            when (state) {

                AutoState.DEPO_FIRST -> {
                    intakePitch.setPosition(intakeUp)

                    if (liftLeft.currentPosition > 1700.0) {
                        outtakeArm.setPosition(outtakeScore)
                    } else {
                        outtakeArm.setPosition(outtakeTransfer)
                    }

                    if (runtime > 2.0) {
                        state = AutoState.INTAKE_SPIKE
                    }
                }

                AutoState.INTAKE_SPIKE -> {

                    robotTarget = Pose2d(-50.0, 50.0, Rotation2d())
                    liftTarget = 0.0
                    extensionTarget = 500.0 + (runtime - 2.0) * 500.0

                    outtakeArm.setPosition(outtakeTransfer)
                    intakePitch.setPosition(intakeDown)

                    if (runtime > 3.5) {
                        state = AutoState.DEPO_SECOND
                    }

                }

                AutoState.DEPO_SECOND -> {
                    extensionTarget = 0.0
                    robotTarget = Pose2d(-55.0, 55.0, Rotation2d.fromDegrees(-45.0))

                    if (extension.currentPosition < 50.0) {
                        liftTarget = 2000.0
                    }

                    intakePitch.setPosition(intakeUp)

                    if (liftLeft.currentPosition > 1700.0) {
                        outtakeArm.setPosition(outtakeScore)
                    } else {
                        outtakeArm.setPosition(outtakeTransfer)
                    }
                }
            }

            pinpoint.update()

            Logger.recordOutput("Runtime", runtime)
            Logger.recordOutput("Lift Position", liftLeft.currentPosition)

            Logger.periodicAfterUser(0.0, 0.0)

            Simulation.update()
        }

    }

    /**
     * Field-centric mecanum drive function from gm0.
     * +x is right.
     * +y is forward.
     */
    fun drive(x: Double, y: Double, rx: Double) {

        val botHeading = -imu.robotYawPitchRollAngles.yaw
        Logger.recordOutput("Heading", botHeading)

        val rotX = x * cos(-botHeading) - y * sin(-botHeading)
        val rotY = x * sin(-botHeading) + y * cos(-botHeading)

        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio,
        // but only if at least one is out of the range [-1, 1]
        val denominator: Double = (abs(rotY) + abs(rotX) + abs(rx)).coerceAtLeast(1.0)
        val frontLeftPower = (rotY + rotX + rx) / denominator
        val backLeftPower = (rotY - rotX + rx) / denominator
        val frontRightPower = (rotY - rotX - rx) / denominator
        val backRightPower = (rotY + rotX - rx) / denominator

        leftFront.setPower(frontLeftPower)
        leftBack.setPower(backLeftPower)
        rightFront.setPower(frontRightPower)
        rightBack.setPower(backRightPower)
    }

    /**
     * PID to Point function.
     */
    fun driveToPoint(target: Pose2d) {
        var pose = pinpoint.position.asWpiPose2d()

        var axialError = target.translation - pose.translation
        var rotationalError = (target.rotation - pose.rotation).radians

        drive(
            coaxialPid.update(axialError.y),
            axialPid.update(axialError.x),
            -rotationalPid.update(rotationalError)
        )

        Logger.recordOutput("Drivetrain/Pose", pinpoint.position.asWpiPose2d())
        Logger.recordOutput("Drivetrain/Visualized Pose", pinpoint.position.asPsikitPose2d())

    }

    fun update1dPid(pid: PID, error: Double, vararg motors: DcMotor) {
       val power = pid.update(error)
        motors.forEach { it.setPower(power) }
    }

}

class FullRobotSimulator : FunSpec({

    test("Full Robot Demo") {

        // add drivetrain and attach motors
        val drive = SimulatedHardware.addSimulationObject(
            MecanumDrivetrain(
                12.0,
                0.02,
                0.096,
                0.3,
                0.3,
                SimulatedMotor.GOBILDA_435,
                1.0,
            )
        )

        drive.connectMotors(
            SimulatedHardware.addMotor(
                "leftFront", SimulatedMotor.GOBILDA_435
            ),
            SimulatedHardware.addMotor(
                "leftBack", SimulatedMotor.GOBILDA_435
            ),
            SimulatedHardware.addMotor(
                "rightBack", SimulatedMotor.GOBILDA_435
            ),
            SimulatedHardware.addMotor(
                "rightFront", SimulatedMotor.GOBILDA_435
            ),
        )

        drive.pose = Pose2d(-65.0 * 0.0254, 37.0 * 0.0254, Rotation2d())

        // add pinpoint
        SimulatedHardware.addHardwareDevice(
            "pinpoint",
            SimulatedGoBildaPinpointDriver(
                drive
            )
        )

        // add imu
        SimulatedHardware.addHardwareDevice(
            "imu",
            SimulatedIMU(
                drive
            )
        )

        // add vertical lift
        val lift = SimulatedHardware.addSimulationObject(
            LinearExtension(
                "Lift",
                -0.15, 1.0, 11.0, 45.0, 90.0, 25.0,
                SimulatedHardware.addMotor(
                    "liftLeft", SimulatedMotor.GOBILDA_435, SimulatedDcMotor.MotorType.GOBILDA_435
                ),
                SimulatedHardware.addMotor(
                    "liftRight", SimulatedMotor.GOBILDA_435
                ),
            )
        )

        // construct a custom deposit arm and link it with the servo
        val depoArm = SimulatedHardware.addSimulationObject(
            SimulationLigament(
                "Deposit Arm",
                0.2,
                -45.0,
                3.0,
                Color8Bit(150, 150, 230)
            )
        )

        depoArm.constrainAngleByServos(
            other=arrayOf(SimulatedHardware.addHardwareDevice("outtakeArm",
                SimulatedServo(
                    -135.0,
                    90.0,
                    540.0,
                )
            ))
        )

        depoArm.constrainLengthByConstant(0.2)

        // add the arm to the lift
        lift.append(
            depoArm
        )

        // create a generic claw servo that is not linked to anything
        SimulatedHardware.addHardwareDevice(
            "outtakeClaw",
            SimulatedServo(0.0, 90.0, 180.0)
        )

        // add horizontal extension
        val extension = SimulatedHardware.addSimulationObject(
            LinearExtension(
                "Extension",
                0.05, 1.0, 11.0, 30.0, 0.0, 25.0,
                SimulatedHardware.addMotor(
                    "extension", SimulatedMotor.GOBILDA_435, SimulatedDcMotor.MotorType.GOBILDA_435
                ),
            )
        )

        // create and link intake object
        val intake = SimulatedHardware.addSimulationObject(
            SimulationLigament(
                "Intake",
                0.025,
                45.0,
                5.0,
                Color8Bit(100, 255, 100)
            )
        )

        intake.constrainAngleByServos(
            other=arrayOf(SimulatedHardware.addHardwareDevice("intakePitch",
                SimulatedServo(
                    -45.0,
                    135.0,
                    360.0,
                )
            ))
        )

        intake.constrainLengthByConstant(0.05)

        extension.append(
            intake
        )

        // add unlinked intake motor object
        SimulatedHardware.addMotor(
            "intake", SimulatedMotor.GOBILDA_435
        )


        // add a linear extension with spool radius 25mm
        //SimulatedHardware.addSimulationObject(LinearExtension("Slides", -0.15, 1.0, 11.0, 40.0, 0.0, 25.0, leftSlide, rightSlide))

        Simulation.addSimulation(FullRobotDemo(), 120.0)

        Simulation.runNextSimulation()

    }

})