package com.necessaryevil.waddle.physics.common

import com.necessaryevil.waddle.UsbHidEnumerationExample
import com.necessaryevil.waddle.ftc.XboxOneControllerInputReport.Companion.fromHidData
import com.necessaryevil.waddle.physics.chassis.MecanumDrivetrain
import org.ejml.simple.SimpleMatrix
import org.hid4java.HidManager
import org.hid4java.HidServicesSpecification
import org.psilynx.psikit.Logger
import org.psilynx.psikit.RLOGServer
import org.psilynx.psikit.mechanism.LoggedMechanism2d
import org.psilynx.psikit.wpi.Color8Bit
import org.psilynx.psikit.wpi.Pose2d
import org.psilynx.psikit.wpi.Rotation2d
import org.psilynx.psikit.wpi.Translation2d
import java.lang.Thread.sleep
import java.util.function.DoubleSupplier
import java.util.function.Supplier
import kotlin.math.PI
import kotlin.math.abs
import kotlin.math.cos
import kotlin.math.max
import kotlin.math.min
import kotlin.math.pow
import kotlin.math.sign

/**
 * Simulated motor class. Control equations: https://ctms.engin.umich.edu/CTMS/index.php?example=MotorSpeed&section=SystemModeling
 *
 * @param internalMoi: Moment of inertia of the motor, in kgm^2.
 * @param kFriction: Viscous friction constant, in Nms.
 * @param kMotor: Represents both the electromotive force constant (V/rad/s) and the torque constant (Nm/Amp).
 * @param resistance: Electrical resistance, in ohms.
 * @param inductance: Electrical inductance, in H (Henry).
 */
class SimulatedMotor(
    val internalMoi: Double,
    val kFriction: Double,
    val kMotor: Double,
    val resistance: Double,
    val inductance: Double
) : SimulationObject {

    /**
     * State vector: [angular position (rad), angular velocity (rad/s, current].
     */
    var state = SimpleMatrix(3, 1)

    /**
     * Minimum angle of the motor. Motor should stall if it hits here, but it's not modelled.
     */
    var minAngle = Double.MIN_VALUE

    /**
     * Maximum angle of the motor. Motor should stall if it hits here, but it's not modelled.
     */
    var maxAngle = Double.MAX_VALUE

    /**
     * State matrix, aka A.
     */
    val stateMatrix = SimpleMatrix(3, 3)

    /**
     * Input matrix, aka B. Acts on a 1x1 "matrix" containing voltage.
     */
    val inputMatrix = SimpleMatrix(doubleArrayOf(0.0, 0.0, 1 / inductance))

    /**
     * Output matrix, aka C.
     */
    val outputMatrix = SimpleMatrix(2, 3)

    // TODO: replace with some voltage reader from the "hub"
    val maxVoltage get() = 12.5

    /**
     * The input to the motor, the only user commands that can be issued. Range: 0-1.
     */
    var power = 0.0

    val disturbanceMatrix = SimpleMatrix(doubleArrayOf(0.0, -1.0 / internalMoi, 0.0))

    /**
     * The torque resisting the motor. In units of Nm.
     */
    val load get() = loadInputs.fold(0.0) { acc, x -> x.asDouble + acc }

    /**
     * Dynamic load modeling. In units of Nm.
     */
    val loadInputs: ArrayList<DoubleSupplier> = ArrayList()

    /**
     * Dynamic moment of inertia modeling. In units of kgm^2.
     */
    val moiInputs: ArrayList<DoubleSupplier> = ArrayList()

    /**
     * True moi of the motor.
     */
    val moi get() = moiInputs.fold(internalMoi) { acc, x -> x.asDouble + acc }

    val rpm: Double get() = outputMatrix.mult(state).get(1, 0)

    /**
     * In radians.
     */
    val angle: Double get() = outputMatrix.mult(state).get(0, 0)

    /**
     * In radians.
     */
    var deltaAngle: Double = 0.0

    // New: Actual torque the motor outputs (after overcoming its own friction and back-EMF)
    // This is the torque applied to the external load (e.g., the wheel/robot inertia)
    val actualOutputTorque: Double
        get() {
            // in gemini we trust
            // Torque from current - Back-EMF torque - Friction torque (that motor has to overcome)
            val backEmfTorque = kMotor * state.get(1, 0) // kMotor * angular_velocity
            val frictionTorque = kFriction * state.get(1, 0) // kFriction * angular_velocity
            val electricalTorque = kMotor * state.get(2, 0) // kMotor * current

            // The output torque is what's left after overcoming internal friction
            // This is the torque the motor produces to act on its external environment
            return electricalTorque - frictionTorque
        }

    init {
        stateMatrix.set(0, 1, 1.0)
        stateMatrix.set(1, 1, -kFriction / internalMoi)
        stateMatrix.set(1, 2, kMotor / internalMoi)
        stateMatrix.set(2, 1, -kMotor / inductance)
        stateMatrix.set(2, 2, -resistance / inductance)
        outputMatrix.set(1, 1, 30.0 / PI)
        outputMatrix.set(0, 0, 1.0)
    }

    override fun update(dt: Double) {

        // update model for new moi
        stateMatrix.set(1, 1, -kFriction / moi)
        stateMatrix.set(1, 2, kMotor / moi)
        disturbanceMatrix.set(1, 0, -1.0 / moi)

        val prevAngle = angle

        // clamp power
        if (abs(power) > 1.0) {
            power = sign(power) * 1.0
        }

        // convert voltage and load to matrices
        val v = SimpleMatrix(doubleArrayOf(maxVoltage * power))
        val t = SimpleMatrix(doubleArrayOf(load))

        // state update by dt
        state = state.plus(
            stateMatrix.mult(state).plus(inputMatrix.mult(v)).plus(disturbanceMatrix.mult(t))
                .scale(dt)
        )
        deltaAngle = angle - prevAngle

        // compensate for state limits
        if (state.get(2) < 0) {
            state.set(2, 0.0)
        }

        state.set(0, state.get(0).coerceIn(minAngle, maxAngle))
    }

    fun addLoad(load: DoubleSupplier) {
        loadInputs.add(load)
    }

    fun addMoi(moi: DoubleSupplier) {
        moiInputs.add(moi)
    }

    companion object {
        val GOBILDA_435 get() = SimulatedMotor(0.025, 0.000109, 0.256, 1.30434782609, 0.01)
        val GOBILDA_1150
            get() = SimulatedMotor(
                0.005,
                0.000179702360396,
                0.0969370938832,
                1.30434782609,
                0.01
            )
    }

}

fun main() {
    var timeElapsed = 0.0


    val pivot0 = SimulatedMotor.GOBILDA_435
    val pivot1 = SimulatedMotor.GOBILDA_435
    val slide0 = SimulatedMotor.GOBILDA_435

    val testMotor = SimulatedMotor.GOBILDA_435
    testMotor.addLoad { 0.0 }

    println(pivot0.stateMatrix.eig().eigenvalues)

    // start at free speed
    //testMotor.state.set(0, 0, 435.0 * 2.0 * PI)
    //testMotor.state.set(1, 0, 0.2)

    var counter: Int = 0
    val p = 0.2
    val g = 0.0
    var targetAngle = PI / 4.0

    var server = RLOGServer()
    Logger.addDataReceiver(server)
    Logger.recordMetadata("opMode name", "aKit test")
    Logger.start()
    Logger.periodicAfterUser(0.0, 0.0)
    Logger.start()
    Logger.setTimeSource { timeElapsed }
    val test = LoggedMechanism2d(10.0, 10.0)
    val testroot = test.getRoot("root", 5.0, 0.1)
    val motors = testroot.append(PhysicsLigament("motor", 0.01, 0.05))
    val slides = testroot.append(PhysicsLigament("slide", 0.01, 0.05))
    val pivot = testroot.append(PhysicsLigament("hey!", 0.5, 0.5, 45.0, 3.0, Color8Bit(0, 255, 0)))
    val test3 = pivot.simAppend(PhysicsLigament("hey2!", 0.2, 0.1, 0.0, 3.0, Color8Bit(0, 0, 255)))

    // drivetrain sim

    val drive = MecanumDrivetrain(
        10.0,
        1.0 * 10.0 * (0.08) / 10.0, // choreo formula but i made it higher to account for outer moi
        0.048,
        0.35, // Better trackwidth  
        0.35, // Better wheelbase
        SimulatedMotor.GOBILDA_435,
        1.0,
        0.6,
        0.15,
        0.3
    )

    val lf = SimulatedMotor.GOBILDA_435
    val lb = SimulatedMotor.GOBILDA_435
    val rf = SimulatedMotor.GOBILDA_435
    val rb = SimulatedMotor.GOBILDA_435

    drive.connectMotors(lf, lb, rf, rb)

    lf.power = 1.0
    lb.power = 0.7
    rf.power = 0.3
    rb.power = 0.5
    testMotor.power = 1.0

    drive.pose = Pose2d(0.24, 1.8288, Rotation2d())

    motors.angleSupplier = Supplier<Double> { motors.angle + Math.toDegrees(pivot0.deltaAngle) }
    slides.angleSupplier = Supplier<Double> {
        (slides.angle + Math.toDegrees(slide0.deltaAngle)).coerceIn(
            0.0,
            1440.0
        )
    }
    slides.constrainLengthByConstant(0.05)
    pivot.constrainAngleByDeltaAngle(motors, 45.0, 0.0, 180.0)
    pivot.constrainLengthByDeltaAngle(slides, 0.5, 0.5, 2.0)
    test3.constrainLengthByConstant(0.1)

    slide0.power = -1.0

    pivot0.addLoad { pivot.angularLoad / 2.0 }
    pivot1.addLoad { pivot.angularLoad / 2.0 }

    slide0.addLoad { pivot.linearForce / 0.05 }

    // set up gamepad

    // Configure to use custom specification
    val hidServicesSpecification = HidServicesSpecification()

    // Use the v0.7.0 manual start feature to get immediate attach events
    hidServicesSpecification.setAutoStart(false)


    // Get HID services using custom specification
    val hidServices = HidManager.getHidServices(hidServicesSpecification)
    hidServices.addHidServicesListener(UsbHidEnumerationExample())
    hidServices.start()

    // get gamepad
    var device = hidServices.attachedHidDevices[0]
    for (hidDevice in hidServices.attachedHidDevices) {
        if (hidDevice.product.startsWith("Xbox")) {
            device = hidDevice
        }
    }

    while (device.isClosed()) {
        device.open()
    }

    device.setNonBlocking(true)

    while (true) {

        val startTime = System.nanoTime()
        //while ((abs(AngleUnit.normalizeRadians(pivot0.angle - targetAngle)) > 0.01 || abs(pivot0.rpm) > 0.5)) {
        Logger.periodicBeforeUser()

        //pivot0.update(0.001)
        //pivot1.update(0.001)
        //slide0.update(0.001)

        // Logger.recordOutput("Pivot/Arm position (rad)", AngleUnit.normalizeRadians(pivot0.angle))
        Logger.recordOutput("Pivot/Arm velocity (rpm)", pivot0.rpm)
        Logger.recordOutput("Pivot/Current (amps)", pivot0.state.get(2, 0))
        Logger.recordOutput("Pivot/Target angle (rad)", targetAngle)
        /*Logger.recordOutput(
            "Pivot/Error (rad)",
            AngleUnit.normalizeRadians(pivot0.angle - targetAngle)
        )*/
        Logger.recordOutput("Drivetrain/Robot pose", drive.pose)
        Logger.recordOutput("Pivot/Load (Nm)", pivot.angularLoad)
        Logger.recordOutput("Pivot/Center of Mass", pivot.centerOfMass)
        Logger.recordOutput("test", test)
        Logger.recordOutput("Slides/Slide length", pivot.lengthSupplier.get())
        Logger.recordOutput("Slides/Slide power", slide0.power)
        Logger.recordOutput("Slides/Slide position", slide0.angle)
        Logger.recordOutput("Test/TestMotor Rpm", testMotor.rpm)
        /*Logger.recordOutput("Drivetrain/LeftFront Speed", lf.rpm)
        Logger.recordOutput("Drivetrain/Left Front Motor Delta Location", lf.deltaAngle)
        Logger.recordOutput("Drivetrain/Left Front Motor Location", lf.angle)
        Logger.recordOutput("Drivetrain/Left Front Motor Load", lf.load)
        Logger.recordOutput("Drivetrain/Left Front Wheel Torque", leftFront.angularLoad)
        Logger.recordOutput("Drivetrain/LF MOI", lf.moi)
        Logger.recordOutput("Drivetrain/lfw moi", leftFront.moi)*/
        Logger.recordOutput("Simulation/LogMillis", getDeltaMillis(startTime))


        val postLogTime = System.nanoTime()
        //println(AngleUnit.normalizeRadians(pivot0.angle))

        /*motors.update(0.001)
        slides.update(0.001)
        pivot.update(0.001)
        test3.update(0.001)*/
        lf.update(DELTA_TIME)
        lb.update(DELTA_TIME)
        rf.update(DELTA_TIME)
        rb.update(DELTA_TIME)
        testMotor.update(DELTA_TIME)
        drive.update(DELTA_TIME)

        Logger.recordOutput("Simulation/PhysicsMillis", getDeltaMillis(postLogTime))

        if (counter % 10 == 0) {
            var error = (targetAngle - pivot.angle % (2.0 * PI))
            //println("initial error: $error")
            if (error < -PI) {
                error += 2.0 * PI
            } else if (error > PI) {
                error -= 2.0 * PI
            }
            val power = error * p + cos(pivot.angle) * g

            Logger.recordOutput("Power", power)
            pivot0.power = power
            pivot1.power = power

            val report = device.read(64, 1)
            val reportbyte = ByteArray(report.size)
            var i = 0
            for (a in report) {
                reportbyte[i] = a
                i++
            }

            val thing = fromHidData(reportbyte)

            if (thing != null) {

                val ix = stickCurve(-thing.lx.toDouble() / 32768.0 + 1.0)
                val iy = stickCurve(-thing.ly.toDouble() / 32768.0 + 1.0)
                val rx = thing.rx.toDouble() / 32768.0 - 1.0

                val transformed = Translation2d(ix, iy).rotateBy(drive.pose.rotation)

                val x = transformed.x
                val y = transformed.y

                Logger.recordOutput("Joystick/rawLeftX", thing.lx)
                Logger.recordOutput("Joystick/leftx", x)
                Logger.recordOutput("Joystick/lefty", y)
                Logger.recordOutput("Joystick/rightx", rx)

                // Use the drivetrain's drive function instead of setting powers directly
                drive.drive(x, y, rx)
                
                Logger.recordOutput("Joystick/Using drive function", true)

                testMotor.power = thing.lt.toDouble() / 1023.0 - thing.rt.toDouble() / 1023.0
                Logger.recordOutput("Joystick/Delta Input", thing.lt.toDouble() / 1023.0 - thing.rt.toDouble() / 1023.0)


            }

        }
        /*if (counter % 5000 == 0) {
            targetAngle = if (targetAngle < PI / 2.0) PI / 2.0 else PI / 4.0
            slide0.power *= -1.0
        }

        if (counter == 2000) {
            lf.power = 0.0
            lb.power = 0.0
            rf.power = 0.0
            rb.power = 0.0
        }*/

        counter += 1

        val remainingMillis = (DELTA_TIME * 1000.0) - ((System.nanoTime() - startTime).toDouble() / 1.0e6)

        if (remainingMillis > 0.0) {
            val remainingNanos = ((remainingMillis) * 1.0e6).toInt()

            //println(remainingNanos)
            //println(remainingMillis)
            sleep(0, remainingNanos)
        } else {
            lf.update(-remainingMillis / 1000.0)
            lb.update(-remainingMillis / 1000.0)
            rf.update(-remainingMillis / 1000.0)
            rb.update(-remainingMillis / 1000.0)
            testMotor.update(-remainingMillis / 1000.0)
            drive.update(-remainingMillis / 1000.0)
        }

        Logger.recordOutput("Simulation/ActualLoopMillis", (DELTA_TIME * 1000.0) - remainingMillis)
        timeElapsed += if (remainingMillis < 0) -remainingMillis / 1000.0 + DELTA_TIME else DELTA_TIME

        Logger.periodicAfterUser(0.0, 0.0)
    }

    Logger.end()
}

fun getDeltaMillis(startTime: Long): Double {
    return (System.nanoTime() - startTime).toDouble() / 1.0e6
}

fun stickCurve(input: Double): Double {
    return (input.pow(3) + input) / 2.0
}

const val DELTA_TIME = 0.001