package com.necessaryevil.simulatedsdk.hardware

import com.necessaryevil.simulatedsdk.SimulationObject
import org.ejml.simple.SimpleMatrix
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.psilynx.psikit.Logger
import org.psilynx.psikit.RLOGServer
import org.psilynx.psikit.mechanism.LoggedMechanism2d
import org.psilynx.psikit.wpi.Color8Bit
import org.psilynx.psikit.wpi.Pose2d
import org.psilynx.psikit.wpi.Rotation2d
import java.lang.Thread.sleep
import java.util.function.DoubleSupplier
import java.util.function.Supplier
import kotlin.math.PI
import kotlin.math.abs
import kotlin.math.cos
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
class SimulatedMotor(val internalMoi: Double, val kFriction: Double, val kMotor: Double, val resistance: Double, val inductance: Double): SimulationObject {

    /**
     * State vector: [angular position, angular velocity, current].
     */
    var state = SimpleMatrix(3, 1)

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
    val load get() = loadInputs.fold(0.0) {acc, x -> x.asDouble + acc}

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
    val moi get() = moiInputs.fold(internalMoi) {acc, x -> x.asDouble + acc}

    val rpm: Double get() = outputMatrix.mult(state).get(1, 0)

    /**
     * In radians.
     */
    val angle: Double get() = outputMatrix.mult(state).get(0, 0)

    /**
     * In radians.
     */
    var deltaAngle: Double = 0.0

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
        state = state.plus(stateMatrix.mult(state).plus(inputMatrix.mult(v)).plus(disturbanceMatrix.mult(t)).scale(dt))
        deltaAngle = angle - prevAngle
    }

    fun addLoad(load: DoubleSupplier) {
        loadInputs.add(load)
    }

    fun addMoi(moi: DoubleSupplier) {
        moiInputs.add(moi)
    }

    companion object {
        val GOBILDA_435 get() = SimulatedMotor(0.001, 0.000109, 0.256, 1.30434782609, 0.001)
        val GOBILDA_1150 get() = SimulatedMotor(0.005, 0.000179702360396, 0.0969370938832, 1.30434782609, 0.01)
    }

}

fun main() {
    var timeElapsed = 0.0
    Logger.setTimeSource { timeElapsed }


    val pivot0 = SimulatedMotor.GOBILDA_435
    val pivot1 = SimulatedMotor.GOBILDA_435
    val slide0 = SimulatedMotor.GOBILDA_435

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
    val test = LoggedMechanism2d(10.0, 10.0)
    val testroot = test.getRoot("root", 5.0, 0.1)
    val motors = testroot.append(PhysicsLigament("motor", 0.01, 0.05))
    val slides = testroot.append(PhysicsLigament("slide", 0.01, 0.05))
    val pivot = testroot.append(PhysicsLigament("hey!", 0.5, 0.5, 45.0, 3.0, Color8Bit(0, 255, 0)))
    val test3 = pivot.simAppend(PhysicsLigament("hey2!", 0.2, 0.1, 0.0, 3.0, Color8Bit(0, 0, 255)))

    // drivetrain sim
    val leftFront = MecanumWheel(
        "leftFront",
        0.1,
        0.1,
        0.1,
        reversed = true
    )

    val leftBack = MecanumWheel(
        "leftBack",
        0.1,
        -0.1,
        0.1,
        reversed = false
    )

    val rightBack = MecanumWheel(
        "rightBack",
        0.1,
        -0.1,
        -0.1,
        reversed = true
    )

    val rightFront = MecanumWheel(
        "rightFront",
        0.1,
        0.1,
        -0.1,
        reversed = false
    )

    val chassis = Chassis(
        10.0,
        leftFront,
        leftBack,
        rightFront,
        rightBack,
        frontalArea=0.1161288,
        trackwidth=0.2
    )

    val lf = SimulatedMotor.GOBILDA_435
    val lb = SimulatedMotor.GOBILDA_435
    val rf = SimulatedMotor.GOBILDA_435
    val rb = SimulatedMotor.GOBILDA_435

    lf.power = 1.0
    lb.power = 0.6
    rf.power = 0.7
    rb.power = 0.3

    val asdf = edu.wpi.first.math.geometry.Pose2d()
    println(asdf)

    chassis.pose = Pose2d(0.24, 1.8288, Rotation2d())

    leftFront.constrainAngleByMotor(lf)
    leftBack.constrainAngleByMotor(lb)
    rightFront.constrainAngleByMotor(rf)
    rightBack.constrainAngleByMotor(rb)

    motors.angleSupplier = Supplier<Double> { motors.angle + Math.toDegrees(pivot0.deltaAngle) }
    slides.angleSupplier = Supplier<Double> { (slides.angle + Math.toDegrees(slide0.deltaAngle)).coerceIn(0.0, 1440.0) }
    slides.constrainLengthByConstant(0.05)
    pivot.constrainAngleByDeltaAngle(motors, 45.0, 0.0, 180.0)
    pivot.constrainLengthByDeltaAngle(slides, 0.5, 0.5, 2.0)
    test3.constrainLengthByConstant(0.1)

    slide0.power = -1.0

    pivot0.addLoad { pivot.angularLoad / 2.0 }
    pivot1.addLoad { pivot.angularLoad / 2.0 }

    slide0.addLoad { pivot.linearForce / 0.05 }

    while (true) {
    //while ((abs(AngleUnit.normalizeRadians(pivot0.angle - targetAngle)) > 0.01 || abs(pivot0.rpm) > 0.5)) {
        Logger.periodicBeforeUser()

        //pivot0.update(0.001)
        //pivot1.update(0.001)
        //slide0.update(0.001)
        lf.update(0.001)
        lb.update(0.001)
        rf.update(0.001)
        rb.update(0.001)

        Logger.recordOutput("Pivot/Arm position (rad)", AngleUnit.normalizeRadians(pivot0.angle))
        Logger.recordOutput("Pivot/Arm velocity (rpm)", pivot0.rpm)
        Logger.recordOutput("Pivot/Current (amps)", pivot0.state.get(2, 0))
        Logger.recordOutput("Pivot/Target angle (rad)", targetAngle)
        Logger.recordOutput("Pivot/Error (rad)", AngleUnit.normalizeRadians(pivot0.angle - targetAngle))
        Logger.recordOutput("Drivetrain/Robot pose", chassis.pose)
        Logger.recordOutput("Pivot/Load (Nm)", pivot.angularLoad)
        Logger.recordOutput("Pivot/Center of Mass", pivot.centerOfMass)
        Logger.recordOutput("test", test)
        Logger.recordOutput("Slides/Slide length", pivot.lengthSupplier.get())
        Logger.recordOutput("Slides/Slide power", slide0.power)
        Logger.recordOutput("Slides/Slide position", slide0.angle)
        Logger.recordOutput("Drivetrain/LeftFront Speed", lf.rpm)
        Logger.recordOutput("Drivetrain/Left Front Motor Delta Location", lf.deltaAngle)
        Logger.recordOutput("Drivetrain/Left Front Motor Location", lf.angle)
        Logger.recordOutput("Drivetrain/Left Front Motor Load", lf.load)
        Logger.recordOutput("Drivetrain/Left Front Wheel Torque", leftFront.angularLoad)
        Logger.recordOutput("Drivetrain/LF MOI", lf.moi)
        Logger.recordOutput("Drivetrain/lfw moi", leftFront.moi)
        //println(AngleUnit.normalizeRadians(pivot0.angle))

        /*motors.update(0.001)
        slides.update(0.001)
        pivot.update(0.001)
        test3.update(0.001)*/
        chassis.update(0.001)

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

        }
        if (counter % 5000 == 0) {
            targetAngle = if (targetAngle < PI / 2.0) PI / 2.0 else PI / 4.0
            slide0.power *= -1.0
        }

        if (counter == 2000) {
            lf.power = 0.0
            lb.power = 0.0
            rf.power = 0.0
            rb.power = 0.0
        }

        counter += 1

        sleep(2)
        timeElapsed += 0.001

        Logger.periodicAfterUser(0.0, 0.0)
    }

    Logger.end()
}