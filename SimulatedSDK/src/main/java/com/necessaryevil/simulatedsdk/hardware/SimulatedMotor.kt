package com.necessaryevil.simulatedsdk.hardware

import org.ejml.data.DMatrixRMaj
import org.ejml.simple.SimpleMatrix
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.psilynx.psikit.Logger
import org.psilynx.psikit.RLOGServer
import java.lang.Thread.sleep
import java.util.Arrays
import kotlin.math.PI
import kotlin.math.abs
import kotlin.math.cos

/**
 * Simulated motor class. Control equations: https://ctms.engin.umich.edu/CTMS/index.php?example=MotorSpeed&section=SystemModeling
 *
 * @param moi: Moment of inertia of the motor, in kgm^2.
 * @param kFriction: Viscous friction constant, in Nms.
 * @param kMotor: Represents both the electromotive force constant (V/rad/s) and the torque constant (Nm/Amp).
 * @param resistance: Electrical resistance, in ohms.
 * @param inductance: Electrical inductance, in H (Henry).
 */
class SimulatedMotor(val moi: Double, val kFriction: Double, val kMotor: Double, val resistance: Double, val inductance: Double) {

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

    val disturbanceMatrix = SimpleMatrix(doubleArrayOf(0.0, -1.0 / moi, 0.0))

    /**
     * The torque resisting the motor. In units of Nm.
     */
    var load = 0.0

    val rpm: Double get() = outputMatrix.mult(state).get(1, 0)

    /**
     * In radians.
     */
    val angle: Double get() = outputMatrix.mult(state).get(0, 0)

    init {
        stateMatrix.set(0, 1, 1.0)
        stateMatrix.set(1, 1, -kFriction / moi)
        stateMatrix.set(1, 2, kMotor / moi)
        stateMatrix.set(2, 1, -kMotor / inductance)
        stateMatrix.set(2, 2, -resistance / inductance)
        outputMatrix.set(1, 1, 30.0 / PI)
        outputMatrix.set(0, 0, 1.0)
    }

    fun update(dt: Double) {
        val v = SimpleMatrix(doubleArrayOf(maxVoltage * power))
        val t = SimpleMatrix(doubleArrayOf(load))
        state = state.plus(stateMatrix.mult(state).plus(inputMatrix.mult(v)).plus(disturbanceMatrix.mult(t)).scale(dt))
    }

    companion object {
        val GOBILDA_435 = SimulatedMotor(0.01, 0.000109, 0.256, 1.30434782609, 0.01)
        val GOBILDA_1150  = SimulatedMotor(0.01, 0.000179702360396, 0.0969370938832, 1.30434782609, 0.01)
    }

}

fun main() {
    val pivot0 = SimulatedMotor.GOBILDA_435
    val pivot1 = SimulatedMotor.GOBILDA_435

    // start at free speed
    //testMotor.state.set(0, 0, 435.0 * 2.0 * PI)
    //testMotor.state.set(1, 0, 0.2)

    var counter: Int = 0
    val p = 0.1
    val g = 0.15
    var targetAngle = PI / 2.0

    var server = RLOGServer()
    Logger.addDataReceiver(server)
    Logger.recordMetadata("opMode name", "aKit test")
    Logger.start()
    Logger.periodicAfterUser(0.0, 0.0)
    Logger.start()

    while (true) {
    //while ((abs(AngleUnit.normalizeRadians(pivot0.angle - targetAngle)) > 0.01 || abs(pivot0.rpm) > 0.5)) {
        Logger.periodicBeforeUser()
        // m x g x mass x cos(angle)
        val load = 0.1 * 9.81 * 1.0 * cos(pivot0.angle)

        // halve load bc torque
        pivot0.load = load / 2.0
        pivot1.load = load / 2.0

        pivot0.update(0.001)
        pivot1.update(0.001)
        Logger.recordOutput("Arm position (rad)", AngleUnit.normalizeRadians(pivot0.angle))
        Logger.recordOutput("Target angle (rad)", targetAngle)
        Logger.recordOutput("Error (rad)", AngleUnit.normalizeRadians(pivot0.angle - targetAngle))
        //println(AngleUnit.normalizeRadians(pivot0.angle))

        if (counter % 10 == 0) {
            var error = (targetAngle - pivot0.angle % (2.0 * PI))
            //println("initial error: $error")
            if (error < -PI) {
                error += 2.0 * PI
            } else if (error > PI) {
                error -= 2.0 * PI
            }
            println("error: $error")
            val power = error * p + cos(pivot0.angle) * g
            Logger.recordOutput("Power", power)
            pivot0.power = power
            pivot1.power = power
        }

        if (counter % 1000 == 0) {
            if (targetAngle < 0.1) {
                targetAngle = PI / 2.0
            } else {
                targetAngle = 0.0
            }
        }

        counter += 1

        sleep(5)

        Logger.periodicAfterUser(0.0, 0.0)
    }

    Logger.end()
}