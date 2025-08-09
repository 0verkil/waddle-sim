package com.necessaryevil.simulatedsdk.ftc.hardware

import com.necessaryevil.simulatedsdk.physics.common.SimulationObject
import com.qualcomm.robotcore.hardware.HardwareDevice
import com.qualcomm.robotcore.hardware.Servo
import com.qualcomm.robotcore.hardware.ServoController
import kotlin.math.min

class SimulatedServo(
    val minDegrees: Double,
    val maxDegrees: Double,
    val maxDegreesPerSecond: Double,
    initialDegrees: Double = minDegrees,
    val port: Int = 0
) : Servo, SimulationObject {

    private var position: Double = initialDegrees
    private var targetPosition: Double = initialDegrees
    private var minScaleDegrees = minDegrees
    private var maxScaleDegrees = maxDegrees
    private var direction = Servo.Direction.FORWARD

    private val invertForDirection = { t: Double -> if (direction == Servo.Direction.FORWARD) t else 1.0 - t}
    private val lerpServoRange = { t: Double -> lerp(minDegrees, maxDegrees, invertForDirection(t)) }
    private val lerpScaledServoRange = { t: Double -> lerp(minScaleDegrees, maxScaleDegrees, invertForDirection(t)) }

    private val antilerpScaledServoRange =
        { k: Double -> invertForDirection(antilerp(minScaleDegrees, maxScaleDegrees, k)) }

    /**
     * Simulation-only method for servo-based sensors. Outputs degrees.
     */
    fun getTruePosition() = position

    override fun getController(): ServoController? {
        TODO("Not yet implemented")
    }

    override fun getPortNumber(): Int {
        return port;
    }

    override fun setDirection(direction: Servo.Direction?) {
        if (direction == null) {
            return
        }

        this.direction = direction
    }

    override fun getDirection(): Servo.Direction? {
        return direction
    }

    override fun setPosition(position: Double) {
        targetPosition = lerpScaledServoRange(position)
    }

    override fun getPosition(): Double {
        return antilerpScaledServoRange(targetPosition)
    }

    override fun scaleRange(min: Double, max: Double) {
        minScaleDegrees = lerpServoRange(min)
        maxScaleDegrees = lerpServoRange(max)
    }

    override fun getManufacturer(): HardwareDevice.Manufacturer? {
        TODO("Not yet implemented")
    }

    override fun getDeviceName(): String? {
        TODO("Not yet implemented")
    }

    override fun getConnectionInfo(): String? {
        TODO("Not yet implemented")
    }

    override fun getVersion(): Int {
        TODO("Not yet implemented")
    }

    override fun resetDeviceConfigurationForOpMode() {
        TODO("Not yet implemented")
    }

    override fun close() {
        TODO("Not yet implemented")
    }

    override fun update(dt: Double) {
        position += (targetPosition - position).coerceIn(-maxDegreesPerSecond, maxDegreesPerSecond)
    }

    fun lerp(a: Double, b: Double, t: Double): Double = (1.0 - t) * a + b * t

    fun antilerp(a: Double, b: Double, k: Double): Double = (k - a) / (b - a)

}