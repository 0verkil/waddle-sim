package org.necessaryevil.waddle.ftc.hardware

import org.necessaryevil.waddle.physics.common.SimulatedMotor
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorController
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareDevice
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType
import kotlin.math.PI

class SimulatedDcMotor(val port: Int, val internalMotor: SimulatedMotor, val motorType: MotorType = MotorType.GENERIC) : DcMotor {

    var zpb: DcMotor.ZeroPowerBehavior = DcMotor.ZeroPowerBehavior.FLOAT
    var motorDirection: DcMotorSimple.Direction = DcMotorSimple.Direction.FORWARD

    enum class MotorType {
        GOBILDA_435,
        GOBILDA_1150,
        GENERIC
    }

    companion object {

        val cpr: Map<MotorType, Double> = mapOf(
            MotorType.GENERIC to 1.0,
            MotorType.GOBILDA_435 to 384.5,
            MotorType.GOBILDA_1150 to 145.1,

        )

        fun getTicksFromMotorType(motorType: MotorType, radians: Double): Int {
            val rot = radians / (2.0 * PI)
            return (cpr[motorType]?.times(rot))?.toInt() ?: 0
        }
    }

    override fun getMotorType(): MotorConfigurationType? {
        TODO("Not yet implemented")
    }

    override fun setMotorType(motorType: MotorConfigurationType?) {
        TODO("Not yet implemented")
    }

    override fun getController(): DcMotorController? {
        TODO("Not yet implemented")
    }

    override fun getPortNumber(): Int {
        return port
    }

    override fun setZeroPowerBehavior(zeroPowerBehavior: DcMotor.ZeroPowerBehavior?) {
        if (zeroPowerBehavior != null) {
            this.zpb = zeroPowerBehavior
        }
    }

    override fun getZeroPowerBehavior(): DcMotor.ZeroPowerBehavior? {
        return zpb
    }

    @Deprecated("ts sucks ass don't use it")
    override fun setPowerFloat() {
        zeroPowerBehavior = DcMotor.ZeroPowerBehavior.FLOAT
        power = 0.0
    }

    override fun getPowerFloat(): Boolean {
        TODO("Not yet implemented")
    }

    override fun setTargetPosition(position: Int) {
        TODO("Not yet implemented")
    }

    override fun getTargetPosition(): Int {
        TODO("Not yet implemented")
    }

    override fun isBusy(): Boolean {
        TODO("Not yet implemented")
    }

    override fun getCurrentPosition(): Int {
        return getTicksFromMotorType(motorType, internalMotor.angle)
    }

    override fun setMode(mode: DcMotor.RunMode?) {
        TODO("Not yet implemented")
    }

    override fun getMode(): DcMotor.RunMode? {
        TODO("Not yet implemented")
    }

    override fun setDirection(direction: DcMotorSimple.Direction?) {
        if (direction != null) {
            motorDirection = direction
        }
    }

    override fun getDirection(): DcMotorSimple.Direction? {
        return motorDirection
    }

    override fun setPower(power: Double) {
        internalMotor.power = if (motorDirection == DcMotorSimple.Direction.FORWARD) power else -power
    }

    override fun getPower(): Double {
        return internalMotor.power
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
}