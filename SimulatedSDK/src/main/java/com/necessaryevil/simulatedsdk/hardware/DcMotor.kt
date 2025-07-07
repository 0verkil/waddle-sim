package com.necessaryevil.simulatedsdk.hardware

import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorController
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareDevice
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType

class DcMotorWrapper(val port: Int) : DcMotor {

    var zpb : DcMotor.ZeroPowerBehavior = DcMotor.ZeroPowerBehavior.FLOAT

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
        TODO("Not yet implemented")
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
        TODO("Not yet implemented")
    }

    override fun setMode(mode: DcMotor.RunMode?) {
        TODO("Not yet implemented")
    }

    override fun getMode(): DcMotor.RunMode? {
        TODO("Not yet implemented")
    }

    override fun setDirection(direction: DcMotorSimple.Direction?) {
        TODO("Not yet implemented")
    }

    override fun getDirection(): DcMotorSimple.Direction? {
        TODO("Not yet implemented")
    }

    override fun setPower(power: Double) {
        TODO("Not yet implemented")
    }

    override fun getPower(): Double {
        TODO("Not yet implemented")
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

class DcMotorModel(val rpm: Int, ) {

}