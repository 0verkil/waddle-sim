package com.necessaryevil.waddle.ftc.hardware

import com.qualcomm.robotcore.hardware.HardwareDevice
import com.qualcomm.robotcore.hardware.HardwareDeviceHealth
import com.qualcomm.robotcore.hardware.I2cAddr
import com.qualcomm.robotcore.hardware.I2cDeviceSynchSimple
import com.qualcomm.robotcore.hardware.I2cWaitControl
import com.qualcomm.robotcore.hardware.TimestampedData

class EmulatedI2cDeviceSynch : I2cDeviceSynchSimple {

    override fun enableWriteCoalescing(enable: Boolean) {

    }

    override fun setI2cAddress(newAddress: I2cAddr?) {

    }

    override fun read8(): Byte {
        TODO("Not yet implemented")
    }

    override fun read8(ireg: Int): Byte {
        TODO("Not yet implemented")
    }

    override fun read(creg: Int): ByteArray? {
        TODO("Not yet implemented")
    }

    override fun read(ireg: Int, creg: Int): ByteArray? {
        TODO("Not yet implemented")
    }

    override fun readTimeStamped(creg: Int): TimestampedData? {
        TODO("Not yet implemented")
    }

    override fun readTimeStamped(
        ireg: Int,
        creg: Int
    ): TimestampedData? {
        TODO("Not yet implemented")
    }

    override fun write8(bVal: Int) {
        TODO("Not yet implemented")
    }

    override fun write8(ireg: Int, bVal: Int) {
        TODO("Not yet implemented")
    }

    override fun write(data: ByteArray?) {
        TODO("Not yet implemented")
    }

    override fun write(ireg: Int, data: ByteArray?) {
        TODO("Not yet implemented")
    }

    override fun write8(
        bVal: Int,
        waitControl: I2cWaitControl?
    ) {
        TODO("Not yet implemented")
    }

    override fun write8(
        ireg: Int,
        bVal: Int,
        waitControl: I2cWaitControl?
    ) {
        TODO("Not yet implemented")
    }

    override fun write(
        data: ByteArray?,
        waitControl: I2cWaitControl?
    ) {
        TODO("Not yet implemented")
    }

    override fun write(
        ireg: Int,
        data: ByteArray?,
        waitControl: I2cWaitControl?
    ) {
        TODO("Not yet implemented")
    }

    override fun waitForWriteCompletions(waitControl: I2cWaitControl?) {
        TODO("Not yet implemented")
    }

    override fun isWriteCoalescingEnabled(): Boolean {
        TODO("Not yet implemented")
    }

    override fun isArmed(): Boolean {
        TODO("Not yet implemented")
    }

    override fun setI2cAddr(i2cAddr: I2cAddr?) {
        TODO("Not yet implemented")
    }

    override fun getI2cAddr(): I2cAddr? {
        TODO("Not yet implemented")
    }

    override fun setLogging(enabled: Boolean) {
        TODO("Not yet implemented")
    }

    override fun getLogging(): Boolean {
        TODO("Not yet implemented")
    }

    override fun setLoggingTag(loggingTag: String?) {
        TODO("Not yet implemented")
    }

    override fun getLoggingTag(): String? {
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

    override fun setHealthStatus(status: HardwareDeviceHealth.HealthStatus?) {
        TODO("Not yet implemented")
    }

    override fun getHealthStatus(): HardwareDeviceHealth.HealthStatus? {
        TODO("Not yet implemented")
    }

    override fun getI2cAddress(): I2cAddr? {
        TODO("Not yet implemented")
    }

    override fun setUserConfiguredName(name: String?) {
        TODO("Not yet implemented")
    }

    override fun getUserConfiguredName(): String? {
        TODO("Not yet implemented")
    }

}