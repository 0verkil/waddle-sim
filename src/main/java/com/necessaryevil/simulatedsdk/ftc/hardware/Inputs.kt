package com.necessaryevil.simulatedsdk.ftc.hardware

import com.qualcomm.robotcore.hardware.AnalogInput
import com.qualcomm.robotcore.hardware.AnalogInputController
import com.qualcomm.robotcore.hardware.DigitalChannel
import com.qualcomm.robotcore.hardware.DigitalChannelController
import com.qualcomm.robotcore.hardware.HardwareDevice
import com.qualcomm.robotcore.util.SerialNumber
import kotlin.math.max

/**
 * Simulated digital channel. INPUT mode does not work. To modify signal, use `setState()`.
 */
class SimulatedDigitalChannel : DigitalChannel {
    private var state: Boolean = false;

    override fun getMode(): DigitalChannel.Mode? {
        return DigitalChannel.Mode.OUTPUT
    }

    override fun setMode(mode: DigitalChannel.Mode?) {
        return
    }

    override fun getState(): Boolean {
        return state
    }

    override fun setState(state: Boolean) {
        this.state = state
    }

    @Deprecated("Deprecated in Java")
    override fun setMode(mode: DigitalChannelController.Mode?) {
        return
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

/**
 * Simulated AnalogInput class. To modify measurement, set `voltage`.
 */
class SimulatedAnalogInput(
    @get:JvmName("voltage") var voltage: Double = 0.0,
    @get:JvmName("maxVoltage") val maxVoltage: Double = 3.3
) : AnalogInput(null, 0) {
    override fun getVoltage(): Double {
        return voltage
    }

    override fun getMaxVoltage(): Double {
        return maxVoltage
    }

    override fun resetDeviceConfigurationForOpMode() {
        // meh
    }

}