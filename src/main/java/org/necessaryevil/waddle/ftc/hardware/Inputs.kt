package org.necessaryevil.waddle.ftc.hardware

import com.qualcomm.robotcore.hardware.AnalogInput
import com.qualcomm.robotcore.hardware.DigitalChannel
import com.qualcomm.robotcore.hardware.DigitalChannelController
import com.qualcomm.robotcore.hardware.HardwareDevice

/**
 * Simulated digital channel. INPUT mode does not work. To modify signal, set `stateSupplier`.
 */
class SimulatedDigitalChannel : DigitalChannel {
    private var state: Boolean = false
        @JvmName("state")
        get() = stateSupplier.invoke();

    var stateSupplier: Function0<Boolean> = { false }

    override fun getMode(): DigitalChannel.Mode? {
        return DigitalChannel.Mode.OUTPUT
    }

    override fun setMode(mode: DigitalChannel.Mode?) {
        return
    }

    override fun getState(): Boolean {
        return state
    }

    /**
     * Sets the state to the target state. USE ONLY IF THIS IS AN OUTPUT DIGITAL CHANNEL. Do not use for sensors.
     */
    override fun setState(state: Boolean) {
        this.stateSupplier = { state }
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
 * Simulated AnalogInput class. To modify measurement, set `voltageSupplier`.
 */
class SimulatedAnalogInput(
    voltage: Double = 0.0,
    @get:JvmName("maxVoltage") val maxVoltage: Double = 3.3
) : AnalogInput(null, 0) {

    var voltageSupplier: Function0<Double> = { voltage }

    override fun getVoltage(): Double {
        return voltageSupplier.invoke()
    }

    override fun getMaxVoltage(): Double {
        return maxVoltage
    }

    override fun resetDeviceConfigurationForOpMode() {
        // meh
    }

}