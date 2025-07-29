package com.necessaryevil.simulatedsdk.ftc

import com.necessaryevil.simulatedsdk.ftc.hardware.DcMotorSimulated
import com.necessaryevil.simulatedsdk.physics.common.SimulatedMotor
import com.necessaryevil.simulatedsdk.physics.common.SimulationObject
import com.qualcomm.robotcore.hardware.AnalogInput
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DigitalChannel
import com.qualcomm.robotcore.hardware.HardwareDevice
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.I2cDevice
import com.qualcomm.robotcore.hardware.I2cDeviceSynch
import com.qualcomm.robotcore.hardware.Servo
import com.qualcomm.robotcore.hardware.VoltageSensor
import java.lang.reflect.GenericArrayType

/**
 * Class to handle simulation hardware updates. You must build out the hardwaremap and the hardware instances yourself.
 */
object SimulatedHardware {

    val hardwareMap: HardwareMap = HardwareMap(null, null)
    private val simulationObjects = arrayListOf<SimulationObject>()

    inline fun <reified T : HardwareDevice> addHardwareDevice(deviceName: String, device: T) {

        when (device) {
            is DcMotor -> hardwareMap.dcMotor.put(deviceName, device)
            is Servo -> hardwareMap.servo.put(deviceName, device)
            is AnalogInput -> hardwareMap.analogInput.put(deviceName, device)
            is DigitalChannel -> hardwareMap.digitalChannel.put(deviceName, device)
            is I2cDeviceSynch -> hardwareMap.i2cDeviceSynch.put(deviceName, device)
            is I2cDevice -> hardwareMap.i2cDevice.put(deviceName, device)
            is VoltageSensor -> hardwareMap.voltageSensor.put(deviceName, device)
            else -> hardwareMap.put(deviceName, device)
        }

    }

    fun addMotor(name: String, motor: SimulatedMotor, motorType: DcMotorSimulated.MotorType = DcMotorSimulated.MotorType.GENERIC) {
        addHardwareDevice(name, DcMotorSimulated(0, motor, motorType))
        addSimulationObject(motor)
    }

    fun addSimulationObject(obj: SimulationObject) {
        simulationObjects.add(obj)
    }

    fun update(dt: Double) {
        simulationObjects.forEach { it.update(dt) }
        simulationObjects.filter { it is SimulatedMotor }.forEach { println((it as SimulatedMotor).rpm) }
    }
}