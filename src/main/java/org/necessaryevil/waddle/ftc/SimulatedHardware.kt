package org.necessaryevil.waddle.ftc

import org.necessaryevil.waddle.ftc.hardware.SimulatedDcMotor
import org.necessaryevil.waddle.ftc.hardware.SimulatedLynxModule
import org.necessaryevil.waddle.ftc.hardware.SimulatedServo
import org.necessaryevil.waddle.ftc.hardware.SimulatedVoltageSensor
import org.necessaryevil.waddle.physics.common.SimulatedMotor
import org.necessaryevil.waddle.physics.common.SimulationObject
import com.qualcomm.hardware.lynx.LynxModule
import com.qualcomm.robotcore.hardware.AnalogInput
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DigitalChannel
import com.qualcomm.robotcore.hardware.HardwareDevice
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.I2cDevice
import com.qualcomm.robotcore.hardware.I2cDeviceSynch
import com.qualcomm.robotcore.hardware.Servo
import com.qualcomm.robotcore.hardware.VoltageSensor

/**
 * Class to handle simulation hardware updates. You must build out the hardwaremap and the hardware instances yourself.
 */
object SimulatedHardware {

    val hardwareMap: HardwareMap = EmulatedHardwareMap()
    private val simulationObjects = arrayListOf<SimulationObject>()

    init {
        addHardwareDevice("Control Hub", SimulatedLynxModule(true)) // control hub
        addHardwareDevice("Expansion Hub", SimulatedLynxModule(false)) // expansion hub
        addHardwareDevice("Voltage Sensor", SimulatedVoltageSensor()) // voltage sensor
    }

    fun <T : HardwareDevice> addHardwareDevice(deviceName: String, device: T): T {

        when (device) {
            is DcMotor -> hardwareMap.dcMotor.put(deviceName, device)
            is SimulatedServo -> { hardwareMap.servo.put(deviceName, device); addSimulationObject(device); }
            is Servo -> hardwareMap.servo.put(deviceName, device)
            is AnalogInput -> hardwareMap.analogInput.put(deviceName, device)
            is DigitalChannel -> hardwareMap.digitalChannel.put(deviceName, device)
            is I2cDeviceSynch -> hardwareMap.i2cDeviceSynch.put(deviceName, device)
            is I2cDevice -> hardwareMap.i2cDevice.put(deviceName, device)
            is VoltageSensor -> hardwareMap.voltageSensor.put(deviceName, device)
            is LynxModule -> hardwareMap.put(deviceName, device)
            else -> hardwareMap.put(deviceName, device)
        }

        return device

    }

    fun addMotor(name: String, motor: SimulatedMotor, motorType: SimulatedDcMotor.MotorType = SimulatedDcMotor.MotorType.GENERIC): SimulatedMotor {
        addHardwareDevice(name, SimulatedDcMotor(0, motor, motorType))
        addSimulationObject(motor)
        return motor
    }

    fun <T : SimulationObject> addSimulationObject(obj: T): T {
        simulationObjects.add(obj)
        return obj
    }

    fun update(dt: Double) {
        simulationObjects.forEach { it.update(dt) }
    }
}

class EmulatedHardwareMap : HardwareMap(null, null) {

    /**
     * Retrieves the (first) device with the indicated name which is also an instance of the
     * indicated class or interface. If no such device is found, null is returned.
     *
     *
     * This is not commonly used; [.get] is the usual method for retrieving items from
     * the map.
     *
     *
     * If the device has not already been initialized, calling this method will initialize it, which
     * may take some time. As a result, you should ONLY call this method during the Init phase of your
     * OpMode.
     *
     * @see .get
     */
    override fun <T : Any?> tryGet(classOrInterface: Class<out T?>?, deviceName: String?): T? {
        var deviceName = deviceName
        synchronized(lock) {
            deviceName = deviceName?.trim { it <= ' ' }
            val list = allDevicesMap[deviceName]
            var result: T? = null

            if (list != null) {
                for (device in list) {
                    if (classOrInterface?.isInstance(device) == true) {
                        //initializeDeviceIfNecessary(device)
                        // (it's not necessary.)
                        result = classOrInterface?.cast(device)
                        break
                    }
                }
            }

            // so actually they're not gonna get a warning about this because it uses android AppUtil

            // Show a warning if the user tried to get the BNO055 IMU when a BHI260 IMU is configured on
            // a Control Hub (Expansion Hubs never have a BHI260)
            /*if (Device.isRevControlHub() && result == null &&
                (classOrInterface.getSimpleName().contains("BNO055") ||
                        classOrInterface.getSimpleName().contains("LynxEmbeddedIMU"))
            ) {
                // Unfortunately, we can't check which IMU is physically present from RobotCore.
                // Instead, we'll just check if the hardware map contains a BHI260 IMU, but no BNO055 IMU.
                var foundBno055 = false
                var foundBhi260 = false
                for (device in this) {
                    val className = device.javaClass.getSimpleName()
                    if (className.contains("BHI260")) {
                        foundBhi260 = true
                    } else if (className.contains("BNO055") || className == "LynxEmbeddedIMU") {
                        foundBno055 = true
                    }
                }

                if (foundBhi260 && !foundBno055) {
                    RobotLog.addGlobalWarningMessage(
                        "You attempted to use the BNO055 interface when " +
                                "only a BHI260AP IMU is configured. This Control Hub contains a BHI260AP IMU, " +
                                "and you need to update your code to use the IMU interface rather than the BNO055 interface."
                    )
                }
            }*/

            // Show a warning if the user previously retrieved an instance of a different driver for the
            // same physical hardware device
            if (result != null && devicesWithMultipleDriversMap.containsKey(deviceName)) {
                for (configEntry in devicesWithMultipleDriversMap[deviceName]!!) {
                    if (configEntry.warnIfOtherDriverHasBeenRetrieved(
                            result as HardwareDevice,
                            deviceName
                        )
                    ) {
                        break
                    }
                }
            }
            return result
        }
    }

}